// dynamic_obstacle_layer.cpp -- a Nav2 costmap layer for MOVING, NON-CONVEX
// obstacles in a forest.
//
// WHY C++: nav2_costmap_2d loads layers through pluginlib, and pluginlib loads
// C++ shared objects. There is no rclpy path to a costmap layer at all, so this
// is not a preference. It also runs inside the costmap update loop at 5-20 Hz
// over every cell it touches, which is the wrong place for an interpreter.
//
// STATUS: IMPLEMENTED (the layer logic, the swept-volume prediction, the
// non-convex rasterisation and the decay are all written out). NOT COMPILED and
// NOT RUN -- this project has no forest rover and no ROS 2 build here. Treat the
// CMake/plugin XML as the contract it would be built against.
//
// WHERE IT SITS. The reference project
// (github.com/Ellakiya15/ros2-dynamic-obstacle-avoidance-bot) runs the STOCK
// Nav2 layer stack in Gazebo -- static, obstacle, inflation -- plus a C++
// navigation_node that sends goals. That stack is correct indoors and is kept
// here unchanged underneath. This is a FOURTH layer added on top of it, and the
// rest of the forest work is in the scan filter below it and the tracker beside
// it. Nothing in the stock stack is modified.
//
// WHAT THE STOCK STACK CANNOT DO, AND THIS DOES
// ---------------------------------------------
//   1. IT HAS NO CONCEPT OF A MOVING OBSTACLE. `ObstacleLayer` marks where a
//      thing IS and clears by ray-casting where it WAS. For a walking animal or
//      a person that produces a smear of marks trailing the real object, and the
//      planner routes into the space the object is about to occupy. This layer
//      takes tracked obstacles with VELOCITY and stamps the swept volume the
//      object will occupy over the controller's horizon, so the route is planned
//      around where the thing is GOING.
//   2. IT ASSUMES OBSTACLES ARE POINT CLOUDS, NOT SHAPES. A trunk with a root
//      flare, a fallen limb, a multi-stem clump: none is convex, and inflating a
//      convex hull around a fallen limb closes a gap the rover could drive. This
//      layer rasterises the tracked SUPPORT POLYGON as given, concave parts and
//      all.
//   3. IT CLEARS BY RAY-CASTING, WHICH A FOREST DEFEATS. Ray-casting assumes a
//      beam that reaches a surface has cleared everything in front of it. Under
//      canopy, beams are occluded by foliage constantly, so real trunks get
//      cleared by a leaf that moved. This layer clears by CONFIDENCE DECAY over
//      time instead -- an obstacle nobody has confirmed for `decay_time_` fades
//      out, and one that is still being tracked never does, however occluded.
//
// SUBSCRIBES  ~/tracked_obstacles   forest_msgs/TrackedObstacles
//                                   (published by forest_perception/trunk_tracker.py)

#include "forest_nav/dynamic_obstacle_layer.hpp"

#include <algorithm>
#include <cmath>
#include <limits>
#include <vector>

#include "nav2_costmap_2d/costmap_math.hpp"
#include "pluginlib/class_list_macros.hpp"
#include "rclcpp/rclcpp.hpp"

namespace forest_nav
{

using nav2_costmap_2d::LETHAL_OBSTACLE;
using nav2_costmap_2d::INSCRIBED_INFLATED_OBSTACLE;
using nav2_costmap_2d::NO_INFORMATION;

void DynamicObstacleLayer::onInitialize()
{
  auto node = node_.lock();
  if (!node) { throw std::runtime_error("DynamicObstacleLayer: no node"); }
  clock_ = node->get_clock();
  logger_ = node->get_logger();

  declareParameter("enabled", rclcpp::ParameterValue(true));
  declareParameter("topic", rclcpp::ParameterValue(std::string("tracked_obstacles")));
  // HOW FAR AHEAD TO STAMP A MOVING OBSTACLE. Match this to the local
  // controller's simulation horizon: predicting further than the controller
  // plans is not caution, it is a wall of cost the controller cannot reason
  // about and will simply refuse to enter.
  declareParameter("prediction_horizon_s", rclcpp::ParameterValue(2.0));
  declareParameter("prediction_step_s", rclcpp::ParameterValue(0.25));
  // Uncertainty grows with prediction time: a track's footprint is padded by
  // this per second of look-ahead, so the far end of a swept volume is fatter
  // than the near end. Without it the layer asserts a confidence about a
  // squirrel's intentions that it does not have.
  declareParameter("prediction_growth_m_per_s", rclcpp::ParameterValue(0.25));
  // Anything slower than this is treated as STATIC and stamped once at its
  // measured shape. Trunks do not move; tracking noise does.
  declareParameter("static_speed_m_s", rclcpp::ParameterValue(0.15));
  // Confidence decay replaces ray-cast clearing. See the header comment.
  declareParameter("decay_time_s", rclcpp::ParameterValue(3.0));
  declareParameter("min_confidence", rclcpp::ParameterValue(0.30));
  // Cost written for the CURRENT footprint versus the PREDICTED swept volume.
  // The prediction is deliberately NOT lethal: it must bend a route, never
  // forbid one, or a rover in a clearing with one moving animal can find the
  // whole clearing lethal and stop.
  declareParameter("current_cost", rclcpp::ParameterValue(254));
  declareParameter("predicted_cost", rclcpp::ParameterValue(200));
  declareParameter("combination_method", rclcpp::ParameterValue(1));  // MAX

  node->get_parameter(name_ + ".enabled", enabled_);
  node->get_parameter(name_ + ".topic", topic_);
  node->get_parameter(name_ + ".prediction_horizon_s", horizon_s_);
  node->get_parameter(name_ + ".prediction_step_s", pred_step_s_);
  node->get_parameter(name_ + ".prediction_growth_m_per_s", growth_m_s_);
  node->get_parameter(name_ + ".static_speed_m_s", static_speed_);
  node->get_parameter(name_ + ".decay_time_s", decay_s_);
  node->get_parameter(name_ + ".min_confidence", min_conf_);
  node->get_parameter(name_ + ".current_cost", current_cost_);
  node->get_parameter(name_ + ".predicted_cost", predicted_cost_);
  node->get_parameter(name_ + ".combination_method", combination_method_);

  global_frame_ = layered_costmap_->getGlobalFrameID();
  rolling_ = layered_costmap_->isRolling();
  matchSize();
  current_ = true;

  sub_ = node->create_subscription<forest_msgs::msg::TrackedObstacles>(
    topic_, rclcpp::SensorDataQoS(),
    [this](forest_msgs::msg::TrackedObstacles::SharedPtr msg) {
      std::lock_guard<std::mutex> lk(mutex_);
      latest_ = *msg;
      have_ = true;
    });

  RCLCPP_INFO(logger_,
    "DynamicObstacleLayer '%s': horizon %.1fs, decay %.1fs, topic '%s'",
    name_.c_str(), horizon_s_, decay_s_, topic_.c_str());
}

void DynamicObstacleLayer::updateBounds(
  double robot_x, double robot_y, double /*robot_yaw*/,
  double * min_x, double * min_y, double * max_x, double * max_y)
{
  if (!enabled_) { return; }
  if (rolling_) { updateOrigin(robot_x - getSizeInMetersX() / 2,
                               robot_y - getSizeInMetersY() / 2); }

  forest_msgs::msg::TrackedObstacles snap;
  { std::lock_guard<std::mutex> lk(mutex_); if (!have_) { return; } snap = latest_; }

  // Reset only what we touched last time. Resetting the whole layer every cycle
  // is correct but is most of the cost of the layer on a large global costmap.
  resetMaps();
  stamped_.clear();

  const rclcpp::Time now = clock_->now();
  for (const auto & t : snap.obstacles) {
    const double age = (now - rclcpp::Time(t.header.stamp)).seconds();
    // CONFIDENCE DECAY, NOT RAY-CAST CLEARING. An obstacle that has not been
    // re-observed loses confidence linearly and disappears at `decay_s_`; one
    // that is still being tracked never decays, however long it is occluded.
    // Under canopy that difference is the whole reason this layer exists.
    double conf = t.confidence;
    if (age > 0.0 && decay_s_ > 0.0) {
      conf *= std::max(0.0, 1.0 - age / decay_s_);
    }
    if (conf < min_conf_) { continue; }

    const double speed = std::hypot(t.velocity.x, t.velocity.y);
    // 1. THE CURRENT FOOTPRINT, at full cost, always.
    stampPolygon(t, 0.0, 0.0, static_cast<unsigned char>(current_cost_),
                 min_x, min_y, max_x, max_y);

    // 2. THE SWEPT VOLUME, only if it is actually moving.
    if (speed < static_speed_) { continue; }
    for (double dt = pred_step_s_; dt <= horizon_s_ + 1e-9; dt += pred_step_s_) {
      // CONSTANT VELOCITY, AND THAT IS AN ADMISSION. Over a 2 s horizon a
      // constant-velocity prediction for a walking animal is roughly right and
      // for a running one is roughly wrong -- which is why `growth_m_s_` fattens
      // the footprint with dt and why the predicted cost is not lethal. A
      // motion model that claimed to know more would be inventing intent.
      const double pad = growth_m_s_ * dt;
      // Cost falls off along the horizon so the controller prefers to pass
      // BEHIND a crossing obstacle rather than racing it -- the near-term cells
      // are dearer than the far-term ones.
      const double frac = 1.0 - 0.5 * (dt / std::max(horizon_s_, 1e-6));
      stampPolygon(t, t.velocity.x * dt, t.velocity.y * dt, pad,
                   static_cast<unsigned char>(predicted_cost_ * frac),
                   min_x, min_y, max_x, max_y);
    }
  }
}

void DynamicObstacleLayer::updateCosts(
  nav2_costmap_2d::Costmap2D & master, int i_min, int j_min, int i_max, int j_max)
{
  if (!enabled_) { return; }
  // MAX, not overwrite. This layer only ever ADDS danger: it must never erase a
  // lethal cell the static or obstacle layer put there. A layer that can clear
  // another layer's obstacle is a layer that can drive the robot into a tree
  // that the map already knew about.
  switch (combination_method_) {
    case 0: updateWithOverwrite(master, i_min, j_min, i_max, j_max); break;
    case 2: updateWithTrueOverwrite(master, i_min, j_min, i_max, j_max); break;
    default: updateWithMax(master, i_min, j_min, i_max, j_max); break;
  }
}

// ------------------------------------------------------------------ raster
void DynamicObstacleLayer::stampPolygon(
  const forest_msgs::msg::TrackedObstacle & t, double ox, double oy, double pad,
  unsigned char cost, double * min_x, double * min_y, double * max_x,
  double * max_y)
{
  // NON-CONVEX BY CONSTRUCTION. `t.polygon` is the tracker's SUPPORT outline --
  // the measured points, in order, not a hull and not a circle. A scanline fill
  // with an even-odd crossing rule renders a concave outline exactly as given,
  // so the notch between two stems of a clump stays drivable instead of being
  // filled in by a convex approximation.
  const auto & poly = t.polygon.points;
  if (poly.size() < 3) {
    // Degenerate track (too few supporting points to form an outline): fall
    // back to a disc of the tracked radius rather than dropping it. A thing seen
    // badly is still a thing.
    stampDisc(t.position.x + ox, t.position.y + oy,
              std::max(0.05, t.radius + pad), cost,
              min_x, min_y, max_x, max_y);
    return;
  }

  double lo_x = std::numeric_limits<double>::max();
  double lo_y = std::numeric_limits<double>::max();
  double hi_x = std::numeric_limits<double>::lowest();
  double hi_y = std::numeric_limits<double>::lowest();
  std::vector<std::pair<double, double>> pts;
  pts.reserve(poly.size());
  for (const auto & p : poly) {
    const double x = p.x + ox, y = p.y + oy;
    pts.emplace_back(x, y);
    lo_x = std::min(lo_x, x); hi_x = std::max(hi_x, x);
    lo_y = std::min(lo_y, y); hi_y = std::max(hi_y, y);
  }
  lo_x -= pad; lo_y -= pad; hi_x += pad; hi_y += pad;

  unsigned int mi0, mj0, mi1, mj1;
  if (!worldToMap(lo_x, lo_y, mi0, mj0) || !worldToMap(hi_x, hi_y, mi1, mj1)) {
    // Partly or wholly off this costmap. Clamp rather than discard: half a
    // fallen limb inside the window is still half a fallen limb.
    mi0 = 0; mj0 = 0;
    mi1 = getSizeInCellsX() - 1; mj1 = getSizeInCellsY() - 1;
  }

  for (unsigned int j = mj0; j <= mj1 && j < getSizeInCellsY(); ++j) {
    for (unsigned int i = mi0; i <= mi1 && i < getSizeInCellsX(); ++i) {
      double wx, wy;
      mapToWorld(i, j, wx, wy);
      if (!insidePolygon(pts, wx, wy) &&
          (pad <= 0.0 || distanceToPolygon(pts, wx, wy) > pad)) {
        continue;
      }
      const unsigned int idx = getIndex(i, j);
      if (cost > costmap_[idx] || costmap_[idx] == NO_INFORMATION) {
        costmap_[idx] = cost;
      }
      stamped_.push_back(idx);
    }
  }
  touch(lo_x, lo_y, min_x, min_y, max_x, max_y);
  touch(hi_x, hi_y, min_x, min_y, max_x, max_y);
}

void DynamicObstacleLayer::stampDisc(
  double cx, double cy, double r, unsigned char cost,
  double * min_x, double * min_y, double * max_x, double * max_y)
{
  unsigned int mi0, mj0, mi1, mj1;
  if (!worldToMap(cx - r, cy - r, mi0, mj0)) { mi0 = 0; mj0 = 0; }
  if (!worldToMap(cx + r, cy + r, mi1, mj1)) {
    mi1 = getSizeInCellsX() - 1; mj1 = getSizeInCellsY() - 1;
  }
  for (unsigned int j = mj0; j <= mj1 && j < getSizeInCellsY(); ++j) {
    for (unsigned int i = mi0; i <= mi1 && i < getSizeInCellsX(); ++i) {
      double wx, wy;
      mapToWorld(i, j, wx, wy);
      if (std::hypot(wx - cx, wy - cy) > r) { continue; }
      const unsigned int idx = getIndex(i, j);
      if (cost > costmap_[idx] || costmap_[idx] == NO_INFORMATION) {
        costmap_[idx] = cost;
      }
      stamped_.push_back(idx);
    }
  }
  touch(cx - r, cy - r, min_x, min_y, max_x, max_y);
  touch(cx + r, cy + r, min_x, min_y, max_x, max_y);
}

bool DynamicObstacleLayer::insidePolygon(
  const std::vector<std::pair<double, double>> & p, double x, double y)
{
  // Even-odd crossing rule -- the reason concavity survives. A convex test, or
  // a hull, would fill the notch between two stems and delete a real gap.
  bool in = false;
  for (size_t i = 0, j = p.size() - 1; i < p.size(); j = i++) {
    const bool straddles = (p[i].second > y) != (p[j].second > y);
    if (!straddles) { continue; }
    const double t = (y - p[i].second) / (p[j].second - p[i].second);
    if (x < p[i].first + t * (p[j].first - p[i].first)) { in = !in; }
  }
  return in;
}

double DynamicObstacleLayer::distanceToPolygon(
  const std::vector<std::pair<double, double>> & p, double x, double y)
{
  double best = std::numeric_limits<double>::max();
  for (size_t i = 0, j = p.size() - 1; i < p.size(); j = i++) {
    const double vx = p[j].first - p[i].first, vy = p[j].second - p[i].second;
    const double wx = x - p[i].first, wy = y - p[i].second;
    const double d2 = vx * vx + vy * vy;
    const double t = (d2 < 1e-12) ? 0.0
                                  : std::clamp((wx * vx + wy * vy) / d2, 0.0, 1.0);
    best = std::min(best, std::hypot(wx - t * vx, wy - t * vy));
  }
  return best;
}

void DynamicObstacleLayer::reset()
{
  resetMaps();
  std::lock_guard<std::mutex> lk(mutex_);
  have_ = false;
  latest_.obstacles.clear();
}

}  // namespace forest_nav

PLUGINLIB_EXPORT_CLASS(forest_nav::DynamicObstacleLayer, nav2_costmap_2d::Layer)
