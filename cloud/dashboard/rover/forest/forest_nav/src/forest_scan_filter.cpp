// forest_scan_filter.cpp -- LiDAR conditioning for a forest, in C++.
//
// WHY C++ AND NOT PYTHON. This runs per-point on every scan: a 32-beam spinner
// at 10 Hz is ~600k points/s, and a 2D spinner at 10-20 Hz still puts a full
// pass over ~1-2k beams inside the sensor period. rclpy cannot hold that budget
// without dropping scans, and a dropped scan in a forest is a trunk the local
// controller never saw. Everything downstream of here (tracking, association)
// runs at a few Hz on a few dozen clusters and is Python for exactly the
// opposite reason.
//
// STATUS: IMPLEMENTED. The filter maths below is complete and self-contained.
// It has NEVER RUN ON HARDWARE -- there is no forest rover in this project. It
// is written against the ROS 2 / Nav2 conventions used by
// github.com/Ellakiya15/ros2-dynamic-obstacle-avoidance-bot (bot_description /
// bot_gazebo / bot_nav, C++ navigation_node, Nav2 + Gazebo), which was read
// before this was written; that repo runs an INDOOR Gazebo world with the stock
// Nav2 layers, so everything specific to trees below is new work, not ported.
//
// WHAT A FOREST BREAKS THAT AN INDOOR MAP DOES NOT
// ------------------------------------------------
//   GROUND RETURNS. Indoors the floor is flat and below the scan plane. On a
//   forest floor the rover pitches over roots and the beam hits dirt 2 m ahead,
//   which the stock obstacle layer marks as a wall directly in front. Removing
//   ground by a fixed z threshold fails on any slope, so this fits a LOCAL
//   ground height per azimuth sector instead (see kGroundSectors).
//
//   CANOPY AND UNDERGROWTH. Leaves and thin branches return, move in wind, and
//   are drivable-through or at least not worth a detour. They are separated from
//   trunks by PERSISTENCE and VERTICAL EXTENT, not by range: a trunk gives
//   returns over a tall contiguous z band at a stable azimuth, foliage gives a
//   sparse flicker.
//
//   NON-CONVEX SHAPES. A trunk is roughly convex, a root flare and a fallen limb
//   are not. Nothing here fits circles; the output keeps the raw supporting
//   points so the costmap layer can rasterise the real shape.
//
//   NO STRAIGHT WALLS TO LOCALISE AGAINST. AMCL against a static map is not
//   available in the open. This node therefore publishes in the SENSOR frame and
//   never assumes a map exists; the localisation answer is odometry + a
//   scan-to-scan estimate, which is a separate problem and is NOT solved here.
//
// SUBSCRIBES  ~/points        sensor_msgs/PointCloud2   (3D lidar)  [optional]
//             ~/scan          sensor_msgs/LaserScan     (2D lidar)  [optional]
// PUBLISHES   ~/scan_filtered sensor_msgs/LaserScan     trunk slice, for Nav2
//             ~/obstacle_points sensor_msgs/PointCloud2 surviving 3D returns
//             ~/ground_points sensor_msgs/PointCloud2   what was classified
//                                                       ground, for RViz sanity

#include <algorithm>
#include <cmath>
#include <limits>
#include <memory>
#include <string>
#include <vector>

#include "rclcpp/rclcpp.hpp"
#include "sensor_msgs/msg/laser_scan.hpp"
#include "sensor_msgs/msg/point_cloud2.hpp"
#include "sensor_msgs/point_cloud2_iterator.hpp"

namespace forest_nav
{

// Azimuth sectors used for the local ground fit. Wide enough that each holds
// enough points to estimate a height, narrow enough that a 15 degree side slope
// does not smear one sector's ground into the next.
static constexpr int kGroundSectors = 72;        // 5 degrees each

class ForestScanFilter : public rclcpp::Node
{
public:
  ForestScanFilter()
  : rclcpp::Node("forest_scan_filter")
  {
    // -- geometry of the machine -------------------------------------------
    sensor_height_m_   = declare_parameter("sensor_height_m", 0.60);
    // The band that is allowed to STOP the rover. Below it is ground and root
    // litter the chassis clears; above it is canopy the chassis drives under.
    // These two numbers are the single most important tuning in the file and
    // they are properties of the VEHICLE, not of the forest.
    trunk_z_min_m_     = declare_parameter("trunk_z_min_m", 0.15);
    trunk_z_max_m_     = declare_parameter("trunk_z_max_m", 1.40);
    // How far a return may sit above the LOCAL ground estimate and still be
    // ground. Covers root flare, leaf litter and the sensor's own range noise.
    ground_tol_m_      = declare_parameter("ground_tolerance_m", 0.12);
    // A sector whose ground estimate is this far from its neighbours' median is
    // not a slope, it is a hole or a bad fit; fall back to the neighbours.
    ground_jump_m_     = declare_parameter("ground_jump_m", 0.35);

    range_min_m_       = declare_parameter("range_min_m", 0.30);
    range_max_m_       = declare_parameter("range_max_m", 25.0);
    // SHADOW / VEIL POINTS. The classic spinning-lidar artefact: a beam that
    // grazes an edge returns a range interpolated between near and far, and
    // plants a phantom obstacle in open space between a trunk and the
    // background. Detected as an implausible range gradient between angularly
    // adjacent returns.
    shadow_angle_min_deg_ = declare_parameter("shadow_angle_min_deg", 12.0);

    // FOLIAGE REJECTION. A cell must be supported by this many points spread
    // over this much z before it is allowed into the trunk slice. One leaf at
    // 8 m subtends fewer points than a 200 mm trunk at 8 m, and this is the test
    // that separates them without pretending to classify species.
    min_support_pts_   = declare_parameter("min_support_points", 3);
    min_vertical_m_    = declare_parameter("min_vertical_extent_m", 0.20);

    out_beams_         = declare_parameter("output_beams", 720);
    output_frame_      = declare_parameter("output_frame", std::string(""));

    auto qos = rclcpp::SensorDataQoS();
    cloud_sub_ = create_subscription<sensor_msgs::msg::PointCloud2>(
      "points", qos,
      std::bind(&ForestScanFilter::onCloud, this, std::placeholders::_1));
    scan_sub_ = create_subscription<sensor_msgs::msg::LaserScan>(
      "scan", qos,
      std::bind(&ForestScanFilter::onScan, this, std::placeholders::_1));

    scan_pub_ = create_publisher<sensor_msgs::msg::LaserScan>(
      "scan_filtered", qos);
    obst_pub_ = create_publisher<sensor_msgs::msg::PointCloud2>(
      "obstacle_points", qos);
    ground_pub_ = create_publisher<sensor_msgs::msg::PointCloud2>(
      "ground_points", qos);

    RCLCPP_INFO(get_logger(),
      "forest_scan_filter: trunk band %.2f..%.2f m, ground tol %.2f m, "
      "%d output beams", trunk_z_min_m_, trunk_z_max_m_, ground_tol_m_,
      static_cast<int>(out_beams_));
  }

private:
  struct Pt { float x, y, z, range, az; };

  // ------------------------------------------------------------------ 3D
  void onCloud(const sensor_msgs::msg::PointCloud2::SharedPtr msg)
  {
    std::vector<Pt> pts;
    pts.reserve(msg->width * msg->height);
    sensor_msgs::PointCloud2ConstIterator<float> ix(*msg, "x");
    sensor_msgs::PointCloud2ConstIterator<float> iy(*msg, "y");
    sensor_msgs::PointCloud2ConstIterator<float> iz(*msg, "z");
    for (; ix != ix.end(); ++ix, ++iy, ++iz) {
      const float x = *ix, y = *iy, z = *iz;
      if (!std::isfinite(x) || !std::isfinite(y) || !std::isfinite(z)) {
        // NOT AN OBSTACLE AND NOT FREE SPACE. A non-finite return is UNKNOWN,
        // and the difference matters: writing it in as either one is how a
        // costmap acquires a wall nobody saw or clears one that is really there.
        continue;
      }
      const float r = std::hypot(x, y);
      if (r < range_min_m_ || r > range_max_m_) { continue; }
      pts.push_back({x, y, z, r, std::atan2(y, x)});
    }
    if (pts.empty()) { return; }

    // -- 1. LOCAL GROUND, PER AZIMUTH SECTOR ------------------------------
    // A single plane fit fails the moment the rover is on a side slope or a
    // root ramp, and both are the normal case here. Per-sector minima are crude
    // but they degrade gracefully: a sector with no ground return simply
    // inherits its neighbours instead of inventing a floor.
    std::vector<float> ground(kGroundSectors,
                              std::numeric_limits<float>::quiet_NaN());
    std::vector<std::vector<float>> zs(kGroundSectors);
    for (const auto & p : pts) { zs[sector(p.az)].push_back(p.z); }
    for (int s = 0; s < kGroundSectors; ++s) {
      if (zs[s].size() < 4) { continue; }
      // The 10th percentile, not the minimum: one bad low return would drag a
      // true minimum down and let the whole sector's real ground through as
      // obstacle.
      std::sort(zs[s].begin(), zs[s].end());
      ground[s] = zs[s][static_cast<size_t>(0.10 * (zs[s].size() - 1))];
    }
    smoothGround(ground);

    // -- 2. CLASSIFY --------------------------------------------------------
    std::vector<Pt> obstacles, groundPts;
    obstacles.reserve(pts.size() / 4);
    for (const auto & p : pts) {
      const float g = ground[sector(p.az)];
      const float h = std::isfinite(g) ? (p.z - g) : (p.z + sensor_height_m_);
      if (h <= ground_tol_m_)          { groundPts.push_back(p); continue; }
      if (h < trunk_z_min_m_)          { groundPts.push_back(p); continue; }
      if (h > trunk_z_max_m_)          { continue; }   // CANOPY: drive under it
      obstacles.push_back(p);
    }

    // -- 3. COLLAPSE TO A 2D TRUNK SLICE -----------------------------------
    // Nav2's obstacle layer and every local controller want a LaserScan. The
    // collapse is where foliage is finally rejected: a beam direction is only
    // emitted if enough points support it over enough vertical extent, which a
    // leaf cluster cannot satisfy and a trunk trivially can.
    const size_t n = static_cast<size_t>(out_beams_);
    std::vector<float> best(n, std::numeric_limits<float>::infinity());
    std::vector<int>   count(n, 0);
    std::vector<float> zmin(n,  std::numeric_limits<float>::infinity());
    std::vector<float> zmax(n, -std::numeric_limits<float>::infinity());
    for (const auto & p : pts) { (void)p; break; }        // (no-op, clarity)
    for (const auto & p : obstacles) {
      const size_t b = beam(p.az, n);
      count[b] += 1;
      zmin[b] = std::min(zmin[b], p.z);
      zmax[b] = std::max(zmax[b], p.z);
      best[b] = std::min(best[b], p.range);
    }

    sensor_msgs::msg::LaserScan out;
    out.header = msg->header;
    if (!output_frame_.empty()) { out.header.frame_id = output_frame_; }
    out.angle_min = static_cast<float>(-M_PI);
    out.angle_max = static_cast<float>(M_PI);
    out.angle_increment = static_cast<float>(2.0 * M_PI / n);
    out.range_min = static_cast<float>(range_min_m_);
    out.range_max = static_cast<float>(range_max_m_);
    out.ranges.assign(n, std::numeric_limits<float>::infinity());
    for (size_t b = 0; b < n; ++b) {
      if (count[b] < min_support_pts_) { continue; }
      if ((zmax[b] - zmin[b]) < min_vertical_m_) { continue; }  // foliage
      out.ranges[b] = best[b];
    }
    rejectShadowPoints(out);
    scan_pub_->publish(out);

    publishCloud(obst_pub_, msg->header, obstacles);
    publishCloud(ground_pub_, msg->header, groundPts);
  }

  // ------------------------------------------------------------------ 2D
  void onScan(const sensor_msgs::msg::LaserScan::SharedPtr msg)
  {
    // A 2D scanner has no z, so ground and canopy cannot be separated here at
    // all -- the mounting height IS the filter. Say so rather than pretending:
    // the only thing this branch can honestly do is range gating and shadow
    // rejection, and it is offered for a 2D platform, not recommended for one.
    auto out = *msg;
    for (auto & r : out.ranges) {
      if (!std::isfinite(r) || r < range_min_m_ || r > range_max_m_) {
        r = std::numeric_limits<float>::infinity();
      }
    }
    rejectShadowPoints(out);
    scan_pub_->publish(out);
  }

  // ------------------------------------------------------------- helpers
  static int sector(float az)
  {
    int s = static_cast<int>((az + M_PI) / (2.0 * M_PI) * kGroundSectors);
    return std::clamp(s, 0, kGroundSectors - 1);
  }

  static size_t beam(float az, size_t n)
  {
    long b = std::lround((az + M_PI) / (2.0 * M_PI) * static_cast<double>(n));
    if (b < 0) { b = 0; }
    if (b >= static_cast<long>(n)) { b = static_cast<long>(n) - 1; }
    return static_cast<size_t>(b);
  }

  void smoothGround(std::vector<float> & g) const
  {
    // Fill gaps from neighbours, then reject sectors that disagree with their
    // neighbours by more than a slope could explain. A sector with no ground
    // estimate must NOT default to zero: that would place the ground at sensor
    // height and classify the entire forest floor as trunk.
    const int n = static_cast<int>(g.size());
    std::vector<float> src = g;
    for (int i = 0; i < n; ++i) {
      if (std::isfinite(src[i])) { continue; }
      for (int d = 1; d < n / 2; ++d) {
        const float a = src[(i - d + n) % n], b = src[(i + d) % n];
        if (std::isfinite(a) && std::isfinite(b)) { g[i] = 0.5f * (a + b); break; }
        if (std::isfinite(a)) { g[i] = a; break; }
        if (std::isfinite(b)) { g[i] = b; break; }
      }
    }
    src = g;
    for (int i = 0; i < n; ++i) {
      const float a = src[(i - 1 + n) % n], b = src[(i + 1) % n];
      if (!std::isfinite(a) || !std::isfinite(b) || !std::isfinite(src[i])) {
        continue;
      }
      const float mid = 0.5f * (a + b);
      if (std::fabs(src[i] - mid) > ground_jump_m_) { g[i] = mid; }
    }
  }

  void rejectShadowPoints(sensor_msgs::msg::LaserScan & s) const
  {
    // A return whose line to its angular neighbour is nearly along the beam is
    // an edge artefact, not a surface. Left in, it puts a phantom obstacle in
    // the gap between two trunks -- which is exactly where the route wants to
    // go, so this is not cosmetic.
    const double thr = shadow_angle_min_deg_ * M_PI / 180.0;
    const size_t n = s.ranges.size();
    if (n < 3) { return; }
    std::vector<uint8_t> drop(n, 0);
    for (size_t i = 1; i < n; ++i) {
      const float r0 = s.ranges[i - 1], r1 = s.ranges[i];
      if (!std::isfinite(r0) || !std::isfinite(r1)) { continue; }
      const double dphi = s.angle_increment;
      const double num = r1 * std::sin(dphi);
      const double den = r0 - r1 * std::cos(dphi);
      const double beta = std::fabs(std::atan2(num, den));
      if (beta < thr) { drop[r0 < r1 ? i : i - 1] = 1; }
    }
    for (size_t i = 0; i < n; ++i) {
      if (drop[i]) { s.ranges[i] = std::numeric_limits<float>::infinity(); }
    }
  }

  void publishCloud(
    const rclcpp::Publisher<sensor_msgs::msg::PointCloud2>::SharedPtr & pub,
    const std_msgs::msg::Header & hdr, const std::vector<Pt> & pts) const
  {
    if (pub->get_subscription_count() == 0) { return; }   // do not pay for RViz
    sensor_msgs::msg::PointCloud2 c;
    c.header = hdr;
    c.height = 1;
    c.width = static_cast<uint32_t>(pts.size());
    sensor_msgs::PointCloud2Modifier mod(c);
    mod.setPointCloud2FieldsByString(1, "xyz");
    mod.resize(pts.size());
    sensor_msgs::PointCloud2Iterator<float> ox(c, "x"), oy(c, "y"), oz(c, "z");
    for (const auto & p : pts) {
      *ox = p.x; *oy = p.y; *oz = p.z;
      ++ox; ++oy; ++oz;
    }
    pub->publish(c);
  }

  double sensor_height_m_, trunk_z_min_m_, trunk_z_max_m_, ground_tol_m_;
  double ground_jump_m_, range_min_m_, range_max_m_, shadow_angle_min_deg_;
  double min_vertical_m_;
  int64_t min_support_pts_, out_beams_;
  std::string output_frame_;

  rclcpp::Subscription<sensor_msgs::msg::PointCloud2>::SharedPtr cloud_sub_;
  rclcpp::Subscription<sensor_msgs::msg::LaserScan>::SharedPtr scan_sub_;
  rclcpp::Publisher<sensor_msgs::msg::LaserScan>::SharedPtr scan_pub_;
  rclcpp::Publisher<sensor_msgs::msg::PointCloud2>::SharedPtr obst_pub_;
  rclcpp::Publisher<sensor_msgs::msg::PointCloud2>::SharedPtr ground_pub_;
};

}  // namespace forest_nav

int main(int argc, char ** argv)
{
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<forest_nav::ForestScanFilter>());
  rclcpp::shutdown();
  return 0;
}
