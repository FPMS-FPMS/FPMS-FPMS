// dynamic_obstacle_layer.hpp -- see dynamic_obstacle_layer.cpp for the design
// and for the honest status note (implemented, never compiled, never run).
#ifndef FOREST_NAV__DYNAMIC_OBSTACLE_LAYER_HPP_
#define FOREST_NAV__DYNAMIC_OBSTACLE_LAYER_HPP_

#include <mutex>
#include <string>
#include <utility>
#include <vector>

#include "forest_msgs/msg/tracked_obstacle.hpp"
#include "forest_msgs/msg/tracked_obstacles.hpp"
#include "nav2_costmap_2d/costmap_layer.hpp"
#include "nav2_costmap_2d/layered_costmap.hpp"
#include "rclcpp/rclcpp.hpp"

namespace forest_nav
{

// A CostmapLayer (not a plain Layer) because this owns its own costmap buffer
// and combines it into the master with MAX -- it must be able to add danger
// without ever being able to clear another layer's lethal cell.
class DynamicObstacleLayer : public nav2_costmap_2d::CostmapLayer
{
public:
  DynamicObstacleLayer() = default;

  void onInitialize() override;
  void updateBounds(
    double robot_x, double robot_y, double robot_yaw,
    double * min_x, double * min_y, double * max_x, double * max_y) override;
  void updateCosts(
    nav2_costmap_2d::Costmap2D & master,
    int min_i, int min_j, int max_i, int max_j) override;
  void reset() override;
  bool isClearable() override { return true; }

private:
  void stampPolygon(
    const forest_msgs::msg::TrackedObstacle & t, double ox, double oy,
    double pad, unsigned char cost,
    double * min_x, double * min_y, double * max_x, double * max_y);
  void stampDisc(
    double cx, double cy, double r, unsigned char cost,
    double * min_x, double * min_y, double * max_x, double * max_y);
  static bool insidePolygon(
    const std::vector<std::pair<double, double>> & p, double x, double y);
  static double distanceToPolygon(
    const std::vector<std::pair<double, double>> & p, double x, double y);

  std::string topic_, global_frame_;
  double horizon_s_{2.0}, pred_step_s_{0.25}, growth_m_s_{0.25};
  double static_speed_{0.15}, decay_s_{3.0}, min_conf_{0.30};
  int current_cost_{254}, predicted_cost_{200}, combination_method_{1};
  bool rolling_{false}, have_{false};

  std::mutex mutex_;
  forest_msgs::msg::TrackedObstacles latest_;
  std::vector<unsigned int> stamped_;
  rclcpp::Clock::SharedPtr clock_;
  rclcpp::Logger logger_{rclcpp::get_logger("DynamicObstacleLayer")};
  rclcpp::Subscription<forest_msgs::msg::TrackedObstacles>::SharedPtr sub_;
};

}  // namespace forest_nav

#endif  // FOREST_NAV__DYNAMIC_OBSTACLE_LAYER_HPP_
