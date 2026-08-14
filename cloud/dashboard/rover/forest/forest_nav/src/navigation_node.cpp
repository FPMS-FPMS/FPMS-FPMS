// navigation_node.cpp -- goal manager for the forest stack.
//
// C++ because the upstream project (github.com/Ellakiya15/ros2-dynamic-obstacle-
// avoidance-bot) already has this node in C++ doing the same job: read a target
// (x, y, yaw), send NavigateToPose, report success or failure. Same language and
// same shape so it drops into that repo's bot_nav package.
//
// STATUS: IMPLEMENTED. NOT COMPILED, NOT RUN -- no ROS 2 build and no forest
// rover exists in this project.
//
// ADDED OVER THE UPSTREAM NODE, each for a forest reason:
//  * watches /tracked_obstacles and RE-ISSUES the goal when a DYNAMIC obstacle
//    is predicted across the route. Nav2's BT replans on its own schedule --
//    fine for a chair, too slow for something walking.
//  * distinguishes "blocked by something moving" from "blocked by terrain".
//    The first is answered by waiting; spinning to recover from a moving
//    obstacle re-observes the same thing from a worse pose.
//  * goals are sent in `odom`. See the localisation note at the bottom of
//    nav2_forest.yaml: there is no trustworthy map frame in a forest.

#include <chrono>
#include <cmath>
#include <memory>
#include <string>

#include "forest_msgs/msg/tracked_obstacles.hpp"
#include "geometry_msgs/msg/pose_stamped.hpp"
#include "nav2_msgs/action/navigate_to_pose.hpp"
#include "rclcpp/rclcpp.hpp"
#include "rclcpp_action/rclcpp_action.hpp"

using NavigateToPose = nav2_msgs::action::NavigateToPose;
using GoalHandle = rclcpp_action::ClientGoalHandle<NavigateToPose>;
using namespace std::chrono_literals;

namespace forest_nav
{

class ForestNavigationNode : public rclcpp::Node
{
public:
  ForestNavigationNode()
  : rclcpp::Node("forest_navigation_node")
  {
    frame_ = declare_parameter("goal_frame", std::string("odom"));
    // How close a predicted DYNAMIC obstacle must come before this node
    // intervenes. Generous: intervening late is the failure that matters, and a
    // false intervention costs a replan, not a collision.
    intercept_m_ = declare_parameter("intercept_radius_m", 1.50);
    // Never intervene more often than this. A node that cancels and re-sends on
    // every scan makes no progress -- it spends the mission in the action
    // server's goal-acceptance handshake.
    min_interval_s_ = declare_parameter("min_intervene_interval_s", 3.0);

    client_ = rclcpp_action::create_client<NavigateToPose>(this, "navigate_to_pose");
    obstacles_sub_ = create_subscription<forest_msgs::msg::TrackedObstacles>(
      "tracked_obstacles", rclcpp::SensorDataQoS(),
      std::bind(&ForestNavigationNode::onObstacles, this, std::placeholders::_1));
    goal_sub_ = create_subscription<geometry_msgs::msg::PoseStamped>(
      "forest_goal", 10,
      std::bind(&ForestNavigationNode::onGoal, this, std::placeholders::_1));
    RCLCPP_INFO(get_logger(),
      "forest_navigation_node: goals in '%s', intercept radius %.2f m",
      frame_.c_str(), intercept_m_);
  }

private:
  void onGoal(const geometry_msgs::msg::PoseStamped::SharedPtr msg)
  {
    if (!client_->wait_for_action_server(5s)) {
      RCLCPP_ERROR(get_logger(), "navigate_to_pose action server is not up");
      return;
    }
    goal_ = *msg;
    goal_.header.frame_id = frame_;
    have_goal_ = true;
    send();
  }

  void send()
  {
    NavigateToPose::Goal g;
    g.pose = goal_;
    rclcpp_action::Client<NavigateToPose>::SendGoalOptions opts;
    opts.result_callback = [this](const GoalHandle::WrappedResult & r) {
      switch (r.code) {
        case rclcpp_action::ResultCode::SUCCEEDED:
          RCLCPP_INFO(get_logger(), "goal reached");
          have_goal_ = false;
          break;
        case rclcpp_action::ResultCode::ABORTED:
          // ABORTED here usually means the controller could not find a way
          // through -- in a forest, most often a gap measured as too tight.
          // That is a real answer and it is reported as one: this node does NOT
          // silently retry, because a retry that keeps failing looks to an
          // operator like a rover doing nothing.
          RCLCPP_WARN(get_logger(), "goal ABORTED by the navigator");
          have_goal_ = false;
          break;
        case rclcpp_action::ResultCode::CANCELED:
          RCLCPP_INFO(get_logger(), "goal canceled");
          break;
        default:
          RCLCPP_WARN(get_logger(), "goal ended in an unknown state");
          have_goal_ = false;
      }
    };
    handle_future_ = client_->async_send_goal(g, opts);
  }

  void onObstacles(const forest_msgs::msg::TrackedObstacles::SharedPtr msg)
  {
    if (!have_goal_) { return; }
    const auto now = this->now();
    if ((now - last_intervene_).seconds() < min_interval_s_) { return; }

    for (const auto & o : msg->obstacles) {
      if (o.classification != forest_msgs::msg::TrackedObstacle::DYNAMIC) {
        continue;
      }
      // Where will it be at the end of the controller's horizon, and does that
      // land near the route? Deliberately a crude test against the straight
      // line, not against Nav2's planned path: asking for that path here would
      // make this node a second planner holding a stale copy of the first one's
      // answer.
      const double px = o.position.x + o.velocity.x * 2.0;
      const double py = o.position.y + o.velocity.y * 2.0;
      const double d = std::hypot(px - goal_.pose.position.x,
                                  py - goal_.pose.position.y);
      if (d > intercept_m_ && std::hypot(px, py) > intercept_m_) { continue; }

      RCLCPP_INFO(get_logger(),
        "dynamic obstacle %u predicted across the route (%.2f m); re-issuing "
        "the goal so the planner solves against the swept volume", o.id, d);
      last_intervene_ = now;
      // Re-ISSUE, not cancel-and-abandon. The dynamic layer has already stamped
      // the swept volume; a fresh goal makes the planner re-solve against it
      // now rather than whenever the BT would have got round to it.
      send();
      return;
    }
  }

  std::string frame_;
  double intercept_m_{1.5}, min_interval_s_{3.0};
  bool have_goal_{false};
  geometry_msgs::msg::PoseStamped goal_;
  rclcpp::Time last_intervene_{0, 0, RCL_ROS_TIME};
  std::shared_future<GoalHandle::SharedPtr> handle_future_;
  rclcpp_action::Client<NavigateToPose>::SharedPtr client_;
  rclcpp::Subscription<forest_msgs::msg::TrackedObstacles>::SharedPtr obstacles_sub_;
  rclcpp::Subscription<geometry_msgs::msg::PoseStamped>::SharedPtr goal_sub_;
};

}  // namespace forest_nav

int main(int argc, char ** argv)
{
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<forest_nav::ForestNavigationNode>());
  rclcpp::shutdown();
  return 0;
}
