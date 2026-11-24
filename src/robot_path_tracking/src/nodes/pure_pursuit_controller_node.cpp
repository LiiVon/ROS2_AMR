/**
 * @file pure_pursuit_controller_node.cpp
 * @brief 纯追踪控制器节点（节点入口层）。
 * @details 所属包：`robot_path_tracking`；所在层次：节点入口层。
 */

#include <algorithm>
#include <cmath>
#include <limits>
#include <memory>
#include <string>

#include "geometry_msgs/msg/twist.hpp"
#include "nav_msgs/msg/odometry.hpp"
#include "nav_msgs/msg/path.hpp"
#include "rclcpp/rclcpp.hpp"
#include "robot_interfaces_core/msg/tracking_error.hpp"
#include "robot_path_tracking/core/tracking_math.hpp"
#include "tf2/utils.h"
#include "tf2_geometry_msgs/tf2_geometry_msgs.hpp"

#include "robot_utils/config/topic_names.hpp"

namespace topics = robot_utils::topics;

class PurePursuitControllerNode final : public rclcpp::Node {
 public:
  PurePursuitControllerNode() : Node("pure_pursuit_controller_node") {
    lookahead_distance_ = declare_parameter<double>("lookahead_distance", 0.8);
    target_speed_ = declare_parameter<double>("target_speed", 0.35);
    max_angular_speed_ = declare_parameter<double>("max_angular_speed", 1.2);
    goal_tolerance_ = std::max(0.0, declare_parameter<double>("goal_tolerance", 0.10));
    odom_timeout_ms_ = declare_parameter<int>("odom_timeout_ms", 300);
    odom_subscription_ = create_subscription<nav_msgs::msg::Odometry>(
        topics::kOdom, 10, [this](nav_msgs::msg::Odometry::SharedPtr msg) {
          odom_ = msg;
          last_odom_stamp_ = now();
          have_odom_ = true;
        });
    path_subscription_ = create_subscription<nav_msgs::msg::Path>(
        topics::kReferencePath, 1, [this](nav_msgs::msg::Path::SharedPtr msg) { path_ = msg; });
    cmd_publisher_ = create_publisher<geometry_msgs::msg::Twist>(topics::kTrackingCmdVel, 10);
    error_publisher_ = create_publisher<robot_interfaces_core::msg::TrackingError>(topics::kTrackingError, 10);
    timer_ = create_wall_timer(std::chrono::milliseconds(50), [this]() { Tick(); });
  }

 private:
  void Tick() {
    if (!odom_ || !path_ || path_->poses.empty()) {
      return;
    }
    // odom 断流时不得用旧位姿继续发速度；坐标系不一致时也不得盲跑。
    if (!have_odom_ || (now() - last_odom_stamp_).nanoseconds() / 1000000 > odom_timeout_ms_) {
      PublishStop();
      RCLCPP_WARN_THROTTLE(get_logger(), *get_clock(), 2000,
                           "odometry stale (> %d ms); publishing zero velocity", odom_timeout_ms_);
      return;
    }
    if (!FramesCompatible()) {
      PublishStop();
      RCLCPP_WARN_THROTTLE(get_logger(), *get_clock(), 2000,
                           "path frame '%s' differs from odom frame '%s'; skipping control",
                           path_->header.frame_id.c_str(), odom_->header.frame_id.c_str());
      return;
    }

    const auto& pose = odom_->pose.pose;
    const double yaw = tf2::getYaw(pose.orientation);
    int nearest_index = 0;
    double nearest_distance = std::numeric_limits<double>::max();
    for (size_t i = 0; i < path_->poses.size(); ++i) {
      const double dx = path_->poses[i].pose.position.x - pose.position.x;
      const double dy = path_->poses[i].pose.position.y - pose.position.y;
      const double distance = std::hypot(dx, dy);
      if (distance < nearest_distance) {
        nearest_distance = distance;
        nearest_index = static_cast<int>(i);
      }
    }

    const auto& nearest_pose = path_->poses[nearest_index].pose;
    const double path_yaw = tf2::getYaw(nearest_pose.orientation);
    const double lateral_error =
        robot_path_tracking::SignedLateralError(nearest_pose.position.x, nearest_pose.position.y,
                                                path_yaw, pose.position.x, pose.position.y);
    const auto& goal = path_->poses.back().pose.position;
    const double distance_to_goal =
        std::hypot(goal.x - pose.position.x, goal.y - pose.position.y);
    if (robot_path_tracking::IsFinalWaypointReached(
            nearest_index, path_->poses.size(), distance_to_goal, goal_tolerance_)) {
      cmd_publisher_->publish(geometry_msgs::msg::Twist{});
      PublishError(nearest_index, lateral_error, 0.0, 0.0);
      return;
    }

    const auto target_index =
        SelectTargetIndex(nearest_index, pose.position.x, pose.position.y, yaw);
    const auto& target = path_->poses[target_index].pose.position;
    const double dx = target.x - pose.position.x;
    const double dy = target.y - pose.position.y;
    const double heading_error = robot_path_tracking::NormalizeAngle(std::atan2(dy, dx) - yaw);
    // 纯追踪曲率半径取“到目标点的实际距离”，而不是配置的预瞄距离：
    // 当目标点受路径末端限制而近于预瞄距离时，用配置值会低估曲率。
    const double target_distance = std::max(std::hypot(dx, dy), 0.05);
    const double curvature = 2.0 * std::sin(heading_error) / target_distance;

    geometry_msgs::msg::Twist cmd;
    cmd.linear.x = target_speed_;
    cmd.angular.z = std::clamp(target_speed_ * curvature, -max_angular_speed_, max_angular_speed_);
    cmd_publisher_->publish(cmd);

    PublishError(nearest_index, lateral_error, heading_error, MeasuredSpeed());
  }

  // 实测速度：取 /odom 的 twist，而不是刚下发的目标速度。
  double MeasuredSpeed() const {
    if (!odom_) {
      return 0.0;
    }
    return std::hypot(odom_->twist.twist.linear.x, odom_->twist.twist.linear.y);
  }

  bool FramesCompatible() const {
    if (!path_ || path_->header.frame_id.empty() || !odom_) {
      return true;
    }
    return path_->header.frame_id == odom_->header.frame_id;
  }

  void PublishStop() {
    cmd_publisher_->publish(geometry_msgs::msg::Twist{});
  }

  size_t SelectTargetIndex(const int nearest_index, const double x, const double y,
                           const double yaw) const {
    size_t first_forward_index = path_->poses.size();
    for (size_t i = static_cast<size_t>(nearest_index); i < path_->poses.size(); ++i) {
      const double dx = path_->poses[i].pose.position.x - x;
      const double dy = path_->poses[i].pose.position.y - y;
      if (robot_path_tracking::ForwardProjection(yaw, dx, dy) <= 0.0) {
        continue;
      }
      if (first_forward_index == path_->poses.size()) {
        first_forward_index = i;
      }
      if (std::hypot(dx, dy) >= lookahead_distance_) {
        return i;
      }
    }
    if (first_forward_index != path_->poses.size()) {
      return first_forward_index;
    }
    return path_->poses.size() - 1;
  }

  void PublishError(const int nearest_index, const double lateral_error,
                    const double heading_error, const double speed) {
    robot_interfaces_core::msg::TrackingError error;
    error.header.stamp = now();
    error.header.frame_id = path_->header.frame_id;
    error.lateral_error = lateral_error;
    error.heading_error = heading_error;
    error.target_speed = target_speed_;
    error.actual_speed = speed;
    error.nearest_path_index = nearest_index;
    error.controller_name = "pure_pursuit";
    error_publisher_->publish(error);
  }

  double lookahead_distance_;
  double target_speed_;
  double max_angular_speed_;
  double goal_tolerance_;
  int odom_timeout_ms_ = 300;
  bool have_odom_ = false;
  rclcpp::Time last_odom_stamp_{0, 0, RCL_ROS_TIME};
  nav_msgs::msg::Odometry::SharedPtr odom_;
  nav_msgs::msg::Path::SharedPtr path_;
  rclcpp::Subscription<nav_msgs::msg::Odometry>::SharedPtr odom_subscription_;
  rclcpp::Subscription<nav_msgs::msg::Path>::SharedPtr path_subscription_;
  rclcpp::Publisher<geometry_msgs::msg::Twist>::SharedPtr cmd_publisher_;
  rclcpp::Publisher<robot_interfaces_core::msg::TrackingError>::SharedPtr error_publisher_;
  rclcpp::TimerBase::SharedPtr timer_;
};

int main(int argc, char** argv) {
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<PurePursuitControllerNode>());
  rclcpp::shutdown();
  return 0;
}
