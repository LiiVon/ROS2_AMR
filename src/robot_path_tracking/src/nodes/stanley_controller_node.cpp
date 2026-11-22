/**
 * @file stanley_controller_node.cpp
 * @brief Stanley控制器节点（节点入口层）。
 * @details 所属包：`robot_path_tracking`；所在层次：节点入口层。
 */

#include <algorithm>
#include <cmath>
#include <limits>
#include <memory>

#include "geometry_msgs/msg/twist.hpp"
#include "nav_msgs/msg/odometry.hpp"
#include "nav_msgs/msg/path.hpp"
#include "rclcpp/rclcpp.hpp"
#include "robot_interfaces_core/msg/tracking_error.hpp"
#include "robot_path_tracking/core/tracking_math.hpp"
#include "tf2/utils.h"
#include "tf2_geometry_msgs/tf2_geometry_msgs.hpp"

class StanleyControllerNode final : public rclcpp::Node {
 public:
  StanleyControllerNode() : Node("stanley_controller_node") {
    gain_ = declare_parameter<double>("gain", 1.0);
    softening_ = declare_parameter<double>("softening", 0.2);
    target_speed_ = declare_parameter<double>("target_speed", 0.35);
    max_angular_speed_ = declare_parameter<double>("max_angular_speed", 1.2);
    goal_tolerance_ = std::max(0.0, declare_parameter<double>("goal_tolerance", 0.10));
    max_steer_angle_rad_ = declare_parameter<double>("max_steer_angle_rad", 0.6);
    wheel_base_ = declare_parameter<double>("wheel_base", 0.5);
    if (!(wheel_base_ > 0.0)) {
      RCLCPP_WARN(get_logger(), "wheel_base must be > 0, using 0.5");
      wheel_base_ = 0.5;
    }
    odom_timeout_ms_ = declare_parameter<int>("odom_timeout_ms", 300);
    odom_subscription_ = create_subscription<nav_msgs::msg::Odometry>(
        "/odom", 10, [this](nav_msgs::msg::Odometry::SharedPtr msg) {
          odom_ = msg;
          last_odom_stamp_ = now();
          have_odom_ = true;
        });
    path_subscription_ = create_subscription<nav_msgs::msg::Path>(
        "/tracking/reference_path", 1, [this](nav_msgs::msg::Path::SharedPtr msg) { path_ = msg; });
    cmd_publisher_ = create_publisher<geometry_msgs::msg::Twist>("/cmd_vel/tracking", 10);
    error_publisher_ = create_publisher<robot_interfaces_core::msg::TrackingError>("/tracking/error", 10);
    timer_ = create_wall_timer(std::chrono::milliseconds(50), [this]() { Tick(); });
  }

 private:
  void Tick() {
    if (!odom_ || !path_ || path_->poses.size() < 2) {
      return;
    }
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
    const int nearest_index = NearestIndex(pose.position.x, pose.position.y);
    const auto& nearest = path_->poses[nearest_index].pose;
    const double path_yaw = tf2::getYaw(nearest.orientation);
    const double lateral_error =
        robot_path_tracking::SignedLateralError(nearest.position.x, nearest.position.y, path_yaw,
                                                pose.position.x, pose.position.y);
    const double heading_error = robot_path_tracking::NormalizeAngle(path_yaw - yaw);
    const auto& goal = path_->poses.back().pose.position;
    const double distance_to_goal =
        std::hypot(goal.x - pose.position.x, goal.y - pose.position.y);
    if (robot_path_tracking::IsFinalWaypointReached(
            nearest_index, path_->poses.size(), distance_to_goal, goal_tolerance_)) {
      cmd_publisher_->publish(geometry_msgs::msg::Twist{});
      PublishError(nearest_index, lateral_error, heading_error, 0.0);
      return;
    }

    // Stanley 输出的是“前轮转向角 δ”，不能直接当角速度发布：
    // 必须先按自行车模型换算 ω = v·tan(δ)/L，否则量纲错误、增益随速度失真。
    const double steer = heading_error + std::atan2(gain_ * lateral_error, target_speed_ + softening_);
    const double steer_angle = std::clamp(steer, -max_steer_angle_rad_, max_steer_angle_rad_);
    const double speed = target_speed_ > 0.0 ? target_speed_ : 0.0;

    geometry_msgs::msg::Twist cmd;
    cmd.linear.x = speed;
    cmd.angular.z = speed > 0.0
                        ? std::clamp(speed * std::tan(steer_angle) / wheel_base_,
                                     -max_angular_speed_, max_angular_speed_)
                        : 0.0;
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

  void PublishError(
      const int nearest_index, const double lateral_error, const double heading_error,
      const double speed) {
    robot_interfaces_core::msg::TrackingError error;
    error.header.stamp = now();
    error.header.frame_id = path_->header.frame_id;
    error.lateral_error = lateral_error;
    error.heading_error = heading_error;
    error.target_speed = target_speed_;
    error.actual_speed = speed;
    error.nearest_path_index = nearest_index;
    error.controller_name = "stanley";
    error_publisher_->publish(error);
  }

  int NearestIndex(const double x, const double y) const {
    int nearest_index = 0;
    double nearest_distance = std::numeric_limits<double>::max();
    for (size_t i = 0; i < path_->poses.size(); ++i) {
      const double dx = path_->poses[i].pose.position.x - x;
      const double dy = path_->poses[i].pose.position.y - y;
      const double distance = std::hypot(dx, dy);
      if (distance < nearest_distance) {
        nearest_distance = distance;
        nearest_index = static_cast<int>(i);
      }
    }
    return nearest_index;
  }

  double gain_;
  double softening_;
  double target_speed_;
  double max_angular_speed_;
  double goal_tolerance_;
  double wheel_base_ = 0.5;
  double max_steer_angle_rad_ = 0.6;
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
  rclcpp::spin(std::make_shared<StanleyControllerNode>());
  rclcpp::shutdown();
  return 0;
}
