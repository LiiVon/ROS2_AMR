/**
 * @file virtual_rc_node.cpp
 * @brief 虚拟遥控器节点（输入设备层）。
 * @details 所属包：`robot_teleop`；所在层次：输入设备层。
 */

#include <chrono>
#include <algorithm>
#include <cmath>
#include <memory>
#include <string>

#include "geometry_msgs/msg/twist.hpp"
#include "rclcpp/rclcpp.hpp"
#include "sensor_msgs/msg/joy.hpp"
#include "std_msgs/msg/bool.hpp"
#include "std_srvs/srv/set_bool.hpp"

using namespace std::chrono_literals;

class VirtualRcNode final : public rclcpp::Node {
 public:
  VirtualRcNode() : Node("virtual_rc_node") {
    publish_rate_hz_ = declare_parameter<double>("publish_rate_hz", 20.0);
    max_linear_mps_ = declare_parameter<double>("max_linear_mps", 0.5);
    max_angular_radps_ = declare_parameter<double>("max_angular_radps", 1.2);
    manual_takeover_ = declare_parameter<bool>("manual_takeover", false);
    command_timeout_ms_ = declare_parameter<int>("command_timeout_ms", 200);

    // 参数防御：除零与非法速率会让下面生成 NaN/Inf 指令或 0ms 定时器。
    if (!(publish_rate_hz_ >= 1.0) || !std::isfinite(publish_rate_hz_)) {
      RCLCPP_WARN(get_logger(), "publish_rate_hz must be >= 1.0, using 20.0");
      publish_rate_hz_ = 20.0;
    }
    if (!(max_linear_mps_ > 0.0) || !std::isfinite(max_linear_mps_)) {
      RCLCPP_WARN(get_logger(), "max_linear_mps must be > 0, using 0.5");
      max_linear_mps_ = 0.5;
    }
    if (!(max_angular_radps_ > 0.0) || !std::isfinite(max_angular_radps_)) {
      RCLCPP_WARN(get_logger(), "max_angular_radps must be > 0, using 1.2");
      max_angular_radps_ = 1.2;
    }
    if (command_timeout_ms_ <= 0) {
      RCLCPP_WARN(get_logger(), "command_timeout_ms must be > 0, using 200");
      command_timeout_ms_ = 200;
    }

    joy_publisher_ = create_publisher<sensor_msgs::msg::Joy>("/teleop/joy", 10);
    takeover_publisher_ = create_publisher<std_msgs::msg::Bool>("/teleop/manual_state", 10);
    teleop_publisher_ = create_publisher<geometry_msgs::msg::Twist>("/cmd_vel/teleop", 10);
    cmd_subscription_ = create_subscription<geometry_msgs::msg::Twist>(
        "/cmd_vel/virtual_rc", 10,
        [this](geometry_msgs::msg::Twist::SharedPtr msg) {
          if (!IsFinite(*msg)) {
            RCLCPP_WARN_THROTTLE(
                get_logger(), *get_clock(), 2000,
                "ignoring non-finite virtual rc command on /cmd_vel/virtual_rc");
            return;
          }
          virtual_cmd_ = *msg;
          last_command_stamp_ = now();
          have_command_ = true;
        });
    set_takeover_service_ = create_service<std_srvs::srv::SetBool>(
        "/teleop/set_manual_takeover",
        [this](const std::shared_ptr<std_srvs::srv::SetBool::Request> request,
               std::shared_ptr<std_srvs::srv::SetBool::Response> response) {
          manual_takeover_ = request->data;
          response->success = true;
          response->message = manual_takeover_ ? "manual takeover active" : "manual takeover cleared";
        });

    const auto period = std::chrono::duration<double>(1.0 / std::max(1.0, publish_rate_hz_));
    timer_ = create_wall_timer(
        std::chrono::duration_cast<std::chrono::milliseconds>(period), [this]() { Tick(); });
  }

 private:
  // deadman：超过 command_timeout_ms 未收到新指令即视为指令失效。
  // 不回发缓存指令，避免上层进程崩溃后车按最后速度持续运动。
  bool CommandFresh() const {
    if (!have_command_) {
      return false;
    }
    const auto age_ms = (now() - last_command_stamp_).nanoseconds() / 1000000;
    return age_ms <= command_timeout_ms_;
  }

  static bool IsFinite(const geometry_msgs::msg::Twist& twist) {
    return std::isfinite(twist.linear.x) && std::isfinite(twist.linear.y) &&
           std::isfinite(twist.linear.z) && std::isfinite(twist.angular.x) &&
           std::isfinite(twist.angular.y) && std::isfinite(twist.angular.z);
  }

  void Tick() {
    const bool fresh = CommandFresh();
    std_msgs::msg::Bool takeover;
    takeover.data = manual_takeover_;
    takeover_publisher_->publish(takeover);

    const geometry_msgs::msg::Twist command = fresh ? virtual_cmd_ : geometry_msgs::msg::Twist{};

    sensor_msgs::msg::Joy joy;
    joy.header.stamp = now();
    joy.header.frame_id = "virtual_rc";
    joy.axes = {
        ClampAxis(command.linear.y / max_linear_mps_),
        ClampAxis(command.linear.x / max_linear_mps_),
        ClampAxis(command.angular.z / max_angular_radps_)};
    joy.buttons = {manual_takeover_ ? 1 : 0};
    joy_publisher_->publish(joy);

    if (!manual_takeover_) {
      return;
    }
    if (!fresh) {
      RCLCPP_WARN_THROTTLE(
          get_logger(), *get_clock(), 2000,
          "virtual rc command stale (no /cmd_vel/virtual_rc within %d ms); publishing zero twist",
          command_timeout_ms_);
    }
    geometry_msgs::msg::Twist limited = command;
    limited.linear.x = Clamp(command.linear.x, -max_linear_mps_, max_linear_mps_);
    limited.linear.y = Clamp(command.linear.y, -max_linear_mps_, max_linear_mps_);
    limited.angular.z = Clamp(command.angular.z, -max_angular_radps_, max_angular_radps_);
    teleop_publisher_->publish(limited);
  }

  static double Clamp(const double value, const double low, const double high) {
    return std::min(std::max(value, low), high);
  }

  static float ClampAxis(const double value) {
    return static_cast<float>(Clamp(value, -1.0, 1.0));
  }

  double publish_rate_hz_ = 20.0;
  double max_linear_mps_ = 0.5;
  double max_angular_radps_ = 1.2;
  bool manual_takeover_ = false;
  int command_timeout_ms_ = 200;
  bool have_command_ = false;
  rclcpp::Time last_command_stamp_{0, 0, RCL_ROS_TIME};
  geometry_msgs::msg::Twist virtual_cmd_;
  rclcpp::Publisher<sensor_msgs::msg::Joy>::SharedPtr joy_publisher_;
  rclcpp::Publisher<std_msgs::msg::Bool>::SharedPtr takeover_publisher_;
  rclcpp::Publisher<geometry_msgs::msg::Twist>::SharedPtr teleop_publisher_;
  rclcpp::Subscription<geometry_msgs::msg::Twist>::SharedPtr cmd_subscription_;
  rclcpp::Service<std_srvs::srv::SetBool>::SharedPtr set_takeover_service_;
  rclcpp::TimerBase::SharedPtr timer_;
};

int main(int argc, char** argv) {
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<VirtualRcNode>());
  rclcpp::shutdown();
  return 0;
}
