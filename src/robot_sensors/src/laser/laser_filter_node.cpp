/**
 * @file laser_filter_node.cpp
 * @brief 激光过滤节点（激光数据处理层）。
 * @details 所属包：`robot_sensors`；所在层次：激光数据处理层。
 */

#include <algorithm>
#include <cmath>
#include <limits>
#include <memory>
#include <string>

#include "diagnostic_msgs/msg/diagnostic_status.hpp"
#include "diagnostic_updater/diagnostic_updater.hpp"
#include "rclcpp/rclcpp.hpp"
#include "sensor_msgs/msg/laser_scan.hpp"

#include "robot_utils/config/topic_names.hpp"

namespace topics = robot_utils::topics;

class LaserFilterNode final : public rclcpp::Node
{
public:
  LaserFilterNode() : Node("laser_filter_node"), diagnostics_(this)
  {
    input_topic_ = declare_parameter<std::string>("input_topic", topics::kSimScan);
    output_topic_ = declare_parameter<std::string>("output_topic", topics::kScan);
    frame_id_ = declare_parameter<std::string>("frame_id", "lidar_link");
    range_min_ = declare_parameter<double>("range_min", 0.08);
    range_max_ = declare_parameter<double>("range_max", 12.0);
    angle_min_ = declare_parameter<double>("angle_min", -3.141592653589793);
    angle_max_ = declare_parameter<double>("angle_max", 3.141592653589793);
    clockwise_ = declare_parameter<bool>("clockwise", false);
    invalid_range_as_inf_ = declare_parameter<bool>("invalid_range_as_inf", true);
    enable_builtin_filter_ = declare_parameter<bool>("enable_builtin_filter", true);
    subscription_ = create_subscription<sensor_msgs::msg::LaserScan>(
        input_topic_, 10, [this](sensor_msgs::msg::LaserScan::SharedPtr msg)
        { Filter(*msg); });
    publisher_ = create_publisher<sensor_msgs::msg::LaserScan>(output_topic_, 10);
    diagnostics_.setHardwareID(frame_id_);
    diagnostics_.add("laser_filter", [this](diagnostic_updater::DiagnosticStatusWrapper &stat)
                     {
      stat.summary(diagnostic_msgs::msg::DiagnosticStatus::OK, "scan normalized");
      stat.add("input_topic", input_topic_);
      stat.add("output_topic", output_topic_);
      stat.add("invalid_ranges", last_invalid_count_);
      stat.add("builtin_filter_enabled", enable_builtin_filter_ ? "true" : "false"); });
  }

private:
  void Filter(sensor_msgs::msg::LaserScan scan)
  {
    scan.header.frame_id = frame_id_;
    scan.range_min = range_min_;
    scan.range_max = range_max_;
    if (clockwise_)
    {
      // 反转数据的同时必须重写角度元数据，否则每个点的角度仍按下标递增，
      // 整幅扫描会被镜像（障碍物左右颠倒）。
      //
      // 推导：反转后下标 j 对应原下标 N-1-j，其物理角度为
      //   -(old_angle_min + (N-1-j) * old_increment)
      //   = -old_angle_max + j * old_increment
      // 因此新序列是「从 -old_angle_max 开始、步长仍为 old_increment」，
      // 而不是把步长取反（取反会让 angle_max 落到 [-π, π] 之外）。
      const double old_angle_min = scan.angle_min;
      const double old_increment = scan.angle_increment;
      const std::size_t count = scan.ranges.size();
      const double old_angle_max =
          count == 0U ? old_angle_min
                      : old_angle_min + old_increment * static_cast<double>(count - 1U);
      std::reverse(scan.ranges.begin(), scan.ranges.end());
      if (scan.intensities.size() == count)
      {
        std::reverse(scan.intensities.begin(), scan.intensities.end());
      }
      scan.angle_min = -old_angle_max;
      scan.angle_increment = old_increment;
      scan.angle_max = -old_angle_min;
    }
    int invalid_count = 0;
    for (std::size_t i = 0; i < scan.ranges.size(); ++i)
    {
      auto &range = scan.ranges[i];
      const double angle = scan.angle_min + static_cast<double>(i) * scan.angle_increment;
      const bool angle_rejected = angle < angle_min_ || angle > angle_max_;
      const bool range_rejected = !std::isfinite(range) || range < range_min_ || range > range_max_;
      if (enable_builtin_filter_ && (range_rejected || angle_rejected))
      {
        range = invalid_range_as_inf_ ? std::numeric_limits<float>::infinity() : 0.0F;
        if (i < scan.intensities.size())
        {
          scan.intensities[i] = 0.0F;
        }
        ++invalid_count;
      }
    }
    publisher_->publish(scan);
    last_invalid_count_ = invalid_count;
    diagnostics_.force_update();
  }

  std::string input_topic_;
  std::string output_topic_;
  std::string frame_id_;
  double range_min_;
  double range_max_;
  double angle_min_;
  double angle_max_;
  bool clockwise_;
  bool invalid_range_as_inf_;
  bool enable_builtin_filter_;
  int last_invalid_count_ = 0;
  rclcpp::Subscription<sensor_msgs::msg::LaserScan>::SharedPtr subscription_;
  rclcpp::Publisher<sensor_msgs::msg::LaserScan>::SharedPtr publisher_;
  diagnostic_updater::Updater diagnostics_;
};

int main(int argc, char **argv)
{
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<LaserFilterNode>());
  rclcpp::shutdown();
  return 0;
}
