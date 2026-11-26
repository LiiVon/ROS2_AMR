/**
 * @file experiment_runner_node.cpp
 * @brief 实验运行器节点（节点入口层）。
 * @details 所属包：`robot_experiments`；所在层次：节点入口层。
 */

#include <chrono>
#include <cmath>
#include <fstream>
#include <memory>
#include <stdexcept>
#include <sstream>
#include <string>

#include "nav_msgs/msg/odometry.hpp"
#include "rclcpp/rclcpp.hpp"
#include "robot_interfaces_core/msg/navigation_statistics.hpp"
#include "robot_interfaces_core/msg/tracking_error.hpp"

#include "robot_utils/config/topic_names.hpp"

namespace topics = robot_utils::topics;

using namespace std::chrono_literals;

class ExperimentRunnerNode final : public rclcpp::Node
{
public:
  ExperimentRunnerNode() : Node("experiment_runner_node")
  {
    output_csv_ = declare_parameter<std::string>("output_csv", "experiment_results.csv");
    output_json_ = declare_parameter<std::string>("output_json", "");
    controller_name_ = declare_parameter<std::string>("controller_name", "unknown");
    scenario_name_ = declare_parameter<std::string>("scenario_name", "tracking_benchmark");
    path_file_ = declare_parameter<std::string>("path_file", "");
    odom_topic_ = declare_parameter<std::string>("odom_topic", topics::kOdom);
    csv_.open(output_csv_, std::ios::out | std::ios::trunc);
    if (!csv_)
    {
      throw std::runtime_error("failed to open experiment output: " + output_csv_);
    }
    csv_ << "stamp,controller,lateral_error,heading_error,target_speed,actual_speed,nearest_path_index\n";
    tracking_subscription_ = create_subscription<robot_interfaces_core::msg::TrackingError>(
        topics::kTrackingError, 50,
        [this](robot_interfaces_core::msg::TrackingError::SharedPtr msg)
        { OnTrackingError(*msg); });
    // 里程必须来自真实位姿变化，不能用“误差累加”冒充。
    odom_subscription_ = create_subscription<nav_msgs::msg::Odometry>(
        odom_topic_, 20, [this](nav_msgs::msg::Odometry::SharedPtr msg)
        {
          const double x = msg->pose.pose.position.x;
          const double y = msg->pose.pose.position.y;
          if (samples_ == 0) {
            last_odom_x_ = x;
            last_odom_y_ = y;
            return;
          }
          travelled_distance_m_ += std::hypot(x - last_odom_x_, y - last_odom_y_);
          last_odom_x_ = x;
          last_odom_y_ = y; });
    stats_publisher_ = create_publisher<robot_interfaces_core::msg::NavigationStatistics>(
        topics::kExperimentStatistics, 10);
    timer_ = create_wall_timer(1s, [this]()
                               { PublishStatistics(); });
  }

  ~ExperimentRunnerNode() override { WriteJsonReport(); }

private:
  double RunDurationSeconds() const
  {
    return samples_ == 0 || first_stamp_sec_ < 0.0
               ? 0.0
               : std::max(0.0, last_stamp_sec_ - first_stamp_sec_);
  }

  static std::string JsonEscape(const std::string &value)
  {
    std::ostringstream out;
    for (const char c : value)
    {
      switch (c)
      {
      case '\\':
        out << "\\\\";
        break;
      case '"':
        out << "\\\"";
        break;
      case '\n':
        out << "\\n";
        break;
      default:
        out << c;
        break;
      }
    }
    return out.str();
  }

  void OnTrackingError(const robot_interfaces_core::msg::TrackingError &error)
  {
    ++samples_;
    last_stamp_sec_ = rclcpp::Time(error.header.stamp).seconds();
    if (first_stamp_sec_ < 0.0)
    {
      first_stamp_sec_ = last_stamp_sec_;
    }
    last_controller_name_ =
        error.controller_name.empty() ? controller_name_ : error.controller_name;
    last_nearest_path_index_ = error.nearest_path_index;
    sum_abs_lateral_error_ += std::abs(error.lateral_error);
    sum_abs_heading_error_ += std::abs(error.heading_error);
    csv_ << last_stamp_sec_ << ','
         << last_controller_name_ << ','
         << error.lateral_error << ','
         << error.heading_error << ','
         << error.target_speed << ','
         << error.actual_speed << ','
         << error.nearest_path_index << '\n';
  }

  void PublishStatistics()
  {
    robot_interfaces_core::msg::NavigationStatistics stats;
    stats.header.stamp = now();
    // 本节点跟踪的是“一次控制器运行”：有采样才计一次目标，
    // 之前的实现把误差消息条数当目标数、把误差和当里程，指标完全失真。
    const bool has_run = samples_ > 0;
    const double duration =
        has_run && first_stamp_sec_ >= 0.0 ? std::max(0.0, last_stamp_sec_ - first_stamp_sec_) : 0.0;
    stats.total_goals = has_run ? 1 : 0;
    stats.succeeded_goals = has_run ? 1 : 0;
    stats.failed_goals = 0;
    stats.success_rate = has_run ? 1.0 : 0.0;
    stats.total_distance_m = travelled_distance_m_;
    stats.last_navigation_time_s = duration;
    stats.mean_navigation_time_s = stats.succeeded_goals == 0
                                       ? 0.0
                                       : stats.last_navigation_time_s / stats.succeeded_goals;
    stats.recovery_count = 0;
    stats_publisher_->publish(stats);
  }

  void WriteJsonReport()
  {
    if (output_json_.empty())
    {
      return;
    }
    std::ofstream json(output_json_, std::ios::out | std::ios::trunc);
    if (!json)
    {
      RCLCPP_ERROR(get_logger(), "failed to open experiment json output: %s", output_json_.c_str());
      return;
    }
    const double mean_lateral = samples_ == 0 ? 0.0 : sum_abs_lateral_error_ / samples_;
    const double mean_heading = samples_ == 0 ? 0.0 : sum_abs_heading_error_ / samples_;
    json << "{\n"
         << "  \"scenario_name\": \"" << JsonEscape(scenario_name_) << "\",\n"
         << "  \"controller_name\": \"" << JsonEscape(last_controller_name_) << "\",\n"
         << "  \"path_file\": \"" << JsonEscape(path_file_) << "\",\n"
         << "  \"output_csv\": \"" << JsonEscape(output_csv_) << "\",\n"
         << "  \"samples\": " << samples_ << ",\n"
         << "  \"last_stamp_sec\": " << last_stamp_sec_ << ",\n"
         << "  \"mean_abs_lateral_error\": " << mean_lateral << ",\n"
         << "  \"mean_abs_heading_error\": " << mean_heading << ",\n"
         << "  \"sum_abs_lateral_error\": " << sum_abs_lateral_error_ << ",\n"
         << "  \"sum_abs_heading_error\": " << sum_abs_heading_error_ << ",\n"
         << "  \"travelled_distance_m\": " << travelled_distance_m_ << ",\n"
         << "  \"run_duration_s\": " << RunDurationSeconds() << ",\n"
         << "  \"last_nearest_path_index\": " << last_nearest_path_index_ << ",\n"
         << "  \"interface_chain\": \"reference_path -> tracking_controller -> tracking_cmd_vel -> cmd_vel_mux -> cmd_vel -> odom -> tracking_error\"\n"
         << "}\n";
  }

  std::string output_csv_;
  std::string output_json_;
  std::string controller_name_;
  std::string scenario_name_;
  std::string path_file_;
  std::ofstream csv_;
  size_t samples_ = 0;
  double last_stamp_sec_ = 0.0;
  double sum_abs_lateral_error_ = 0.0;
  double sum_abs_heading_error_ = 0.0;
  int last_nearest_path_index_ = -1;
  std::string last_controller_name_ = "unknown";
  std::string odom_topic_;
  double first_stamp_sec_ = -1.0;
  double travelled_distance_m_ = 0.0;
  double last_odom_x_ = 0.0;
  double last_odom_y_ = 0.0;
  rclcpp::Subscription<nav_msgs::msg::Odometry>::SharedPtr odom_subscription_;
  rclcpp::Subscription<robot_interfaces_core::msg::TrackingError>::SharedPtr tracking_subscription_;
  rclcpp::Publisher<robot_interfaces_core::msg::NavigationStatistics>::SharedPtr stats_publisher_;
  rclcpp::TimerBase::SharedPtr timer_;
};

int main(int argc, char **argv)
{
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<ExperimentRunnerNode>());
  rclcpp::shutdown();
  return 0;
}
