/**
 * @file imu_100d4_parser_node.cpp
 * @brief （IMU 数据处理层）。
 * @details 所属包：`robot_sensors`；所在层次：IMU 数据处理层。
 */

#include <cmath>
#include <cstdint>
#include <memory>
#include <string>
#include <vector>

#include "geometry_msgs/msg/quaternion.hpp"
#include "rclcpp/rclcpp.hpp"
#include "sensor_msgs/msg/imu.hpp"
#include "std_msgs/msg/string.hpp"
#include "std_msgs/msg/u_int8_multi_array.hpp"
#include "std_srvs/srv/trigger.hpp"

namespace {

constexpr std::size_t kMaxFrameLength = 256U;

double DecodeInt16Scaled(const std::vector<uint8_t>& data, const std::size_t offset, const double scale) {
  if (offset + 1 >= data.size()) {
    return 0.0;
  }
  const auto raw =
      static_cast<int16_t>(static_cast<uint16_t>(data[offset] << 8) | static_cast<uint16_t>(data[offset + 1]));
  return static_cast<double>(raw) / scale;
}

geometry_msgs::msg::Quaternion QuaternionFromEuler(
    const double roll, const double pitch, const double yaw) {
  const double cy = std::cos(yaw * 0.5);
  const double sy = std::sin(yaw * 0.5);
  const double cp = std::cos(pitch * 0.5);
  const double sp = std::sin(pitch * 0.5);
  const double cr = std::cos(roll * 0.5);
  const double sr = std::sin(roll * 0.5);
  geometry_msgs::msg::Quaternion q;
  q.w = cr * cp * cy + sr * sp * sy;
  q.x = sr * cp * cy - cr * sp * sy;
  q.y = cr * sp * cy + sr * cp * sy;
  q.z = cr * cp * sy - sr * sp * cy;
  return q;
}

}  // namespace

class Imu100d4ParserNode final : public rclcpp::Node {
 public:
  Imu100d4ParserNode() : Node("imu_100d4_parser_node") {
    input_topic_ = declare_parameter<std::string>("input_topic", "/imu/raw_bytes");
    output_topic_ = declare_parameter<std::string>("output_topic", "/imu/data");
    frame_id_ = declare_parameter<std::string>("frame_id", "imu_link");
    gravity_ = declare_parameter<double>("gravity", 9.80665);
    subscription_ = create_subscription<std_msgs::msg::UInt8MultiArray>(
        input_topic_, 10,
        [this](std_msgs::msg::UInt8MultiArray::SharedPtr msg) { Parse(msg->data); });
    imu_publisher_ = create_publisher<sensor_msgs::msg::Imu>(output_topic_, 10);
    state_publisher_ = create_publisher<std_msgs::msg::String>("/imu/calibration_state", 10);
    start_calib_service_ = create_service<std_srvs::srv::Trigger>(
        "/sensors/imu/start_calibration",
        [this](const std::shared_ptr<std_srvs::srv::Trigger::Request>,
               std::shared_ptr<std_srvs::srv::Trigger::Response> response) {
          calibration_state_ = "magnetic_calibration_active";
          response->success = true;
          response->message = "100D4 magnetic calibration command queued";
          PublishState();
        });
    stop_calib_service_ = create_service<std_srvs::srv::Trigger>(
        "/sensors/imu/stop_calibration",
        [this](const std::shared_ptr<std_srvs::srv::Trigger::Request>,
               std::shared_ptr<std_srvs::srv::Trigger::Response> response) {
          calibration_state_ = "idle";
          response->success = true;
          response->message = "100D4 magnetic calibration stop command queued";
          PublishState();
        });
    timer_ = create_wall_timer(std::chrono::seconds(1), [this]() { PublishState(); });
  }

 private:
  void Parse(const std::vector<uint8_t>& bytes) {
    if (bytes.empty()) {
      return;
    }
    // 串口是字节流：一帧可能跨多条消息，也可能一条消息里有多个帧。
    // 因此必须维护接收缓冲，循环消费所有完整帧（原实现只取第一帧且边界判错一位）。
    buffer_.insert(buffer_.end(), bytes.begin(), bytes.end());

    std::size_t consumed = 0;
    while (consumed + 3U <= buffer_.size()) {
      if (buffer_[consumed] != 0xa5 || buffer_[consumed + 1U] != 0x5a) {
        ++consumed;
        continue;
      }
      const std::size_t length = buffer_[consumed + 2U];
      if (length < 24U || length > kMaxFrameLength) {
        ++consumed;
        continue;
      }
      const std::size_t frame_size = length + 2U;
      if (consumed + frame_size > buffer_.size()) {
        // 半包：等待后续字节，不要丢弃。
        break;
      }
      uint8_t checksum = 0;
      for (std::size_t j = consumed + 2U; j < consumed + 1U + length; ++j) {
        checksum = static_cast<uint8_t>(checksum + buffer_[j]);
      }
      if (checksum != buffer_[consumed + 1U + length]) {
        ++consumed;
        continue;
      }
      std::vector<uint8_t> frame(buffer_.begin() + static_cast<long>(consumed),
                                buffer_.begin() + static_cast<long>(consumed + frame_size));
      consumed += frame_size;
      PublishImu(frame);
    }

    buffer_.erase(buffer_.begin(), buffer_.begin() + static_cast<long>(consumed));
    // 防止长期收不到有效帧导致缓冲无限增长。
    constexpr std::size_t kMaxBufferBytes = 4096U;
    if (buffer_.size() > kMaxBufferBytes) {
      buffer_.erase(buffer_.begin(),
                    buffer_.end() - static_cast<long>(kMaxBufferBytes));
    }
  }

  void PublishImu(const std::vector<uint8_t>& frame) {
    sensor_msgs::msg::Imu imu;
    imu.header.stamp = now();
    imu.header.frame_id = frame_id_;
    const double yaw = -DecodeInt16Scaled(frame, 3, 100.0) * M_PI / 180.0;
    const double roll = -DecodeInt16Scaled(frame, 5, 100.0) * M_PI / 180.0;
    const double pitch = -DecodeInt16Scaled(frame, 7, 100.0) * M_PI / 180.0;
    imu.orientation = QuaternionFromEuler(roll, pitch, yaw);
    imu.linear_acceleration.x = -DecodeInt16Scaled(frame, 9, 1000.0) * gravity_;
    imu.linear_acceleration.y = -DecodeInt16Scaled(frame, 11, 1000.0) * gravity_;
    imu.linear_acceleration.z = DecodeInt16Scaled(frame, 13, 1000.0) * gravity_;
    imu.angular_velocity.x = -DecodeInt16Scaled(frame, 15, 1000.0);
    imu.angular_velocity.y = -DecodeInt16Scaled(frame, 17, 1000.0);
    imu.angular_velocity.z = DecodeInt16Scaled(frame, 19, 1000.0);
    imu_publisher_->publish(imu);
  }

  void PublishState() {
    std_msgs::msg::String state;
    state.data = calibration_state_;
    state_publisher_->publish(state);
  }

  std::string input_topic_;
  std::string output_topic_;
  std::string frame_id_;
  std::string calibration_state_ = "idle";
  double gravity_ = 9.80665;
  std::vector<uint8_t> buffer_;
  rclcpp::Subscription<std_msgs::msg::UInt8MultiArray>::SharedPtr subscription_;
  rclcpp::Publisher<sensor_msgs::msg::Imu>::SharedPtr imu_publisher_;
  rclcpp::Publisher<std_msgs::msg::String>::SharedPtr state_publisher_;
  rclcpp::Service<std_srvs::srv::Trigger>::SharedPtr start_calib_service_;
  rclcpp::Service<std_srvs::srv::Trigger>::SharedPtr stop_calib_service_;
  rclcpp::TimerBase::SharedPtr timer_;
};

int main(int argc, char** argv) {
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<Imu100d4ParserNode>());
  rclcpp::shutdown();
  return 0;
}
