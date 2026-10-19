// 底盘驱动 ROS 2 节点
// ROS2 上层发出运动指令，这个节点把指令交给底盘，
// 并持续读取底盘状态，再把机器人当前的运动信息发布给整个 ROS 2 系统
#include <algorithm>
#include <chrono>
#include <cmath>
#include <memory>
#include <stdexcept>
#include <string>

#include "robot_hardware/chassis_backend_factory.hpp"
#include "robot_hardware/chassis_kinematics.hpp"
#include "robot_hardware/chassis_system_adapter.hpp"
#include "geometry_msgs/msg/transform_stamped.hpp"
#include "geometry_msgs/msg/twist.hpp"
#include "nav_msgs/msg/odometry.hpp"
#include "rclcpp/rclcpp.hpp"
#include "robot_interfaces/msg/chassis_state.hpp"
#include "robot_interfaces_core/srv/set_chassis_mode.hpp"
#include "sensor_msgs/msg/imu.hpp"
#include "std_srvs/srv/empty.hpp"
#include "tf2_ros/transform_broadcaster.h"

using namespace srd::chrono::literals;

namespace
{
    geometry_msgs::msg::Quaternion QuaternionFromYaw(double yaw)
    {
        geometry_msgs::msg::Quaternion q;
        q.z = std::sin(yaw * 0.5);
        q.w = std::cos(yaw * 0.5);
    }
}