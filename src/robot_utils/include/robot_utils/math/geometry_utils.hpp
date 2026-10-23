/**
 * @file geometry_utils.hpp
 * @brief 几何工具（工具函数层）。
 * @details 所属包：`robot_utils`；所在层次：工具函数层。
 * @note 教学说明见 `docs/教学/` 对应章节。
 */

#pragma once

#include <cmath>

#include "geometry_msgs/msg/quaternion.hpp"

namespace robot_utils {

/**
 * @brief 归一化角度。
 * @param angle 数值参数
 * @return 计算结果（数值）。
 */
inline double NormalizeAngle(double angle) {
  while (angle > M_PI) {
    angle -= 2.0 * M_PI;
  }
  while (angle < -M_PI) {
    angle += 2.0 * M_PI;
  }
  return angle;
}

/**
 * @brief QuaternionFromYaw。
 * @param yaw 数值参数
 * @return 计算结果。
 */
inline geometry_msgs::msg::Quaternion QuaternionFromYaw(const double yaw) {
  geometry_msgs::msg::Quaternion q;
  q.z = std::sin(yaw * 0.5);
  q.w = std::cos(yaw * 0.5);
  return q;
}

}  // namespace robot_utils
