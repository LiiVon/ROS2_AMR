/**
 * @file geometry_utils.hpp
 * @brief 几何工具函数（工具函数层）。
 * @details 所属包：`robot_utils`；所在层次：工具函数层。
 *
 * 只放与 ROS 无关或只依赖几何消息的纯函数：无状态、无副作用、可单测。
 * 角度约定：弧度制，范围见各函数说明。
 */

#pragma once

#include <cmath>

#include "geometry_msgs/msg/quaternion.hpp"

namespace robot_utils
{

  /**
   * @brief 把角度归一化到 [-π, π]。
   * @param angle 任意弧度的角度（不限圈数，可正可负）
   * @return 归一化后的角度，区间 [-π, π]
   * @details 用循环加减 2π 实现，因此也适用于误差累加后的“大角度”。
   *          边界约定：恰好 π 时返回 π（不翻转符号）。
   */
  inline double NormalizeAngle(double angle)
  {
    while (angle > M_PI)
    {
      angle -= 2.0 * M_PI;
    }
    while (angle < -M_PI)
    {
      angle += 2.0 * M_PI;
    }
    return angle;
  }

  /**
   * @brief 把偏航角转成四元数（只绕固定 z 轴旋转）。
   * @param yaw 偏航角，单位弧度（建议先用 NormalizeAngle 归一化）
   * @return 四元数，x = y = 0，z = sin(yaw/2)，w = cos(yaw/2)
   * @details 平面差速底盘只有 yaw，roll/pitch 恒为 0，因此无需完整欧拉角转换。
   *          返回值的 x、y 保持消息默认值 0，可直接赋给 Pose.orientation。
   */
  inline geometry_msgs::msg::Quaternion QuaternionFromYaw(const double yaw)
  {
    geometry_msgs::msg::Quaternion q;
    q.z = std::sin(yaw * 0.5);
    q.w = std::cos(yaw * 0.5);
    return q;
  }

} // namespace robot_utils
