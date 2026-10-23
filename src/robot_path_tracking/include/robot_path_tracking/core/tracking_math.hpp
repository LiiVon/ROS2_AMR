/**
 * @file tracking_math.hpp
 * @brief 跟踪数学（纯算法层）。
 * @details 所属包：`robot_path_tracking`；所在层次：纯算法层。
 * @note 教学说明见 `docs/教学/` 对应章节。
 */

#pragma once

#include <algorithm>
#include <cstddef>
#include <cmath>

namespace robot_path_tracking {

/**
 * @brief 归一化角度。
 * @param angle 数值参数
 * @return 计算结果（数值）。
 */
inline double NormalizeAngle(double angle) {
  while (angle > M_PI) angle -= 2.0 * M_PI;
  while (angle < -M_PI) angle += 2.0 * M_PI;
  return angle;
}

/**
 * @brief SignedLateralError。
 * @param path_x 数值参数
 * @param path_y 数值参数
 * @param path_yaw 数值参数
 * @param robot_x 数值参数
 * @param robot_y 数值参数
 * @return 计算结果（数值）。
 */
inline double SignedLateralError(const double path_x, const double path_y,
                                 const double path_yaw, const double robot_x,
                                 const double robot_y) {
  const double dx = path_x - robot_x;
  const double dy = path_y - robot_y;
  return std::sin(path_yaw) * dx - std::cos(path_yaw) * dy;
}

/**
 * @brief ForwardProjection。
 * @param robot_yaw 数值参数
 * @param dx 数值参数
 * @param dy 数值参数
 * @return 计算结果（数值）。
 */
inline double ForwardProjection(const double robot_yaw, const double dx, const double dy) {
  return std::cos(robot_yaw) * dx + std::sin(robot_yaw) * dy;
}

/**
 * @brief 判断是否 IsFinalWaypointReached。
 * @param nearest_index 索引
 * @param path_size 数量/尺寸
 * @param distance_to_goal 数值参数
 * @param goal_tolerance 数值参数
 * @return 满足条件返回 true，否则返回 false。
 */
inline bool IsFinalWaypointReached(
    const int nearest_index, const std::size_t path_size, const double distance_to_goal,
    const double goal_tolerance) {
  return path_size > 0 && nearest_index == static_cast<int>(path_size - 1) &&
         distance_to_goal <= std::max(0.0, goal_tolerance);
}

}  // namespace robot_path_tracking
