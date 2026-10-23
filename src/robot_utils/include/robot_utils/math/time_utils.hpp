/**
 * @file time_utils.hpp
 * @brief 时间工具（工具函数层）。
 * @details 所属包：`robot_utils`；所在层次：工具函数层。
 * @note 教学说明见 `docs/教学/` 对应章节。
 */

#pragma once

#include <chrono>

#include "rclcpp/rclcpp.hpp"

namespace robot_utils {

/**
 * @brief SecondsBetween。
 * @param newer 参数
 * @param older 参数
 * @return 计算结果（数值）。
 */
inline double SecondsBetween(const rclcpp::Time& newer, const rclcpp::Time& older) {
  return (newer - older).seconds();
}

/**
 * @brief SecondsToMilliseconds。
 * @param seconds 数值参数
 * @return 计算结果。
 */
inline std::chrono::milliseconds SecondsToMilliseconds(const double seconds) {
  return std::chrono::duration_cast<std::chrono::milliseconds>(
      std::chrono::duration<double>(seconds));
}

}  // namespace robot_utils
