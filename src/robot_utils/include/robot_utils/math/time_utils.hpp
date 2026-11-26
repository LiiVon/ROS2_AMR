/**
 * @file time_utils.hpp
 * @brief 时间换算工具（工具函数层）。
 * @details 所属包：`robot_utils`；所在层次：工具函数层。
 *
 * 统一时间差与单位换算，避免各处重复写 duration_cast。
 * 约定：对外一律秒（double），只有定时器参数使用毫秒。
 */

#pragma once

#include <chrono>

#include "rclcpp/rclcpp.hpp"

namespace robot_utils
{

  /**
   * @brief 计算两个 ROS 时间戳之差（秒）。
   * @param newer 较新的时间戳
   * @param older 较旧的时间戳
   * @return 差值，单位秒，可为负（顺序反了时）
   * @note rclcpp::Time 不允许混用不同 clock_type（如 ROS_TIME 与 STEADY_TIME），
   *       传入前需保证两者同源。
   */
  inline double SecondsBetween(const rclcpp::Time &newer, const rclcpp::Time &older)
  {
    return (newer - older).seconds();
  }

  /**
   * @brief 秒转毫秒（用于 wall timer 周期、超时参数等整型入参）。
   * @param seconds 秒数，可以是小数（如 0.02 表示 20 ms）
   * @return 毫秒时长
   * @note duration_cast 是**截断**而非四舍五入：0.0005 s 会得到 0 ms。
   *       定时器周期必须保证换算后 > 0，否则 create_wall_timer 会抛错。
   */
  inline std::chrono::milliseconds SecondsToMilliseconds(const double seconds)
  {
    return std::chrono::duration_cast<std::chrono::milliseconds>(
        std::chrono::duration<double>(seconds));
  }

} // namespace robot_utils
