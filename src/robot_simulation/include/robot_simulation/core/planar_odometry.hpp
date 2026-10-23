/**
 * @file planar_odometry.hpp
 * @brief 平面里程计（纯算法层）。
 * @details 所属包：`robot_simulation`；所在层次：纯算法层。
 * @note 教学说明见 `docs/教学/` 对应章节。
 */

#pragma once

#include <string>

#include "nav_msgs/msg/odometry.hpp"

namespace robot_simulation {

/**
 * @brief ProjectOdometryToPlanarFootprint。
 * @param input 参数
 * @param odom_frame 字符串
 * @param base_frame 字符串
 * @return 计算结果。
 */
nav_msgs::msg::Odometry ProjectOdometryToPlanarFootprint(
    const nav_msgs::msg::Odometry& input, const std::string& odom_frame,
    const std::string& base_frame);

}  // namespace robot_simulation
