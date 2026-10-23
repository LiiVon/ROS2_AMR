/**
 * @file kinematics.hpp
 * @brief 运动学（运动学层）。
 * @details chassis_kinematics.hpp 底盘运动学计算模块：定义位姿、轮速及运动学函数接口。
 * @details 所属包：`robot_hardware`；所在层次：运动学层。
 * @note 教学说明见 `docs/教学/` 对应章节。
 */

#pragma once

#include <array>
#include <string>

#include "robot_hardware/protocol/packet.hpp"

namespace robot_hardware::kinematics
{
    using protocol::Command;


    // 机器人在世界坐标系（例如 odom）中的二维位姿。
    // x、y 的单位是米，yaw 的单位是弧度。
    /**
     * @brief Pose2D 数据结构定义
     */
    struct Pose2D
    {
        double x_m = 0.0;
        double y_m = 0.0;
        double yaw_rad = 0.0;
    };

    // 四轮底盘的轮子转速，单位为 RPM（转/分钟）。
    // 索引顺序固定为：
    // 0 = 左前轮，1 = 右前轮，
    // 2 = 左后轮，3 = 右后轮。
    /**
     * @brief WheelSpeeds 数据结构定义
     */
    struct WheelSpeeds
    {
        std::array<double, 4> rpm{0.0, 0.0, 0.0, 0.0};
    };

    // 将已支持的运动学模型名称转换为标准名称。
    // 当前支持 diff_drive 和 mecanum。
    std::string NormalizeModel(const std::string &model_name);

    // 根据机器人当前位姿、机体坐标系速度和时间间隔，
    // 估算新的世界坐标系位姿。
    Pose2D IntegratePose(
        const Pose2D &pose,
        const Command &command,
        double dt_seconds);

    // 根据底盘速度计算四个车轮的转速。
    // wheel_base_m：轴距；track_width_m：轮距。
    WheelSpeeds EstimateWheelSpeeds(
        const Command &command,
        const std::string &model,
        double wheel_diameter_m,
        double wheel_base_m,
        double track_width_m);

    // 根据四个车轮的转速，估算底盘线速度与角速度。
    Command EstimateChassisCommand(
        const WheelSpeeds &speeds,
        const std::string &model,
        double wheel_diameter_m,
        double wheel_base_m,
        double track_width_m);

} // namespace robot_hardware