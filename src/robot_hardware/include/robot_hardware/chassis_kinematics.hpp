// chassis_kinematics
// 运动学计算模块
#pragma once

#include <array>
#include <string>

#include "robot_hardware/chassis_packet.hpp"

namespace robot_hardware
{
    // 机器人在二维平面上的位姿（位置和朝向）
    struct PlanarPose
    {
        double x_m = 0.0;     // 机器人在世界坐标系下的x坐标
        double y_m = 0.0;     // 机器人在世界坐标系下的y坐标
        double yaw_rad = 0.0; // 机器人在世界坐标系下的航向角（弧度）
    };

    // 机器人在平面上的速度（线速度和角速度）
    struct WheelSpeeds
    {
        // 四个轮子的转速（单位：转/分钟）
        // [0,1,2,3] 分别对应 [左前轮, 右前轮, 左后轮, 右后轮]
        // revolutions per minute 的缩写，表示每分钟转数
        std::array<double, 4> rpm{0.0, 0.0, 0.0, 0.0};
    };

    // 统一运动学模型名称
    std::string NormalizeKinematicsModel(const std::string &model_name);

    // 根据速度估算新位姿    当前pose，底盘运动指令，经过的时间
    PlanarPose IntegratePlanarMotion(const PlanarPose &pose, const ChassisCommand &command, double dt_seconds);

    // 底盘速度换算成车轮转速  底盘运动指令，底盘运动学模型，车轮直径，轴距(前后车轴的距离)，轮距(左右车轮的距离)
    WheelSpeeds EstimateWheelSpeeds(const ChassisCommand &command, const std::string &model,
                                    double wheel_diameter_m, double wheel_base_m, double track_width_m);

    // 车轮转速反推底盘运动
    ChassisCommand EstimateCommandFromWheelSpeeds(const WheelSpeeds &speeds, const std::string &model,
                                                  double wheel_diameter_m, double wheel_base_m, double track_width_m);
}