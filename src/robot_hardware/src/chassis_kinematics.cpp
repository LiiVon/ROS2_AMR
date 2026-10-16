#include "robot_hardware/chassis_kinematics.hpp"

#include <algorithm>
#include <cmath>
#include <stdexcept>

namespace robot_hardware
{
    namespace
    {
        const double kPi = 3.14159265358979323846;

        // 线速度转换成 RPM
        // 周长 = π * 直径  转1圈，就是1周长的距离
        // 线速度v，每秒转动的圈数rps = v / 周长 = v / (π * 直径)
        // 转每分钟的圈数rpm = rps * 60
        double WheelRpmFromLinear(const double speed_mps, const double wheel_diameter_m)
        {
            if (wheel_diameter_m <= 0.0)
            {
                throw std::invalid_argument("Wheel diameter must be positive.");
            }
            return speed_mps / (kPi * wheel_diameter_m) * 60.0;
        }

        // RPM 转换成线速度
        double WheelLinearFromRpm(const double rpm, const double wheel_diameter_m)
        {
            if (wheel_diameter_m <= 0.0)
            {
                throw std::invalid_argument("Wheel diameter must be positive.");
            }
            return rpm / 60.0 * kPi * wheel_diameter_m;
        }
    }

    std::string NormalizeKinematicsModel(const std::string &model_name)
    {
        // 麦科姆轮、全向轮 都归为 mecanum，即y方向速度由轮子转速决定
        if (model_name == "mecanum" || model_name == "Mecanum" || model_name == "omni" || model_name == "Omni")
        {
            return "mecanum";
        }

        // 目前是 差速驱动、Ackermann 转向、四轮转向 都归为 diff_drive，即y方向速度为0，角速度由左右轮差速决定
        if (model_name == "diff_drive" || model_name == "differential_drive" ||
            model_name == "ackermann" || model_name == "acker" ||
            model_name == "ackermann_drive" || model_name == "four_ws4wd" || model_name == "4ws4wd" ||
            model_name == "four_wheel_steer")
        {
            return "diff_drive";
        }

        else
        {
            throw std::invalid_argument("Unsupported kinematics model: " + model_name);
        }
    }

    //   当前pose，底盘运动指令，经过的时间
    PlanarPose IntegratePlanarMotion(const PlanarPose &pose, const ChassisCommand &command, double dt_seconds)
    {
        const double dt = std::max(dt_seconds, 0.0); // 确保时间间隔非负
        PlanarPose tmp_pose = pose;

        // 单纯的 先求 角度的cos sin ，然后再求速度的分解
        const double cos_yaw = std::cos(tmp_pose.yaw_rad);
        const double sin_yaw = std::sin(tmp_pose.yaw_rad);

        // 速度的分解 ，得到Vx Vy ，然后再乘时间   Sx = Vx * dt  Sy = Vy * dt
        tmp_pose.x_m += (command.linear_x_mps * cos_yaw - command.linear_y_mps * sin_yaw) * dt;
        tmp_pose.y_m += (command.linear_x_mps * sin_yaw + command.linear_y_mps * cos_yaw) * dt;

        // 角速度*时间
        tmp_pose.yaw_rad += command.angular_z_radps * dt;
        // 归一化到 [-π, π]
        tmp_pose.yaw_rad = std::atan2(std::sin(tmp_pose.yaw_rad), std::cos(tmp_pose.yaw_rad));

        return tmp_pose;
    }

    // 底盘速度换算成车轮转速  底盘运动指令，底盘运动学模型，车轮直径，轴距(前后车轴的距离)，轮距(左右车轮的距离)
    WheelSpeeds EstimateWheelSpeeds(const ChassisCommand &command, const std::string &model,
                                    double wheel_diameter_m, double wheel_base_m, double track_width_m)
    {
        WheelSpeeds speeds;
        const std::string normalized_model = NormalizeKinematicsModel(model);

        // 计算半轴距
        const double half_track = track_width_m * 0.5;

        // 旋转半径，适用于差速驱动和全向轮
        const double rotation_radius = (wheel_base_m + track_width_m) * 0.5;

        if (normalized_model == "mecanum")
        {
            // 左前轮
            const double fl =
                command.linear_x_mps - command.linear_y_mps - command.angular_z_radps * rotation_radius;

            // 右前轮
            const double fr =
                command.linear_x_mps + command.linear_y_mps + command.angular_z_radps * rotation_radius;

            // 左后轮
            const double rl =
                command.linear_x_mps + command.linear_y_mps - command.angular_z_radps * rotation_radius;

            // 右后轮
            const double rr =
                command.linear_x_mps - command.linear_y_mps + command.angular_z_radps * rotation_radius;

            speeds.rpm = {
                WheelRpmFromLinear(fl, wheel_diameter_m),
                WheelRpmFromLinear(fr, wheel_diameter_m),
                WheelRpmFromLinear(rl, wheel_diameter_m),
                WheelRpmFromLinear(rr, wheel_diameter_m)};
            return speeds;
        }

        if (normalized_model == "diff_drive")
        {
            // 左轮
            const double left =
                command.linear_x_mps - command.angular_z_radps * half_track;

            // 右轮
            const double right =
                command.linear_x_mps + command.angular_z_radps * half_track;

            speeds.rpm = {
                WheelRpmFromLinear(left, wheel_diameter_m),
                WheelRpmFromLinear(right, wheel_diameter_m),
                WheelRpmFromLinear(left, wheel_diameter_m),
                WheelRpmFromLinear(right, wheel_diameter_m)};
            return speeds;
        }
        return speeds;
    }

    // 车轮转速反推底盘运动
    ChassisCommand EstimateCommandFromWheelSpeeds(const WheelSpeeds &speeds, const std::string &model,
                                                  double wheel_diameter_m, double wheel_base_m, double track_width_m)
    {
        ChassisCommand command;
        const std::string normalized = NormalizeKinematicsModel(model);

        const double fl =
            WheelLinearFromRpm(speeds.rpm[0], wheel_diameter_m);

        const double fr =
            WheelLinearFromRpm(speeds.rpm[1], wheel_diameter_m);

        const double rl =
            WheelLinearFromRpm(speeds.rpm[2], wheel_diameter_m);

        const double rr =
            WheelLinearFromRpm(speeds.rpm[3], wheel_diameter_m);

        if (normalized == "mecanum")
        {
            const double rotation_radius =
                (wheel_base_m + track_width_m) * 0.5;

            command.linear_x_mps = (fl + fr + rl + rr) * 0.25;

            command.linear_y_mps = (-fl + fr + rl - rr) * 0.25;

            if (rotation_radius > 0.0)
            {
                command.angular_z_radps =
                    (-fl + fr - rl + rr) / (4.0 * rotation_radius);
            }

            return command;
        }

        // diff_drive
        const double left = (fl + rl) * 0.5;
        const double right = (fr + rr) * 0.5;
        command.linear_x_mps = (left + right) * 0.5;
        if (track_width_m > 0.0)
        {
            command.angular_z_radps = (right - left) / track_width_m;
        }
        return command;
    }
}