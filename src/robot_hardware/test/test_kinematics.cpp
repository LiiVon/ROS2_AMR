
#include "robot_hardware/chassis_kinematics.hpp"

#include <cmath>
#include <iostream>
#include <stdexcept>
#include <string>

namespace
{
    using namespace robot_hardware;

    // ============================================================
    // 测试辅助函数
    // ============================================================

    // 判断两个浮点数是否足够接近
    void ExpectNear(
        double actual,
        double expected,
        const std::string &message,
        double tolerance = 1e-6)
    {
        if (std::abs(actual - expected) > tolerance)
        {
            throw std::runtime_error(
                message +
                " | actual = " + std::to_string(actual) +
                ", expected = " + std::to_string(expected));
        }
    }

    // 计算车轮线速度对应的 RPM，仅用于测试预期值
    double ToRpm(double linear_speed, double wheel_diameter)
    {
        return linear_speed / (wheel_diameter * 3.14159265358979323846) * 60.0;
    }

    // ============================================================
    // 一、差速底盘测试
    // ============================================================

    void TestDiffDrive()
    {
        std::cout << "\n========== Diff Drive Tests ==========\n";

        const double wheel_diameter = 0.2;
        const double wheel_base = 0.4;
        const double track_width = 0.3;

        // --------------------------------------------------------
        // 1. 测试直线前进
        // --------------------------------------------------------

        ChassisCommand forward;
        forward.linear_x_mps = 1.0;
        forward.linear_y_mps = 0.0;
        forward.angular_z_radps = 0.0;

        WheelSpeeds forward_speeds = EstimateWheelSpeeds(
            forward,
            "diff_drive",
            wheel_diameter,
            wheel_base,
            track_width);

        // 左右轮速度应该相同
        ExpectNear(
            forward_speeds.rpm[0],
            ToRpm(1.0, wheel_diameter),
            "DiffDrive forward left wheel");

        ExpectNear(
            forward_speeds.rpm[1],
            ToRpm(1.0, wheel_diameter),
            "DiffDrive forward right wheel");

        std::cout << "[PASS] DiffDrive forward movement\n";

        // --------------------------------------------------------
        // 2. 测试转弯运动
        // --------------------------------------------------------

        ChassisCommand turning;
        turning.linear_x_mps = 1.0;
        turning.linear_y_mps = 0.0;
        turning.angular_z_radps = 0.5;

        WheelSpeeds turning_speeds = EstimateWheelSpeeds(
            turning,
            "diff_drive",
            wheel_diameter,
            wheel_base,
            track_width);

        const double half_track = track_width * 0.5;

        const double expected_left =
            1.0 - 0.5 * half_track;

        const double expected_right =
            1.0 + 0.5 * half_track;

        ExpectNear(
            turning_speeds.rpm[0],
            ToRpm(expected_left, wheel_diameter),
            "DiffDrive turning left wheel");

        ExpectNear(
            turning_speeds.rpm[1],
            ToRpm(expected_right, wheel_diameter),
            "DiffDrive turning right wheel");

        std::cout << "[PASS] DiffDrive turning movement\n";

        // --------------------------------------------------------
        // 3. 测试正运动学：根据轮速反推底盘运动
        // --------------------------------------------------------

        ChassisCommand estimated = EstimateCommandFromWheelSpeeds(
            turning_speeds,
            "diff_drive",
            wheel_diameter,
            wheel_base,
            track_width);

        ExpectNear(
            estimated.linear_x_mps,
            turning.linear_x_mps,
            "DiffDrive estimated linear velocity");

        ExpectNear(
            estimated.angular_z_radps,
            turning.angular_z_radps,
            "DiffDrive estimated angular velocity");

        std::cout << "[PASS] DiffDrive forward and inverse kinematics\n";
    }

    // ============================================================
    // 二、麦克纳姆轮底盘测试
    // ============================================================

    void TestMecanum()
    {
        std::cout << "\n========== Mecanum Tests ==========\n";

        const double wheel_diameter = 0.2;
        const double wheel_base = 0.4;
        const double track_width = 0.3;

        // --------------------------------------------------------
        // 1. 测试组合运动
        // --------------------------------------------------------

        ChassisCommand command;
        command.linear_x_mps = 1.0;
        command.linear_y_mps = 0.2;
        command.angular_z_radps = 0.5;

        WheelSpeeds speeds = EstimateWheelSpeeds(
            command,
            "mecanum",
            wheel_diameter,
            wheel_base,
            track_width);

        const double rotation_radius =
            (wheel_base + track_width) * 0.5;

        // 根据运动学公式计算四个轮子的预期线速度
        const double expected_fl =
            1.0 - 0.2 - 0.5 * rotation_radius;

        const double expected_fr =
            1.0 + 0.2 + 0.5 * rotation_radius;

        const double expected_rl =
            1.0 + 0.2 - 0.5 * rotation_radius;

        const double expected_rr =
            1.0 - 0.2 + 0.5 * rotation_radius;

        // 验证四个车轮的 RPM
        ExpectNear(
            speeds.rpm[0],
            ToRpm(expected_fl, wheel_diameter),
            "Mecanum front left wheel");

        ExpectNear(
            speeds.rpm[1],
            ToRpm(expected_fr, wheel_diameter),
            "Mecanum front right wheel");

        ExpectNear(
            speeds.rpm[2],
            ToRpm(expected_rl, wheel_diameter),
            "Mecanum rear left wheel");

        ExpectNear(
            speeds.rpm[3],
            ToRpm(expected_rr, wheel_diameter),
            "Mecanum rear right wheel");

        std::cout << "[PASS] Mecanum wheel speed calculation\n";

        // --------------------------------------------------------
        // 2. 测试正运动学：根据四个轮子的速度反推底盘运动
        // --------------------------------------------------------

        ChassisCommand estimated = EstimateCommandFromWheelSpeeds(
            speeds,
            "mecanum",
            wheel_diameter,
            wheel_base,
            track_width);

        ExpectNear(
            estimated.linear_x_mps,
            command.linear_x_mps,
            "Mecanum estimated linear_x");

        ExpectNear(
            estimated.linear_y_mps,
            command.linear_y_mps,
            "Mecanum estimated linear_y");

        ExpectNear(
            estimated.angular_z_radps,
            command.angular_z_radps,
            "Mecanum estimated angular_z");

        std::cout << "[PASS] Mecanum forward and inverse kinematics\n";
    }
}

// ================================================================
// 测试程序入口
// ================================================================

int main()
{
    try
    {
        TestDiffDrive();
        TestMecanum();

        std::cout << "\n====================================\n";
        std::cout << "All kinematics tests passed!\n";
        std::cout << "====================================\n";

        return 0;
    }
    catch (const std::exception &e)
    {
        std::cerr << "\n[FAILED] " << e.what() << '\n';
        return 1;
    }
}
