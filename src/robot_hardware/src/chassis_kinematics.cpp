// chassis_kinematics.cpp
// 底盘运动学计算的具体实现。

#include "robot_hardware/chassis_kinematics.hpp"

#include <cmath>
#include <stdexcept>
#include <string>

namespace robot_hardware
{

    namespace
    {

        constexpr double kPi = 3.14159265358979323846;
        constexpr double kVelocityEpsilon = 1e-9;

        // 检查数值是否有限，拒绝 NaN 和正负无穷大。
        void ValidateFinite(const char *name, double value)
        {
            if (!std::isfinite(value))
            {
                throw std::invalid_argument(
                    std::string(name) + " must be finite.");
            }
        }

        // 检查物理尺寸是否为有限正数。
        void ValidatePositiveFinite(const char *name, double value)
        {
            if (!std::isfinite(value) || value <= 0.0)
            {
                throw std::invalid_argument(
                    std::string(name) + " must be positive and finite.");
            }
        }

        // 检查底盘速度指令的数值是否合法。
        void ValidateCommand(const ChassisCommand &command)
        {
            ValidateFinite("linear_x_mps", command.linear_x_mps);
            ValidateFinite("linear_y_mps", command.linear_y_mps);
            ValidateFinite("angular_z_radps", command.angular_z_radps);
        }

        // 将车轮接触地面的线速度转换为 RPM。
        double WheelRpmFromLinear(
            double speed_mps,
            double wheel_diameter_m)
        {
            ValidateFinite("speed_mps", speed_mps);
            ValidatePositiveFinite(
                "wheel_diameter_m", wheel_diameter_m);

            // 轮子每秒转数 = 线速度 / 圆周长
            // RPM = 每秒转数 * 60
            return speed_mps /
                   (kPi * wheel_diameter_m) * 60.0;
        }

        // 将车轮 RPM 转换为车轮接触地面的线速度。
        double WheelLinearFromRpm(
            double rpm,
            double wheel_diameter_m)
        {
            ValidateFinite("wheel_rpm", rpm);
            ValidatePositiveFinite(
                "wheel_diameter_m", wheel_diameter_m);

            return rpm / 60.0 *
                   kPi * wheel_diameter_m;
        }

    } // namespace

    // 将模型名称统一为标准名称。
    std::string NormalizeKinematicsModel(
        const std::string &model_name)
    {
        if (model_name == "diff_drive" ||
            model_name == "differential_drive")
        {
            return "diff_drive";
        }

        if (model_name == "mecanum" ||
            model_name == "Mecanum")
        {
            return "mecanum";
        }

        throw std::invalid_argument(
            "Unsupported kinematics model: " + model_name +
            ". Supported models: diff_drive, mecanum.");
    }

    // 根据机体坐标系速度积分机器人位姿。
    // 使用区间中点的朝向估算位置变化，降低转弯时的积分误差。
    PlanarPose IntegratePlanarMotion(
        const PlanarPose &pose,
        const ChassisCommand &command,
        double dt_seconds)
    {
        ValidateFinite("pose.x_m", pose.x_m);
        ValidateFinite("pose.y_m", pose.y_m);
        ValidateFinite("pose.yaw_rad", pose.yaw_rad);
        ValidateCommand(command);

        if (!std::isfinite(dt_seconds) || dt_seconds < 0.0)
        {
            throw std::invalid_argument(
                "dt_seconds must be finite and non-negative.");
        }

        const double dt = dt_seconds;

        // 区间中点的航向角。
        const double yaw_mid =
            pose.yaw_rad +
            command.angular_z_radps * dt * 0.5;

        const double cos_yaw = std::cos(yaw_mid);
        const double sin_yaw = std::sin(yaw_mid);

        PlanarPose result = pose;

        // 将机体坐标系速度转换成世界坐标系速度，再积分位置。
        result.x_m +=
            (command.linear_x_mps * cos_yaw -
             command.linear_y_mps * sin_yaw) *
            dt;

        result.y_m +=
            (command.linear_x_mps * sin_yaw +
             command.linear_y_mps * cos_yaw) *
            dt;

        // 积分航向角。
        result.yaw_rad =
            pose.yaw_rad +
            command.angular_z_radps * dt;

        // 将航向角归一化至 [-pi, pi]。
        result.yaw_rad = std::atan2(
            std::sin(result.yaw_rad),
            std::cos(result.yaw_rad));

        return result;
    }

    // 根据底盘速度计算轮子转速。
    WheelSpeeds EstimateWheelSpeeds(
        const ChassisCommand &command,
        const std::string &model,
        double wheel_diameter_m,
        double wheel_base_m,
        double track_width_m)
    {
        const std::string normalized_model =
            NormalizeKinematicsModel(model);

        ValidateCommand(command);
        ValidatePositiveFinite(
            "wheel_diameter_m", wheel_diameter_m);
        ValidatePositiveFinite(
            "track_width_m", track_width_m);

        WheelSpeeds speeds;

        if (normalized_model == "mecanum")
        {
            ValidatePositiveFinite(
                "wheel_base_m", wheel_base_m);

            // 假设 wheel_base 和 track_width
            // 分别是前后方向及左右方向的完整车轮间距。
            const double rotation_radius =
                (wheel_base_m + track_width_m) * 0.5;

            // 各轮接触地面的有符号线速度。
            const double fl =
                command.linear_x_mps -
                command.linear_y_mps -
                command.angular_z_radps * rotation_radius;

            const double fr =
                command.linear_x_mps +
                command.linear_y_mps +
                command.angular_z_radps * rotation_radius;

            const double rl =
                command.linear_x_mps +
                command.linear_y_mps -
                command.angular_z_radps * rotation_radius;

            const double rr =
                command.linear_x_mps -
                command.linear_y_mps +
                command.angular_z_radps * rotation_radius;

            speeds.rpm = {
                WheelRpmFromLinear(fl, wheel_diameter_m),
                WheelRpmFromLinear(fr, wheel_diameter_m),
                WheelRpmFromLinear(rl, wheel_diameter_m),
                WheelRpmFromLinear(rr, wheel_diameter_m)};

            return speeds;
        }

        // 经过 NormalizeKinematicsModel() 后，
        // 这里仅剩 diff_drive 模型。
        // 差速底盘不支持横向平移。
        if (std::abs(command.linear_y_mps) >
            kVelocityEpsilon)
        {
            throw std::invalid_argument(
                "diff_drive does not support lateral velocity.");
        }

        const double half_track = track_width_m * 0.5;

        const double left =
            command.linear_x_mps -
            command.angular_z_radps * half_track;

        const double right =
            command.linear_x_mps +
            command.angular_z_radps * half_track;

        speeds.rpm = {
            WheelRpmFromLinear(left, wheel_diameter_m),
            WheelRpmFromLinear(right, wheel_diameter_m),
            WheelRpmFromLinear(left, wheel_diameter_m),
            WheelRpmFromLinear(right, wheel_diameter_m)};

        return speeds;
    }

    // 根据四轮转速反推底盘速度。
    ChassisCommand EstimateCommandFromWheelSpeeds(
        const WheelSpeeds &speeds,
        const std::string &model,
        double wheel_diameter_m,
        double wheel_base_m,
        double track_width_m)
    {
        const std::string normalized_model =
            NormalizeKinematicsModel(model);

        ValidatePositiveFinite(
            "wheel_diameter_m", wheel_diameter_m);
        ValidatePositiveFinite(
            "track_width_m", track_width_m);

        // 将各轮 RPM 转换为有符号的接触地面线速度。
        const double fl =
            WheelLinearFromRpm(
                speeds.rpm[0], wheel_diameter_m);

        const double fr =
            WheelLinearFromRpm(
                speeds.rpm[1], wheel_diameter_m);

        const double rl =
            WheelLinearFromRpm(
                speeds.rpm[2], wheel_diameter_m);

        const double rr =
            WheelLinearFromRpm(
                speeds.rpm[3], wheel_diameter_m);

        ChassisCommand command;

        if (normalized_model == "mecanum")
        {
            ValidatePositiveFinite(
                "wheel_base_m", wheel_base_m);

            const double rotation_radius =
                (wheel_base_m + track_width_m) * 0.5;

            command.linear_x_mps =
                (fl + fr + rl + rr) * 0.25;

            command.linear_y_mps =
                (-fl + fr + rl - rr) * 0.25;

            command.angular_z_radps =
                (-fl + fr - rl + rr) /
                (4.0 * rotation_radius);

            return command;
        }

        // 差速底盘：先平均左右两侧轮速。
        const double left = (fl + rl) * 0.5;
        const double right = (fr + rr) * 0.5;

        command.linear_x_mps =
            (left + right) * 0.5;

        command.linear_y_mps = 0.0;

        command.angular_z_radps =
            (right - left) / track_width_m;

        return command;
    }

} // namespace robot_hardware