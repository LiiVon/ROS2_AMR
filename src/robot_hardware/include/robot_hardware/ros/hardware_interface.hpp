/**
 * @file hardware_interface.hpp
 * @brief 硬件接口（ROS 适配层）。
 * @details chassis_hardware_interface.hpp 底盘硬件接口类，继承自ROS2的SystemInterface，提供对底盘硬件的抽象访问
 * @details 所属包：`robot_hardware`；所在层次：ROS 适配层。
 * @note 教学说明见 `docs/教学/` 对应章节。
 */

#pragma once

#include <array>
#include <memory>
#include <string>
#include <vector>

#include "robot_hardware/transport/backend.hpp"
#include "robot_hardware/system/adapter.hpp"
#include "hardware_interface/handle.hpp"
#include "hardware_interface/hardware_info.hpp"
#include "hardware_interface/system_interface.hpp"
#include "hardware_interface/types/hardware_component_interface_params.hpp"
#include "hardware_interface/types/hardware_interface_return_values.hpp"
#include "rclcpp_lifecycle/state.hpp"

namespace robot_hardware
{
    using protocol::Command;
    using transport::BackendConfig;
    using system::Adapter;
    using system::Config;
    using system::Snapshot;
    using kinematics::WheelSpeeds;
    using kinematics::EstimateChassisCommand;

    // 底盘硬件接口类，继承自ROS2的SystemInterface，提供对底盘硬件的抽象访问
    /**
     * @brief 底盘硬件接口（ChassisHardwareInterface）
     */
    class ChassisHardwareInterface final : public hardware_interface::SystemInterface
    {
    public:
        // 初始化 读取硬件描述和参数，初始化必要的数据，检查配置是否有效
        hardware_interface::CallbackReturn on_init(
            const hardware_interface::HardwareComponentInterfaceParams &params) override;

        // 加载配置、准备通信资源和创建适配器
        hardware_interface::CallbackReturn on_configure(
            const rclcpp_lifecycle::State &previous_state) override;

        // 激活 使硬件进入可供控制的状态
        hardware_interface::CallbackReturn on_activate(
            const rclcpp_lifecycle::State &previous_state) override;

        // 停止控制器，释放资源
        hardware_interface::CallbackReturn on_deactivate(
            const rclcpp_lifecycle::State &previous_state) override;

        // 导出状态接口和命令接口
        std::vector<hardware_interface::StateInterface> export_state_interfaces() override;
        std::vector<hardware_interface::CommandInterface> export_command_interfaces() override;

        hardware_interface::return_type read(
            const rclcpp::Time &time, const rclcpp::Duration &period) override;
        hardware_interface::return_type write(
            const rclcpp::Time &time, const rclcpp::Duration &period) override;

    private:
        // 加载参数、验证关节配置、计算指令和更新轮子状态
        bool LoadParameters();

        // 验证关节配置是否正确
        bool ValidateJoints() const;

        // 根据轮子速度命令计算底盘运动指令
        Command WheelCommandsToChassisCommand() const;

        // 根据底盘状态更新轮子位置和速度
        void UpdateWheelStates(const Snapshot &state, double period_seconds);

        BackendConfig backend_config_;                                             // 底盘后端配置
        Config adapter_config_;                                       // 底盘系统适配器配置
        std::string calibration_profile_ = "nominal_reference";                           // 校准配置文件名
        std::string backend_name_ = "mock";                                               // 底盘通信后端名称，默认使用模拟后端
        std::array<std::string, 2> joint_names_{"left_wheel_joint", "right_wheel_joint"}; // 轮子关节名称
        std::array<double, 2> wheel_position_rad_{0.0, 0.0};                              // 轮子位置（单位：弧度）
        std::array<double, 2> wheel_velocity_radps_{0.0, 0.0};                            // 轮子速度（单位：弧度/秒）
        std::array<double, 2> wheel_command_radps_{0.0, 0.0};                             // 轮子命令（单位：弧度/秒）
        std::unique_ptr<Adapter> adapter_;                                   // 底盘系统适配器，用于处理底盘通信和状态更新
    };
}