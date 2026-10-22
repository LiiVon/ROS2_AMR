// chassis_system_adapter.hpp
// 底盘通信层与上层 ROS2 控制逻辑的适配器
// 统一管理底盘通信、运动学计算、状态读取和里程计的中间层
#pragma once
#include <memory>
#include <string>

#include "robot_hardware/transport/backend.hpp"
#include "robot_hardware/kinematics/kinematics.hpp"
#include "robot_hardware/protocol/packet.hpp"

namespace robot_hardware::system
{
    using protocol::Command;
    using protocol::Packet;
    using protocol::PacketKind;
    using protocol::PacketStream;
    using protocol::BinaryPacketStream;
    using transport::Backend;
    using transport::BackendConfig;
    using kinematics::Pose2D;
    using kinematics::WheelSpeeds;
    using kinematics::IntegratePose;
    using kinematics::EstimateWheelSpeeds;
    using kinematics::NormalizeModel;

    // 配置结构体
    struct Config
    {
        std::string calibration_profile = "nominal_reference"; // 校准配置文件名
        std::string kinematics_model = "diff_drive";           // 运动学模型名称
        std::string protocol = "text";                         // 底盘通信协议名称

        double wheel_diameter_m = 0.15;         // 车轮直径（单位：米）
        double wheel_base_m = 0.42;             // 轴距（前后车轴的距离，单位：米）
        double track_width_m = 0.43;            // 轮距（左右车轮的距离，单位：米）
        double left_encoder_scale = 1.0;        // 左轮编码器缩放系数
        double right_encoder_scale = 1.0;       // 右轮编码器缩放系数
        int left_direction_sign = 1;            // 左轮编码器方向符号（1表示正向，-1表示反向）
        int right_direction_sign = 1;           // 右轮编码器方向符号 （1表示正向，-1表示反向）
        double fallback_battery_voltage = 24.0; // 备用电池电压（单位：伏特）
    };

    // 配置校验
    bool ValidateConfig(const Config &config, std::string *error);

    // 统一状态快照
    struct Snapshot
    {
        std::string backend_name;                // 底盘通信协议名称
        std::string hardware_status = "running"; // 底盘硬件状态，默认值为 "running"
        bool backend_open = false;               // 底盘通信协议是否已打开，默认值为 false
        bool connected = false;                  // 底盘是否已连接，默认值为 false

        double x_m = 0.0;              // 机器人在世界坐标系下的x坐标
        double y_m = 0.0;              // 机器人在世界坐标系下的y坐标
        double yaw_rad = 0.0;          // 机器人在世界坐标系下的航向角（弧度）
        double linear_x_mps = 0.0;     // 机器人在世界坐标系下的线速度（单位：米/秒）
        double linear_y_mps = 0.0;     // 机器人在世界坐标系下的线速度（单位：米/秒）
        double angular_z_radps = 0.0;  // 机器人在世界坐标系下的角速度（单位：rad/秒）
        double battery_voltage = 24.0; // 机器人电池电压（单位：伏特）

        WheelSpeeds wheel_speeds;               // 机器人四个轮子的转速（单位：转/分钟）
        unsigned long long packet_sequence = 0; // 底盘通信数据包的序列号，默认值为 0
    };

    class Adapter
    {
    public:
        // 注入通信后端并加载配置
        Adapter(std::unique_ptr<Backend> backend, Config config = {});

        std::string BackendName() const;
        // 打开 关闭 通信后端
        bool Open(std::string *error);
        void Close();
        bool IsOpen() const;

        // 切换运动学模型
        void SetKinematicsModel(const std::string &model);

        // 处理并发送底盘运动指令
        bool Write(const Command &command, std::string *error);

        // 读取底盘数据并更新状态
        bool Read(double period_seconds, Snapshot *state, std::string *error);

        // 重置里程计状态
        void ResetOdometry();

        // 获取当前状态快照的只读引用
        const Snapshot &State() const;

    private:
        // 规范化指令
        Command NormalizeCommand(const Command &command) const;

        // 积分模拟运动
        void IntegrateMockMotion(double period_seconds);

        // 读取底层数据包
        bool ReadBackend(std::string *error);

        // 将数据包应用到机器人状态
        void ApplyPacket(const Packet &packet);

        // 刷新后端打开状态
        void RefreshBackendState();

        std::unique_ptr<Backend> backend_; // 通信后端
        Config config_;       // 运行配置
        Snapshot state_;                // 当前状态
        Command command_;                  // 当前指令

        PacketStream text_stream_;      // 文本协议数据流
        BinaryPacketStream binary_stream_; // Mick 二进制数据流
    };
}