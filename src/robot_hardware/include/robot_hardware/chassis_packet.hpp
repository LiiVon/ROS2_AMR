// chassis_packet
// 机器人底盘硬件的 通信数据包定义与协议编解码
#pragma once

#include <cstddef>
#include <cstdint>
#include <optional>
#include <string>
#include <vector>

namespace robot_hardware
{
    // ======================== 数据结构定义 =========================
    // 底盘运动指令
    struct ChassisCommand
    {
        double linear_x_maps = 0.0;  // 机器人在x方向的线速度（单位：米/秒）
        double linear_y_maps = 0.0;  // 机器人在y方向的线速度（单位：米/秒）
        double angular_z_rads = 0.0; // 机器人绕z轴的角速度（单位：弧度/秒）
    };

    // 底盘里程计数据
    struct ChassisOdometryPacket
    {
        double x_m = 0.0;     // 机器人在世界坐标系下的x坐标（单位：米）
        double y_m = 0.0;     // 机器人在世界坐标系下的y坐标（单位：米）
        double yaw_rad = 0.0; // 机器人在世界坐标系下的航向角（单位：弧度）

        double linear_x_maps = 0.0;  // 机器人在x方向的线速度（单位：米/秒）
        double linear_y_maps = 0.0;  // 机器人在y方向的线速度（单位：米/秒）
        double angular_z_rads = 0.0; // 机器人绕z轴的角速度（单位：弧度/秒）

        std::optional<double> battery_voltage; // 电池电压（单位：伏特），可选字段
    };

    // 底盘状态数据
    struct ChassisStatePacket
    {
        std::string status;                    // 底盘状态描述，例如“正常”、“故障”等
        std::optional<double> battery_voltage; // 电池电压（单位：伏特），可选字段
    };

    // 数据包类型
    enum class ChassisPacketKind
    {
        kOdometry, // 里程计数据包
        kState,    // 状态数据包
    };

    // 统一的数据包容器
    struct ChassisPacket
    {
        // 默认是状态数据包
        ChassisPacketKind kind = ChassisPacketKind::kState;
        ChassisOdometryPacket odometry;
        ChassisStatePacket state;
    };

    // ======================== 文本协议的编码与解码 =========================

    std::string EncodeCommand(const ChassisCommand &command);
    std::string EncodeOdometry(const ChassisOdometryPacket &odometry);
    std::string EncodeState(const ChassisStatePacket &state);

    std::optional<ChassisCommand> DecodeCommand(const std::string &text);
    std::optional<ChassisPacket> DecodePacket(const std::string &text);

    // ======================== 二进制协议的编码与解码 =========================
    struct MickBinaryFrame
    {
        uint8_t type = 0;          // 数据包类型
        std::vector<uint8_t> data; // 二进制数据
    };

    // 将帧类型和数据内容组装成二进制数据帧
    std::vector<uint8_t> EncodeMickBinaryFrame(uint8_t type, const std::vector<uint8_t> &data);

    // 将底盘速度指令转换成 Mick 协议所需要的二进制数据
    std::vector<uint8_t> EncodeMickSpeedCommand(const ChassisCommand &command);

    // 生成清零里程计的二进制控制指令
    std::vector<uint8_t> EncodeMickClearOdometryCommand();

    // 将解析好的二进制帧解码成统一的 ChassisPacket 对象
    std::optional<ChassisPacket> DecodeMickBinaryPacket(const MickBinaryFrame &frame);

    // ======================== 文本数据流解析器 =========================
    class ChassisPacketStreamParser
    {
    public:
        explicit ChassisPacketStreamParser(std::size_t max_buffer_size = 4096);
        void AppendData(const std::string &data);
        void AppendData(const char *data, std::size_t size);

        std::vector<ChassisPacket> DrainPackets();

        bool Overflowed() const;

        std::size_t InvalidLineCount() const;

        void ClearOverflow();
        void ClearInvalidLineCount();
        void Clear();
    };
}