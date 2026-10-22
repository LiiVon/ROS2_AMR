// chassis_packet
// 机器人底盘硬件的 通信数据包定义与协议编解码
#pragma once

#include <cstddef>
#include <cstdint>
#include <optional>
#include <string>
#include <vector>

namespace robot_hardware::protocol
{

    // ======================== 数据结构定义 =========================
    // 底盘运动指令
    struct Command
    {
        double linear_x_mps = 0.0;    // 机器人在x方向的线速度（单位：米/秒）
        double linear_y_mps = 0.0;    // 机器人在y方向的线速度（单位：米/秒）
        double angular_z_radps = 0.0; // 机器人绕z轴的角速度（单位：弧度/秒）
    };

    // 底盘里程计数据
    struct OdometryPacket
    {
        double x_m = 0.0;     // 机器人在世界坐标系下的x坐标（单位：米）
        double y_m = 0.0;     // 机器人在世界坐标系下的y坐标（单位：米）
        double yaw_rad = 0.0; // 机器人在世界坐标系下的航向角（单位：弧度）

        double linear_x_mps = 0.0;    // 机器人在x方向的线速度（单位：米/秒）
        double linear_y_mps = 0.0;    // 机器人在y方向的线速度（单位：米/秒）
        double angular_z_radps = 0.0; // 机器人绕z轴的角速度（单位：弧度/秒）

        std::optional<double> battery_voltage; // 电池电压（单位：伏特），可选字段
    };

    // 底盘状态数据
    struct StatePacket
    {
        std::string status;                    // 底盘状态描述，例如“正常”、“故障”等
        std::optional<double> battery_voltage; // 电池电压（单位：伏特），可选字段
    };

    // 数据包类型
    enum class PacketKind
    {
        kOdometry, // 里程计数据包
        kState,    // 状态数据包
    };

    // 统一的数据包容器
    struct Packet
    {
        // 默认是状态数据包
        PacketKind kind = PacketKind::kState;
        OdometryPacket odometry;
        StatePacket state;
    };

    // ======================== 文本协议的编码与解码 =========================

    std::string EncodeCommand(const Command &command);
    std::string EncodeOdometry(const OdometryPacket &odometry);
    std::string EncodeState(const StatePacket &state);

    std::optional<Command> DecodeCommand(const std::string &text);
    std::optional<Packet> DecodePacket(const std::string &text);

    // ======================== 二进制协议的编码与解码 =========================
    struct BinaryFrame
    {
        uint8_t type = 0;          // 数据包类型
        std::vector<uint8_t> data; // 二进制数据
    };

    // 将帧类型和数据内容组装成二进制数据帧
    std::vector<uint8_t> EncodeBinaryFrame(const uint8_t type, const std::vector<uint8_t> &data);

    // 将底盘速度指令转换成 Mick 协议所需要的二进制数据
    std::vector<uint8_t> EncodeMickSpeedCommand(const Command &command);

    // 生成清零里程计的二进制控制指令
    std::vector<uint8_t> EncodeMickClearOdometryCommand();

    // 将解析好的二进制帧解码成统一的 Packet 对象
    std::optional<Packet> DecodeMickBinaryPacket(const BinaryFrame &frame);

    // ======================== 文本数据流解析器 =========================
    class PacketStream
    {
    public:
        explicit PacketStream(std::size_t max_buffer_size = 4096);

        // 向解析器中追加数据
        void Append(const std::string &data);
        void Append(const char *data, std::size_t size);

        // 取出完整数据包
        std::vector<Packet> DrainPackets();

        bool Overflowed() const;

        std::size_t InvalidLineCount() const;

        // 清除溢出标记、清除无效行计数，清空整个解析器状态
        void ClearOverflow();
        void ClearInvalidLineCount();
        void Clear();

    private:
        std::size_t max_buffer_size_;
        std::string buffer_;
        bool overflowed_ = false;
        std::size_t invalid_line_count_ = 0;
    };

    // ======================== 二进制数据流解析器 =========================
    class BinaryPacketStream
    {
    public:
        explicit BinaryPacketStream(std::size_t max_buffer_size = 4096);

        // 向解析器中追加数据
        void Append(const char *data, std::size_t size);
        void Append(const std::string &data);

        // 取出完整数据包
        std::vector<Packet> DrainPackets();

        bool Overflowed() const;

        std::size_t InvalidFrameCount() const;

        // 清除溢出标记、清除无效帧计数，清空整个解析器状态
        void ClearOverflow();
        void ClearInvalidFrameCount();
        void Clear();

    private:
        std::size_t max_buffer_size_;
        std::vector<uint8_t> buffer_;
        bool overflowed_ = false;
        std::size_t invalid_frame_count_ = 0;
    };
}