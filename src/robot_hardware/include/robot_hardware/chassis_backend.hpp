// chassis_backend
// 底盘通信后端  抽象接口层
#pragma once

#include <string>
#include <algorithm>

#include "robot_hardware/chassis_packet.hpp"

namespace robot_hardware
{
    struct ChassisBackendConfig
    {
        std::string serial_device = "/dev/ttyUSB0"; // 串口设备路径
        int serial_baudrate = 115200;               // 串口波特率

        std::string udp_host = "192.168.1.10"; // UDP 目标主机
        int udp_port = 9000;

        std::string protocol = "text"; // 协议类型，支持 "text" 或 "binary"
    };

    class ChassisBackend
    {
    public:
        virtual ~ChassisBackend() = default;

        virtual std::string Name() const = 0;
        virtual bool Open(std::string*error) = 0;
        virtual void Close() = 0;
        virtual bool IsOpen() const = 0;
        virtual bool WriteCommand(const ChassisCommand &command, std::string*error) = 0;
        virtual std::optional<std::string> Read(std::string*error) = 0;
    };
}