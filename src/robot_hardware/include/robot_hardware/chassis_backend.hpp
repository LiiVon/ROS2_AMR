// chassis_backend.hpp
// 底盘通信后端抽象接口。
// 定义统一的底盘配置，以及所有后端必须实现的操作。

#pragma once

#include <optional>
#include <string>

#include "robot_hardware/chassis_packet.hpp"

namespace robot_hardware
{

    // 底盘后端配置。
    // SerialBackend 使用串口相关配置；UdpBackend 使用 UDP 相关配置。
    // protocol 供底盘数据协议解析层使用。
    struct ChassisBackendConfig
    {
        // 串口配置
        std::string serial_device{"/dev/ttyUSB0"};
        int serial_baud{115200};

        // UDP 配置
        std::string udp_host{"192.168.1.10"};
        int udp_port{9000};

        // 底盘数据协议：
        // "text"       : 文本协议
        // "mick_binary": Mick 二进制协议
        std::string protocol{"text"};
    };

    // 底盘通信后端抽象基类。
    // Mock、Serial、UDP 等具体后端都必须实现该接口。
    class ChassisBackend
    {
    public:
        // 通过基类指针销毁具体后端对象时，确保调用派生类析构函数。
        virtual ~ChassisBackend() = default;

        // 返回后端名称，例如 "mock"、"serial" 或 "udp"。
        virtual std::string Name() const = 0;

        // 打开底盘通信后端。
        // 成功返回 true；失败返回 false，并尽可能填写 error。
        virtual bool Open(std::string *error) = 0;

        // 关闭底盘通信后端并释放相关通信资源。
        virtual void Close() = 0;

        // 查询后端是否处于打开状态。
        virtual bool IsOpen() const = 0;

        // 向底盘发送运动指令。
        // command 是要发送的底盘指令；失败时尽可能填写 error。
        virtual bool WriteCommand(
            const ChassisCommand &command,
            std::string *error) = 0;

        // 读取底盘通信数据。
        // 有数据时返回数据字符串；无数据时返回 std::nullopt。
        // 发生错误时尽可能填写 error。
        virtual std::optional<std::string> Read(
            std::string *error) = 0;
    };

} // namespace robot_hardware