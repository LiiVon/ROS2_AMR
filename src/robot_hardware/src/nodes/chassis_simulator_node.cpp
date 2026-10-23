/**
 * @file chassis_simulator_node.cpp
 * @brief 底盘模拟器节点（节点入口层）。
 * @details chassis_simulator_node.cpp 底盘模拟器 ROS 2 节点 这个节点模拟底盘的运动学和动力学行为，接收来自上层的运动指令，并根据底盘模型计算轮子速度和位置 同时发布模拟的 底盘状态和里程计信息
 * @details 所属包：`robot_hardware`；所在层次：节点入口层。
 * @note 教学说明见 `docs/教学/` 对应章节。
 */

#include <algorithm>
#include <chrono>
#include <cmath>
#include <cerrno>
#include <cstring>
#include <optional>
#include <string>

#include <arpa/inet.h>
#include <netinet/in.h>
#include <sys/socket.h>
#include <unistd.h>

#include "rclcpp/rclcpp.hpp"
#include "robot_hardware/kinematics/kinematics.hpp"
#include "robot_hardware/protocol/packet.hpp"

using namespace std::chrono_literals;
using namespace robot_hardware::protocol;
using namespace robot_hardware::kinematics;


namespace
{
    // 接收缓冲区超时时间，单位为秒
    constexpr double kReceiveBufferTimeoutSeconds = 0.5;

    // 创建一个绑定到指定主机和端口的 UDP 套接字地址
    std::optional<sockaddr_in> MakeBindAddress(const std::string &host, const int port)
    {
        sockaddr_in addr{};
        addr.sin_family = AF_INET;
        addr.sin_port = htons(static_cast<uint16_t>(port));
        if (inet_pton(AF_INET, host.c_str(), &addr.sin_addr) != 1)
        {
            return std::nullopt;
        }
        return addr;
    }
}

class ChassisSimulatorNode final : public rclcpp::Node
{
public:
    ChassisSimulatorNode() : Node("chassis_simulator_node")
    {
        bind_host_ = declare_parameter<std::string>("bind_host", "0.0.0.0");
        port_ = declare_parameter<int>("port", 9000);
        battery_voltage_ = declare_parameter<double>("battery_voltage", 24.0);
        OpenSocket();
        last_update_ = now();
        last_rx_buffer_update_ = last_update_;
        timer_ = create_wall_timer(20ms, [this]()
                                   { OnTimer(); });
    }

    ~ChassisSimulatorNode()
    {
        if (fd_ >= 0)
        {
            close(fd_);
        }
    }

private:
    void OpenSocket()
    {
        fd_ = socket(AF_INET, SOCK_DGRAM | SOCK_NONBLOCK, 0);
        if (fd_ < 0)
        {
            RCLCPP_ERROR(get_logger(), "Failed to create UDP socket: %s", std::strerror(errno));
            return;
        }

        const int reuse = 1;
        setsockopt(fd_, SOL_SOCKET, SO_REUSEADDR, &reuse, sizeof(reuse));
        const auto addr = MakeBindAddress(bind_host_, port_);
        if (!addr.has_value())
        {
            RCLCPP_ERROR(get_logger(), "Invalid bind address: %s:%d", bind_host_.c_str(), port_);
            close(fd_);
            fd_ = -1;
            return;
        }

        if (bind(fd_, reinterpret_cast<const sockaddr *>(&*addr), sizeof(*addr)) < 0)
        {
            RCLCPP_ERROR(get_logger(), "Failed to bind UDP socket to %s:%d: %s", bind_host_.c_str(), port_, std::strerror(errno));
            close(fd_);
            fd_ = -1;
            return;
        }

        RCLCPP_INFO(get_logger(), "UDP socket bound to %s:%d", bind_host_.c_str(), port_);
    }

    void OnTimer()
    {
        const auto stamp = now();
        const double dt = std::max(0.001, (stamp - last_update_).seconds());
        last_update_ = stamp;

        ReadCommands(stamp);
        pose_ = IntegratePose(pose_, command_, dt);
        PublishPackets();
    }

    void ReadCommands(const rclcpp::Time &stamp)
    {
        if (fd_ < 0)
        {
            return;
        }
        if (!rx_buffer_.empty() &&
            (stamp - last_rx_buffer_update_).seconds() > kReceiveBufferTimeoutSeconds)
        {
            rx_buffer_.clear();
            RCLCPP_WARN(get_logger(), "cleared stale simulator receive buffer");
        }
        char buffer[512];
        sockaddr_in from{};
        socklen_t from_len = sizeof(from);
        const ssize_t received =
            recvfrom(fd_, buffer, sizeof(buffer), 0, reinterpret_cast<sockaddr *>(&from), &from_len);
        if (received < 0)
        {
            if (errno != EAGAIN && errno != EWOULDBLOCK)
            {
                RCLCPP_WARN_THROTTLE(
                    get_logger(), *get_clock(), 2000, "simulator UDP receive failed: %s",
                    std::strerror(errno));
            }
            return;
        }
        if (received == 0)
        {
            return;
        }

        remote_addr_ = from;
        has_remote_ = true;
        rx_buffer_.append(buffer, static_cast<std::size_t>(received));
        last_rx_buffer_update_ = stamp;
        std::size_t newline = std::string::npos;
        while ((newline = rx_buffer_.find('\n')) != std::string::npos)
        {
            const std::string line = rx_buffer_.substr(0, newline);
            rx_buffer_.erase(0, newline + 1);
            if (auto command = DecodeCommand(line))
            {
                command_ = *command;
            }
        }
        if (rx_buffer_.size() > 1024)
        {
            rx_buffer_.clear();
            RCLCPP_WARN(get_logger(), "cleared oversized simulator receive buffer");
        }
    }

    void PublishPackets()
    {
        if (fd_ < 0 || !has_remote_)
        {
            return;
        }
        OdometryPacket odom;
        odom.x_m = pose_.x_m;
        odom.y_m = pose_.y_m;
        odom.yaw_rad = pose_.yaw_rad;
        odom.linear_x_mps = command_.linear_x_mps;
        odom.angular_z_radps = command_.angular_z_radps;
        odom.battery_voltage = battery_voltage_;
        StatePacket state;
        state.status = "simulated";
        state.battery_voltage = battery_voltage_;
        const std::string packet = EncodeOdometry(odom) + EncodeState(state);
        const ssize_t sent =
            sendto(fd_, packet.data(), packet.size(), 0,
                   reinterpret_cast<const sockaddr *>(&remote_addr_), sizeof(remote_addr_));
        if (sent < 0 && errno != EAGAIN && errno != EWOULDBLOCK)
        {
            RCLCPP_WARN_THROTTLE(
                get_logger(), *get_clock(), 2000, "simulator UDP send failed: %s", std::strerror(errno));
        }
    }

private:
    std::string bind_host_;                  // UDP 绑定的主机地址
    int port_ = 9000;                        // UDP 绑定的端口号
    int fd_ = -1;                            // UDP 套接字文件描述符
    sockaddr_in remote_addr_{};              // 远程主机的 UDP 地址
    bool has_remote_ = false;                // 是否已经接收到远程主机的 UDP 数据
    std::string rx_buffer_;                  // 接收缓冲区，用于存储接收到的 UDP 数据
    Pose2D pose_;        // 机器人在平面上的位姿，包括 x、y 坐标和航向角
    double battery_voltage_ = 24.0;          // 机器人电池电压（单位：伏特）
    Command command_; // 当前的底盘运动指令，包括线速度和角速度
    rclcpp::Time last_update_;               // 上一次更新的时间戳
    rclcpp::Time last_rx_buffer_update_;     // 上一次接收到 UDP 数据的时间戳
    rclcpp::TimerBase::SharedPtr timer_;     // 定时器，用于周期性地调用 OnTimer() 函数
};

int main(int argc, char **argv)
{
    rclcpp::init(argc, argv);
    rclcpp::spin(std::make_shared<ChassisSimulatorNode>());
    rclcpp::shutdown();
    return 0;
}
