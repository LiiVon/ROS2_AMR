#include "robot_hardware/transport/udp_backend.hpp"

#include <arpa/inet.h>
#include <cerrno>
#include <cstring>
#include <string>
#include <sys/socket.h>
#include <unistd.h>
#include <utility>

namespace robot_hardware::transport
{
    using protocol::Command;
    using protocol::EncodeCommand;
    using protocol::EncodeMickSpeedCommand;

    UdpBackend::UdpBackend(BackendConfig config) : config_(std::move(config)) {}

    UdpBackend::~UdpBackend()
    {
        Close();
    }

    std::string UdpBackend::Name() const
    {
        return "udp";
    }

    BackendKind UdpBackend::Kind() const
    {
        return BackendKind::kUdp;
    }

    bool UdpBackend::Open(std::string *error)
    {
        Close();
        fd_ = socket(AF_INET, SOCK_DGRAM | SOCK_NONBLOCK, 0);
        if (fd_ < 0)
        {
            if (error)
            {
                *error = "Failed to create UDP socket: " + std::string(strerror(errno));
            }
            return false;
        }

        remote_addr_ = {};
        remote_addr_.sin_family = AF_INET;
        remote_addr_.sin_port = htons(static_cast<uint16_t>(config_.udp_port));
        if (inet_pton(AF_INET, config_.udp_host.c_str(), &remote_addr_.sin_addr) <= 0)
        {
            if (error)
            {
                *error = "Invalid UDP host address: " + config_.udp_host;
            }
            Close();
            return false;
        }
        return true;
    }

    void UdpBackend::Close()
    {
        if (fd_ >= 0)
        {
            close(fd_);
            fd_ = -1;
        }
    }
    
    bool UdpBackend::IsOpen() const
    {
        return fd_ >= 0;
    }

    bool UdpBackend::WriteCommand(const Command &command, std::string *error)
    {
        if (!IsOpen())
        {
            if (error)
            {
                *error = "UDP socket is not open";
            }
            return false;
        }

        const auto binary_packet = EncodeMickSpeedCommand(command);
        const std::string text_packet = EncodeCommand(command);

        const char *data = config_.protocol == "mick_binary" ? reinterpret_cast<const char *>(binary_packet.data()) : text_packet.data();
        const std::size_t size = config_.protocol == "mick_binary" ? binary_packet.size() : text_packet.size();

        const ssize_t sent_bytes = sendto(fd_, data, size, 0, reinterpret_cast<sockaddr *>(&remote_addr_), sizeof(remote_addr_));

        if (sent_bytes < 0 && errno != EAGAIN && errno != EWOULDBLOCK)
        {
            if (error != nullptr)
            {
                *error = std::strerror(errno);
            }
            return false;
        }
        return sent_bytes >= 0 || errno == EAGAIN || errno == EWOULDBLOCK;
    }

    std::optional<std::string> UdpBackend::Read(std::string *error)
    {
        if (!IsOpen())
        {
            return std::nullopt;
        }
        char buffer[512];
        sockaddr_in from{};
        socklen_t from_len = sizeof(from);
        const ssize_t received =
            recvfrom(fd_, buffer, sizeof(buffer), 0, reinterpret_cast<sockaddr *>(&from), &from_len);
        if (received < 0)
        {
            if (errno != EAGAIN && errno != EWOULDBLOCK && error != nullptr)
            {
                *error = std::strerror(errno);
            }
            return std::nullopt;
        }
        if (received == 0)
        {
            return std::nullopt;
        }
        return std::string(buffer, static_cast<std::size_t>(received));
    }
}