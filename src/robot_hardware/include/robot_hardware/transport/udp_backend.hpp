#pragma once

#include <netinet/in.h>

#include "robot_hardware/transport/backend.hpp"

namespace robot_hardware::transport
{
    using protocol::Command;


    class UdpBackend final : public Backend
    {
    public:
        explicit UdpBackend(BackendConfig config);
        ~UdpBackend() override;

        std::string Name() const override;
        BackendKind Kind() const override;
        bool Open(std::string *error) override;
        void Close() override;
        bool IsOpen() const override;
        bool WriteCommand(const Command &command, std::string *error) override;
        std::optional<std::string> Read(std::string *error) override;

    private:
        BackendConfig config_;
        int fd_ = -1;
        sockaddr_in remote_addr_{};
    };

} // namespace robot_hardware
