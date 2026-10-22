#pragma once

#include <string>

#include "robot_hardware/transport/backend.hpp"

namespace robot_hardware::transport
{
    using protocol::Command;


    class SerialBackend final : public Backend
    {
    public:
        explicit SerialBackend(BackendConfig config);
        ~SerialBackend() override;

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
    };

} // namespace robot_hardware
