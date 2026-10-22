#pragma once

#include "robot_hardware/transport/backend.hpp"

namespace robot_hardware::transport
{
    using protocol::Command;

    class MockBackend final : public Backend
    {
    public:
        MockBackend() = default;
        std::string Name() const override;
        BackendKind Kind() const override;
        bool Open(std::string *error) override;
        void Close() override;
        bool IsOpen() const override;
        bool WriteCommand(const Command &command, std::string *error) override;
        std::optional<std::string> Read(std::string *error) override;

        Command LastCommand() const;
    private:
        bool open_ = false;
        Command last_command_;
    };
}