/**
 * @file mock_backend.hpp
 * @brief 模拟后端（传输层）。
 * @details 所属包：`robot_hardware`；所在层次：传输层。
 * @note 教学说明见 `docs/教学/` 对应章节。
 */

#pragma once

#include "robot_hardware/transport/backend.hpp"

namespace robot_hardware::transport
{
    using protocol::Command;

    /**
     * @brief 模拟后端（MockBackend）
     */
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