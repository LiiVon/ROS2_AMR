/**
 * @file serial_backend.hpp
 * @brief 串口后端（传输层）。
 * @details 所属包：`robot_hardware`；所在层次：传输层。
 * @note 教学说明见 `docs/教学/` 对应章节。
 */

#pragma once

#include <string>

#include "robot_hardware/transport/backend.hpp"

namespace robot_hardware::transport
{
    using protocol::Command;


    /**
     * @brief 串口后端（SerialBackend）
     */
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
