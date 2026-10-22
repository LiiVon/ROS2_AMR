#include "robot_hardware/transport/mock_backend.hpp"

namespace robot_hardware::transport
{
    using protocol::Command;

    std::string MockBackend::Name() const
    {
        return "mock";
    }

    BackendKind MockBackend::Kind() const
    {
        return BackendKind::kMock;
    }

    bool MockBackend::Open(std::string *error)
    {
        open_ = true;
        if(error)
        {
            *error = "";
        }
        return true;
    }

    void MockBackend::Close()
    {
        open_ = false;
    }

    bool MockBackend::IsOpen() const
    {
        return open_;
    }

    bool MockBackend::WriteCommand(const Command &command, std::string *error)
    {
        last_command_ = command;
        if (!open_)
        {
            if (error)
            {
                *error = "MockBackend is not open.";
            }
            return false;
        }
        return open_;
    }

    std::optional<std::string> MockBackend::Read(std::string *error)
    {
        if (!open_)
        {
            if (error)
            {
                *error = "MockBackend is not open.";
            }
            return std::nullopt;
        }
        return std::optional<std::string>("Mock data");
    }

    Command MockBackend::LastCommand() const
    {
        return last_command_;
    }
}