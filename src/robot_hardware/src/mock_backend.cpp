#include "robot_hardware/mock_backend.hpp"

namespace robot_hardware
{
    std::string MockBackend::Name() const
    {
        return "mock";
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

    bool MockBackend::WriteCommand(const ChassisCommand &command, std::string *error)
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

    ChassisCommand MockBackend::LastCommand() const
    {
        return last_command_;
    }
}