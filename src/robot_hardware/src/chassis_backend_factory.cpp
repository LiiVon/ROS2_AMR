#include "robot_hardware/chassis_backend_factory.hpp"
#include "robot_hardware/mock_backend.hpp"
#include "robot_hardware/serial_backend.hpp"
#include "robot_hardware/udp_backend.hpp"

namespace robot_hardware
{
    std::unique_ptr<ChassisBackend> CreateChassisBackend(
        const std::string &backend, const ChassisBackendConfig &config)
    {
        if (backend == "serial")
        {
            return std::make_unique<SerialBackend>(config);
        }
        else if (backend == "udp")
        {
            return std::make_unique<UdpBackend>(config);
        }
        else if (backend == "mock")
        {
            return std::make_unique<MockBackend>(config);
        }
        else
        {
            throw std::invalid_argument("Unknown backend type: " + backend);
        }
    }
}