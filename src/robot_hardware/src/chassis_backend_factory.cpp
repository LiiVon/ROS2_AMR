#include "robot_hardware/chassis_backend_factory.hpp"
#include "robot_hardware/mock_backend.hpp"
#include "robot_hardware/serial_backend.hpp"
#include "robot_hardware/udp_backend.hpp"

namespace robot_hardware
{
    // 工厂，遇到不同类型的底盘后端时，创建相应的实例
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
        return std::make_unique<MockBackend>();
    }
}