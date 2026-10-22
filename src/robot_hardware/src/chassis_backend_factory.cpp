// chassis_backend_factory.cpp
// 底盘后端工厂的具体实现。

#include "robot_hardware/chassis_backend_factory.hpp"

#include <stdexcept>

#include "robot_hardware/mock_backend.hpp"
#include "robot_hardware/serial_backend.hpp"
#include "robot_hardware/udp_backend.hpp"

namespace robot_hardware
{

    std::unique_ptr<ChassisBackend> CreateChassisBackend(
        const std::string &backend,
        const ChassisBackendConfig &config)
    {
        // 1. 显式创建模拟底盘后端。
        if (backend == "mock")
        {
            return std::make_unique<MockBackend>();
        }

        // 2. 创建串口通信后端。
        if (backend == "serial")
        {
            return std::make_unique<SerialBackend>(config);
        }

        // 3. 创建 UDP 通信后端。
        if (backend == "udp")
        {
            return std::make_unique<UdpBackend>(config);
        }

        // 4. 未知类型直接报错，不允许静默回退到 Mock。
        throw std::invalid_argument(
            "Unsupported chassis backend: '" + backend +
            "'. Supported backends: mock, serial, udp.");
    }

} // namespace robot_hardware