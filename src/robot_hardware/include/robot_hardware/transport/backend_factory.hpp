// chassis_backend_factory.hpp
// 底盘后端工厂：根据指定类型创建对应的底盘通信后端。

#pragma once

#include <memory>
#include <string>

#include "robot_hardware/transport/backend.hpp"

namespace robot_hardware::transport
{


    // 创建底盘通信后端。
    // 支持的类型：mock、serial、udp。
    // 不支持的类型会抛出 std::invalid_argument。
    std::unique_ptr<Backend> CreateBackend(
        const std::string &backend,
        const BackendConfig &config);

} // namespace robot_hardware