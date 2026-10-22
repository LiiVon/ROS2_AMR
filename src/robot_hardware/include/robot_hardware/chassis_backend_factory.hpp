// chassis_backend_factory.hpp
// 底盘后端工厂函数声明，提供创建不同类型底盘后端
#pragma once

#include <memory>
#include <string>

#include "robot_hardware/chassis_backend.hpp"

namespace robot_hardware
{
    // 工厂函数，用于创建不同类型的底盘后端实例
    std::unique_ptr<ChassisBackend> CreateChassisBackend(
        const std::string &backend, const ChassisBackendConfig &config);

}