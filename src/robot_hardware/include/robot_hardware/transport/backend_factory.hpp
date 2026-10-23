/**
 * @file backend_factory.hpp
 * @brief 后端工厂（传输层）。
 * @details chassis_backend_factory.hpp 底盘后端工厂：根据指定类型创建对应的底盘通信后端。
 * @details 所属包：`robot_hardware`；所在层次：传输层。
 * @note 教学说明见 `docs/教学/` 对应章节。
 */

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