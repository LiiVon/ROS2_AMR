/**
 * @file mission_recovery_policy.hpp
 * @brief 任务恢复策略（决策策略层）。
 * @details 所属包：`robot_tasks`；所在层次：决策策略层。
 * @note 教学说明见 `docs/教学/` 对应章节。
 */

#pragma once

#include <string>

namespace robot_tasks {

/**
 * @brief 任务恢复配置（MissionRecoveryConfig）
 */
struct MissionRecoveryConfig {
  bool enabled = true;
  std::string policy = "retry_then_dock";
  int retry_limit = 1;
};

/**
 * @brief 任务恢复判定（MissionRecoveryDecision）
 */
struct MissionRecoveryDecision {
  std::string action = "none";
  std::string message;
  bool recoverable = true;
};

/**
 * @brief DecideMissionRecovery。
 * @param config 配置参数（字段见同名结构体）
 * @param retry_count 数量
 * @param is_dock_mission 布尔判定标志
 * @param mission_id 对应标识
 * @return 计算结果。
 */
MissionRecoveryDecision DecideMissionRecovery(
    const MissionRecoveryConfig& config, int retry_count, bool is_dock_mission,
    const std::string& mission_id);

}  // namespace robot_tasks
