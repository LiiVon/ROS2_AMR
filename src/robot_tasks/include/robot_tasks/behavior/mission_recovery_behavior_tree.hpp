/**
 * @file mission_recovery_behavior_tree.hpp
 * @brief 任务恢复行为树（行为编排层）。
 * @details 所属包：`robot_tasks`；所在层次：行为编排层。
 * @note 教学说明见 `docs/教学/` 对应章节。
 */

#pragma once

#include <cstdint>
#include <optional>
#include <string>

#include "robot_tasks/model/mission_queue.hpp"
#include "robot_tasks/policy/mission_recovery_policy.hpp"

namespace robot_tasks {

/**
 * @brief MissionRecoveryBehaviorTreeInput 数据结构定义
 */
struct MissionRecoveryBehaviorTreeInput {
  MissionRecoveryConfig config;
  int retry_count = 0;
  bool is_dock_mission = false;
  std::string mission_id;
  std::string failure_message;
  std::optional<MissionProfile> active_profile;
  std::string mission_file;
  int recovery_priority = 0;
  std::uint64_t next_queue_sequence = 0;
};

/**
 * @brief 任务恢复行为树结果（MissionRecoveryBehaviorTreeResult）
 */
struct MissionRecoveryBehaviorTreeResult {
  bool success = false;
  std::string branch;
  MissionRecoveryDecision decision;
  std::string state;
  std::string state_message;
  bool recoverable = false;
  int retry_count = 0;
  std::uint64_t next_queue_sequence = 0;
  bool queue_running = false;
  bool queue_start_requested = false;
  bool request_dock_return = false;
  bool stop_queue = false;
  std::optional<QueuedMission> retry_mission;
};

/**
 * @brief DefaultMissionRecoveryBehaviorTreeXml。
 * @return 描述性字符串。
 */
std::string DefaultMissionRecoveryBehaviorTreeXml();
/**
 * @brief 周期推进任务恢复行为树。
 * @param input 参数
 * @return 计算结果。
 */
MissionRecoveryBehaviorTreeResult TickMissionRecoveryBehaviorTree(
    const MissionRecoveryBehaviorTreeInput& input);

}  // namespace robot_tasks
