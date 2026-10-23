/**
 * @file mission_resource_cleanup_behavior_tree.hpp
 * @brief 任务资源清理行为树（行为编排层）。
 * @details 所属包：`robot_tasks`；所在层次：行为编排层。
 * @note 教学说明见 `docs/教学/` 对应章节。
 */

#pragma once

#include <string>

#include "robot_tasks/workflow/mission_resource_cleanup.hpp"

namespace robot_tasks {

/**
 * @brief MissionResourceCleanupBehaviorTreeInput 数据结构定义
 */
struct MissionResourceCleanupBehaviorTreeInput {
  FacilityReservationMap& facility_reservations;
  MissionResourceMap& facility_resource_by_mission;
  RouteLockMap& route_locks;
  RouteLockReservationMap& route_locks_by_mission;
  std::string mission_id;
};

/**
 * @brief 任务资源清理行为树结果（MissionResourceCleanupBehaviorTreeResult）
 */
struct MissionResourceCleanupBehaviorTreeResult {
  bool success = false;
  std::string branch;
  MissionResourceCleanupResult cleanup;
};

/**
 * @brief DefaultMissionResourceCleanupBehaviorTreeXml。
 * @return 描述性字符串。
 */
std::string DefaultMissionResourceCleanupBehaviorTreeXml();
/**
 * @brief 周期推进任务资源清理行为树。
 * @param input 参数
 * @return 计算结果。
 */
MissionResourceCleanupBehaviorTreeResult TickMissionResourceCleanupBehaviorTree(
    MissionResourceCleanupBehaviorTreeInput input);

}  // namespace robot_tasks
