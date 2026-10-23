/**
 * @file mission_queue_behavior_tree.hpp
 * @brief 任务队列行为树（行为编排层）。
 * @details 所属包：`robot_tasks`；所在层次：行为编排层。
 * @note 教学说明见 `docs/教学/` 对应章节。
 */

#pragma once

#include <string>
#include <unordered_set>
#include <vector>

#include "robot_tasks/model/mission_queue.hpp"

namespace robot_tasks {

/**
 * @brief MissionQueueBehaviorTreeInput 数据结构定义
 */
struct MissionQueueBehaviorTreeInput {
  std::vector<QueuedMission> queue;
  std::unordered_set<std::string> paused_order_ids;
};

/**
 * @brief 任务队列行为树结果（MissionQueueBehaviorTreeResult）
 */
struct MissionQueueBehaviorTreeResult {
  bool success = false;
  std::string branch;
  MissionQueueDispatchResult dispatch;
};

/**
 * @brief DefaultMissionQueueBehaviorTreeXml。
 * @return 描述性字符串。
 */
std::string DefaultMissionQueueBehaviorTreeXml();
/**
 * @brief 周期推进任务队列行为树。
 * @param input 参数
 * @return 计算结果。
 */
MissionQueueBehaviorTreeResult TickMissionQueueBehaviorTree(
    const MissionQueueBehaviorTreeInput& input);

}  // namespace robot_tasks
