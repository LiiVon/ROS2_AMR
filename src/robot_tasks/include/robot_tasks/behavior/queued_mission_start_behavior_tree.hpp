/**
 * @file queued_mission_start_behavior_tree.hpp
 * @brief 已排队任务启动行为树（行为编排层）。
 * @details 所属包：`robot_tasks`；所在层次：行为编排层。
 * @note 教学说明见 `docs/教学/` 对应章节。
 */

#pragma once

#include <string>

#include "robot_tasks/model/mission_queue.hpp"

namespace robot_tasks {

/**
 * @brief QueuedMissionStartBehaviorTreeInput 数据结构定义
 */
struct QueuedMissionStartBehaviorTreeInput {
  QueuedMissionStartRequest request;
};

/**
 * @brief 已排队任务启动行为树结果（QueuedMissionStartBehaviorTreeResult）
 */
struct QueuedMissionStartBehaviorTreeResult {
  bool success = false;
  std::string branch;
  QueuedMissionStartDecision decision;
};

/**
 * @brief DefaultQueuedMissionStartBehaviorTreeXml。
 * @return 描述性字符串。
 */
std::string DefaultQueuedMissionStartBehaviorTreeXml();
/**
 * @brief 周期推进已排队任务启动行为树。
 * @param input 参数
 * @return 计算结果。
 */
QueuedMissionStartBehaviorTreeResult TickQueuedMissionStartBehaviorTree(
    const QueuedMissionStartBehaviorTreeInput& input);

}  // namespace robot_tasks
