/**
 * @file mission_runtime_behavior_tree.hpp
 * @brief 任务运行时行为树（行为编排层）。
 * @details 所属包：`robot_tasks`；所在层次：行为编排层。
 * @note 教学说明见 `docs/教学/` 对应章节。
 */

#pragma once

#include <string>

#include "robot_tasks/workflow/mission_runtime_workflow.hpp"

namespace robot_tasks {

/**
 * @brief MissionRuntimeBehaviorTreeInput 数据结构定义
 */
struct MissionRuntimeBehaviorTreeInput {
  MissionRuntimeTickInput tick;
};

/**
 * @brief 任务运行时行为树结果（MissionRuntimeBehaviorTreeResult）
 */
struct MissionRuntimeBehaviorTreeResult {
  bool success = false;
  std::string branch;
  MissionRuntimeTickDecision decision;
};

/**
 * @brief DefaultMissionRuntimeBehaviorTreeXml。
 * @return 描述性字符串。
 */
std::string DefaultMissionRuntimeBehaviorTreeXml();
/**
 * @brief 周期推进任务运行时行为树。
 * @param input 参数
 * @return 计算结果。
 */
MissionRuntimeBehaviorTreeResult TickMissionRuntimeBehaviorTree(
    const MissionRuntimeBehaviorTreeInput& input);

}  // namespace robot_tasks
