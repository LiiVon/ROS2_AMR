/**
 * @file mission_result_behavior_tree.hpp
 * @brief 任务结果行为树（行为编排层）。
 * @details 所属包：`robot_tasks`；所在层次：行为编排层。
 * @note 教学说明见 `docs/教学/` 对应章节。
 */

#pragma once

#include <string>

#include "robot_tasks/workflow/mission_result_workflow.hpp"

namespace robot_tasks {

/**
 * @brief 任务结果行为树结果（MissionResultBehaviorTreeResult）
 */
struct MissionResultBehaviorTreeResult {
  bool success = false;
  std::string branch;
  MissionResultWorkflowDecision decision;
};

/**
 * @brief DefaultMissionResultBehaviorTreeXml。
 * @return 描述性字符串。
 */
std::string DefaultMissionResultBehaviorTreeXml();
/**
 * @brief 周期推进任务结果行为树。
 * @param input 参数
 * @return 计算结果。
 */
MissionResultBehaviorTreeResult TickMissionResultBehaviorTree(
    const MissionResultWorkflowInput& input);

}  // namespace robot_tasks
