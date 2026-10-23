/**
 * @file docking_behavior_tree.hpp
 * @brief 停靠行为树（行为编排层）。
 * @details 所属包：`robot_tasks`；所在层次：行为编排层。
 * @note 教学说明见 `docs/教学/` 对应章节。
 */

#pragma once

#include <string>

#include "robot_tasks/workflow/docking_workflow.hpp"

namespace robot_tasks {

/**
 * @brief DockingBehaviorTreeInput 数据结构定义
 */
struct DockingBehaviorTreeInput {
  DockingRuntimeState dock_state;
  std::string resource_id = "charger";
};

/**
 * @brief 停靠行为树结果（DockingBehaviorTreeResult）
 */
struct DockingBehaviorTreeResult {
  bool success = false;
  std::string branch;
  DockingRuntimeState dock_state;
  ResourceReservation reservation;
  DockingTransition transition;
};

/**
 * @brief DefaultDockingBehaviorTreeXml。
 * @return 描述性字符串。
 */
std::string DefaultDockingBehaviorTreeXml();
/**
 * @brief 周期推进停靠行为树。
 * @param input 参数
 * @return 计算结果。
 */
DockingBehaviorTreeResult TickDockingBehaviorTree(const DockingBehaviorTreeInput& input);

}  // namespace robot_tasks
