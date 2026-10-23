/**
 * @file facility_behavior_tree.hpp
 * @brief 设施行为树（行为编排层）。
 * @details 所属包：`robot_tasks`；所在层次：行为编排层。
 * @note 教学说明见 `docs/教学/` 对应章节。
 */

#pragma once

#include <string>

#include "robot_tasks/workflow/facility_workflow.hpp"

namespace robot_tasks {

/**
 * @brief FacilityActionBehaviorTreeInput 数据结构定义
 */
struct FacilityActionBehaviorTreeInput {
  FacilityCatalog catalog;
  FacilityReservationMap reservations;
  MissionResourceMap resource_by_mission;
  std::string request_id;
  std::string resource_id;
  std::string resource_type;
  std::string action = "reserve";
  bool hold_after_action = true;
};

/**
 * @brief FacilityActionBehaviorTreeResult 数据结构定义
 */
struct FacilityActionBehaviorTreeResult {
  bool success = false;
  std::string branch;
  std::string message;
  std::string mission_id;
  std::string resource_id;
  FacilityReservationMap reservations;
  MissionResourceMap resource_by_mission;
};

/**
 * @brief LiftSessionBehaviorTreeInput 数据结构定义
 */
struct LiftSessionBehaviorTreeInput {
  FacilityReservationMap reservations;
  FacilityResource lift;
  std::string session_id;
  std::string requester_id;
  bool release_existing_for_holder = true;
  bool release = false;
};

/**
 * @brief LiftSessionBehaviorTreeResult 数据结构定义
 */
struct LiftSessionBehaviorTreeResult {
  bool success = false;
  std::string branch;
  LiftSessionResult lift_result;
  FacilityReservationMap reservations;
};

/**
 * @brief DefaultFacilityActionBehaviorTreeXml。
 * @return 描述性字符串。
 */
std::string DefaultFacilityActionBehaviorTreeXml();
/**
 * @brief 周期推进 TickFacilityActionBehaviorTree。
 * @param input 参数
 * @return 计算结果。
 */
FacilityActionBehaviorTreeResult TickFacilityActionBehaviorTree(
    const FacilityActionBehaviorTreeInput& input);

/**
 * @brief DefaultLiftSessionBehaviorTreeXml。
 * @return 描述性字符串。
 */
std::string DefaultLiftSessionBehaviorTreeXml();
/**
 * @brief 周期推进 TickLiftSessionBehaviorTree。
 * @param input 参数
 * @return 计算结果。
 */
LiftSessionBehaviorTreeResult TickLiftSessionBehaviorTree(
    const LiftSessionBehaviorTreeInput& input);

}  // namespace robot_tasks
