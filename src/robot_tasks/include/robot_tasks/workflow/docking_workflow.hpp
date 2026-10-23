/**
 * @file docking_workflow.hpp
 * @brief 停靠工作流（业务流程层）。
 * @details 所属包：`robot_tasks`；所在层次：业务流程层。
 * @note 教学说明见 `docs/教学/` 对应章节。
 */

#pragma once

#include <string>
#include <unordered_map>

#include "robot_tasks/model/facility_reservation.hpp"
#include "robot_tasks/policy/mission_preflight.hpp"

namespace robot_tasks {

/**
 * @brief 停靠运行时状态（DockingRuntimeState）
 */
struct DockingRuntimeState {
  std::string dock_id;
  std::string mission_id;
  std::string state = "AVAILABLE";
  bool simulate_contact_success = true;
  int contact_attempts = 0;
};

using DockingStateMap = std::unordered_map<std::string, DockingRuntimeState>;
using DockingMissionMap = std::unordered_map<std::string, std::string>;

/**
 * @brief DockingTransition 数据结构定义
 */
struct DockingTransition {
  bool changed = false;
  std::string mission_id;
  std::string dock_id;
  std::string previous_state;
  std::string state;
  std::string event_state;
  std::string event_message;
  bool event_recoverable = true;
};

/**
 * @brief DockReturnRequest 数据结构定义
 */
struct DockReturnRequest {
  bool dock_profile_available = false;
  std::string load_error_message;
  bool dock_already_requested_or_running = false;
  bool mission_active = false;
};

/**
 * @brief DockReturnDecision 数据结构定义
 */
struct DockReturnDecision {
  bool response_success = false;
  std::string response_message;
  bool set_error_state = false;
  std::string error_message;
  bool queue_dock_mission = false;
  bool cancel_active_mission = false;
  bool dispatch_dock_mission = false;
  std::string branch = "load_failed";
};

/**
 * @brief UndockingRequest 数据结构定义
 */
struct UndockingRequest {
  std::string dock_id;
  std::string charger_resource_id;
  std::string request_id;
  bool release_charger = false;
};

/**
 * @brief UndockingDecision 数据结构定义
 */
struct UndockingDecision {
  bool success = true;
  bool mutate_state = true;
  std::string mission_id;
  std::string dock_id;
  std::string previous_state;
  std::string state = "UNDOCKED";
  std::string message;
  bool release_charger = false;
  std::string release_mission_id;
};

/**
 * @brief 停靠请求闸门请求（DockingRequestGateRequest）
 */
struct DockingRequestGateRequest {
  std::string dock_id;
  bool dock_enabled = true;
  bool dock_available = true;
  std::string dock_state = "AVAILABLE";
  bool dock_references_valid = true;
  std::string dock_reference_message;
  std::string mission_id;
  bool mission_already_active_or_queued = false;
  MissionPreflightResult preflight;
};

/**
 * @brief 停靠请求闸门判定（DockingRequestGateDecision）
 */
struct DockingRequestGateDecision {
  bool accepted = false;
  std::string mission_id;
  std::string state;
  std::string message;
};

/**
 * @brief CompleteDockingSubflow。
 * @param dock_states 参数
 * @param dock_by_mission 参数
 * @param reservations 参数
 * @param resource_by_mission 参数
 * @param mission_id 对应标识
 * @return 计算结果。
 */
DockingTransition CompleteDockingSubflow(
    DockingStateMap& dock_states, const DockingMissionMap& dock_by_mission,
    FacilityReservationMap& reservations, const MissionResourceMap& resource_by_mission,
    const std::string& mission_id);

/**
 * @brief StageDockingContactRetry。
 * @param dock_states 参数
 * @param dock_by_mission 参数
 * @param reservations 参数
 * @param resource_by_mission 参数
 * @param mission_id 对应标识
 * @return 计算结果。
 */
DockingTransition StageDockingContactRetry(
    DockingStateMap& dock_states, const DockingMissionMap& dock_by_mission,
    FacilityReservationMap& reservations, const MissionResourceMap& resource_by_mission,
    const std::string& mission_id);

/**
 * @brief MarkDockingSubflowTerminal。
 * @param dock_states 参数
 * @param dock_by_mission 参数
 * @param mission_id 对应标识
 * @param state 输入状态（字段见同名结构体）
 * @param message 字符串
 * @return 计算结果。
 */
DockingTransition MarkDockingSubflowTerminal(
    DockingStateMap& dock_states, DockingMissionMap& dock_by_mission,
    const std::string& mission_id, const std::string& state, const std::string& message);

/**
 * @brief PlanDockReturn。
 * @param request 输入请求（字段见同名结构体）
 * @return 计算结果。
 */
DockReturnDecision PlanDockReturn(const DockReturnRequest& request);

/**
 * @brief PlanDockingRequestGate。
 * @param request 输入请求（字段见同名结构体）
 * @return 计算结果。
 */
DockingRequestGateDecision PlanDockingRequestGate(
    const DockingRequestGateRequest& request);

/**
 * @brief PlanUndocking。
 * @param request 输入请求（字段见同名结构体）
 * @param dock_states 参数
 * @param reservations 参数
 * @return 计算结果。
 */
UndockingDecision PlanUndocking(
    const UndockingRequest& request, const DockingStateMap& dock_states,
    const FacilityReservationMap& reservations);

/**
 * @brief 应用 ApplyUndockingDecision。
 * @param decision 判定结果（字段见同名结构体）
 * @param charger_resource_id 对应标识
 * @param dock_states 参数
 * @param reservations 参数
 * @param resource_by_mission 参数
 */
void ApplyUndockingDecision(
    const UndockingDecision& decision, const std::string& charger_resource_id,
    DockingStateMap& dock_states, FacilityReservationMap& reservations,
    MissionResourceMap& resource_by_mission);

}  // namespace robot_tasks
