/**
 * @file payload_workflow.hpp
 * @brief 载荷工作流（业务流程层）。
 * @details 所属包：`robot_tasks`；所在层次：业务流程层。
 * @note 教学说明见 `docs/教学/` 对应章节。
 */

#pragma once

#include <string>

namespace robot_tasks {

/**
 * @brief 载荷工作流状态（PayloadWorkflowState）
 */
struct PayloadWorkflowState {
  bool loaded = false;
  std::string payload_id;
  std::string state = "EMPTY";
  std::string last_action = "none";
  std::string message = "payload empty";
  double weight_kg = 0.0;
  double capacity_kg = 0.0;
  std::string top_module_type = "manual_load";
};

/**
 * @brief 载荷工作流判定（PayloadWorkflowDecision）
 */
struct PayloadWorkflowDecision {
  bool success = false;
  bool mutate_state = false;
  bool loaded = false;
  std::string payload_id;
  std::string state = "EMPTY";
  std::string last_action = "none";
  std::string message;
  double weight_kg = 0.0;
  std::string event_mission_id;
  std::string event_state;
  std::string event_previous_state;
  std::string event_message;
  bool event_recoverable = true;
};

/**
 * @brief PlanConfirmLoad。
 * @param state 输入状态（字段见同名结构体）
 * @param payload_id 对应标识
 * @param station_id 对应标识
 * @param weight_kg 数值参数
 * @return 计算结果。
 */
PayloadWorkflowDecision PlanConfirmLoad(
    const PayloadWorkflowState& state, const std::string& payload_id,
    const std::string& station_id, double weight_kg);
/**
 * @brief PlanConfirmUnload。
 * @param state 输入状态（字段见同名结构体）
 * @param requested_payload_id 对应标识
 * @param station_id 对应标识
 * @return 计算结果。
 */
PayloadWorkflowDecision PlanConfirmUnload(
    const PayloadWorkflowState& state, const std::string& requested_payload_id,
    const std::string& station_id);
/**
 * @brief PlanSetPayloadLoaded。
 * @param state 输入状态（字段见同名结构体）
 * @param loaded 布尔标志
 * @param payload_id 对应标识
 * @param weight_kg 数值参数
 * @param message 字符串
 * @return 计算结果。
 */
PayloadWorkflowDecision PlanSetPayloadLoaded(
    const PayloadWorkflowState& state, bool loaded, const std::string& payload_id,
    double weight_kg, const std::string& message);
/**
 * @brief PlanTopModuleAction。
 * @param state 输入状态（字段见同名结构体）
 * @param action 字符串
 * @return 计算结果。
 */
PayloadWorkflowDecision PlanTopModuleAction(
    const PayloadWorkflowState& state, const std::string& action);

/**
 * @brief TopModuleActionSupported。
 * @param top_module_type 字符串
 * @param action 字符串
 * @return 满足条件返回 true，否则返回 false。
 */
bool TopModuleActionSupported(
    const std::string& top_module_type, const std::string& action);

}  // namespace robot_tasks
