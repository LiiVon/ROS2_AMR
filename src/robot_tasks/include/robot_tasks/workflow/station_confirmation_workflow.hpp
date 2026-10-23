/**
 * @file station_confirmation_workflow.hpp
 * @brief 站点确认工作流（业务流程层）。
 * @details 所属包：`robot_tasks`；所在层次：业务流程层。
 * @note 教学说明见 `docs/教学/` 对应章节。
 */

#pragma once

#include <string>
#include <unordered_map>
#include <vector>

namespace robot_tasks {

/**
 * @brief 站点确认运行时状态（StationConfirmationRuntimeState）
 */
struct StationConfirmationRuntimeState {
  std::string confirmation_id;
  std::string station_id;
  std::string action;
  std::string mission_id;
  std::string operator_hint;
  std::string state = "PENDING";
  int timeout_sec = 0;
  double created_at_sec = 0.0;
  std::string timeout_policy = "manual";
};

using StationConfirmationStateMap =
    std::unordered_map<std::string, StationConfirmationRuntimeState>;

/**
 * @brief 站点确认请求（StationConfirmationRequest）
 */
struct StationConfirmationRequest {
  std::string confirmation_id;
  std::string station_id;
  std::string action;
  std::string mission_id;
  std::string operator_hint;
  int timeout_sec = 0;
  std::string timeout_policy = "manual";
  double created_at_sec = 0.0;
  bool station_known = false;
};

/**
 * @brief 站点确认请求判定（StationConfirmationRequestDecision）
 */
struct StationConfirmationRequestDecision {
  bool success = false;
  bool mutate_state = false;
  StationConfirmationRuntimeState confirmation;
  std::string confirmation_id;
  std::string state = "REJECTED";
  std::string message;
  std::string event_mission_id;
  std::string event_state;
  std::string event_previous_state = "READY";
  std::string event_message;
  bool event_recoverable = true;
};

/**
 * @brief StationConfirmationResolveRequest 数据结构定义
 */
struct StationConfirmationResolveRequest {
  std::string confirmation_id;
  std::string state;
  std::string operator_id;
  std::string note;
};

/**
 * @brief StationConfirmationResolveDecision 数据结构定义
 */
struct StationConfirmationResolveDecision {
  bool success = false;
  bool mutate_state = false;
  std::string confirmation_id;
  std::string state = "UNKNOWN";
  std::string message;
  std::string event_mission_id;
  std::string event_state;
  std::string event_previous_state = "PENDING";
  std::string event_message;
  bool event_recoverable = true;
};

/**
 * @brief StationConfirmationTimeoutDecision 数据结构定义
 */
struct StationConfirmationTimeoutDecision {
  bool timed_out = false;
  bool update_created_at = false;
  double new_created_at_sec = 0.0;
  bool update_state = false;
  std::string state;
  std::string event_mission_id;
  std::string event_state;
  std::string event_previous_state = "PENDING";
  std::string event_message;
  bool event_recoverable = true;
  bool request_dock_return = false;
  bool set_runner_state = false;
  std::string runner_state;
  std::string runner_message;
  bool runner_recoverable = true;
};

/**
 * @brief PendingStationConfirmationsProjection 数据结构定义
 */
struct PendingStationConfirmationsProjection {
  bool success = true;
  std::vector<std::string> confirmation_ids;
  std::vector<std::string> station_ids;
  std::vector<std::string> actions;
  std::vector<std::string> mission_ids;
  std::vector<std::string> operator_hints;
  std::vector<std::string> states;
  std::string message;
};

/**
 * @brief 归一化 NormalizeStationConfirmationTimeoutPolicy。
 * @param policy 字符串
 * @return 描述性字符串。
 */
std::string NormalizeStationConfirmationTimeoutPolicy(const std::string& policy);

/**
 * @brief PlanStationConfirmationRequest。
 * @param request 输入请求（字段见同名结构体）
 * @param confirmations 参数
 * @return 计算结果。
 */
StationConfirmationRequestDecision PlanStationConfirmationRequest(
    const StationConfirmationRequest& request, const StationConfirmationStateMap& confirmations);

/**
 * @brief PlanStationConfirmationResolve。
 * @param request 输入请求（字段见同名结构体）
 * @param confirmations 参数
 * @return 计算结果。
 */
StationConfirmationResolveDecision PlanStationConfirmationResolve(
    const StationConfirmationResolveRequest& request,
    const StationConfirmationStateMap& confirmations);

/**
 * @brief PlanStationConfirmationTimeout。
 * @param confirmation 参数
 * @param current_time_sec 数值参数
 * @return 计算结果。
 */
StationConfirmationTimeoutDecision PlanStationConfirmationTimeout(
    const StationConfirmationRuntimeState& confirmation, double current_time_sec);

/**
 * @brief 构建 BuildPendingStationConfirmationsProjection。
 * @param confirmations 参数
 * @param station_filter 字符串
 * @return 计算结果。
 */
PendingStationConfirmationsProjection BuildPendingStationConfirmationsProjection(
    const StationConfirmationStateMap& confirmations, const std::string& station_filter);

}  // namespace robot_tasks
