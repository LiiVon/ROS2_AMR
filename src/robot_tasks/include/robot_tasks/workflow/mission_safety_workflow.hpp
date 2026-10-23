/**
 * @file mission_safety_workflow.hpp
 * @brief 任务安全工作流（业务流程层）。
 * @details 所属包：`robot_tasks`；所在层次：业务流程层。
 * @note 教学说明见 `docs/教学/` 对应章节。
 */

#pragma once

#include <string>

namespace robot_tasks {

/**
 * @brief 任务安全状态（MissionSafetyState）
 */
struct MissionSafetyState {
  bool obstacle_blocked = false;
  bool blockage_paused_mission = false;
  double runtime_speed_limit_mps = 0.0;
  double zone_speed_limit_mps = 0.6;
  std::string stop_reason;
  std::string blockage_id;
  std::string mission_id;
  std::string message = "safety ok";
};

/**
 * @brief 任务安全事件（MissionSafetyEvent）
 */
struct MissionSafetyEvent {
  bool emit = false;
  std::string mission_id;
  std::string state;
  std::string previous_state;
  std::string message;
  bool recoverable = true;
};

/**
 * @brief MissionSafetySideEffects 数据结构定义
 */
struct MissionSafetySideEffects {
  bool request_pause = false;
  bool request_resume = false;
  bool set_runner_state = false;
  std::string runner_state;
  std::string runner_message;
  bool runner_recoverable = true;
};

/**
 * @brief 任务安全判定（MissionSafetyDecision）
 */
struct MissionSafetyDecision {
  bool success = false;
  bool mutate_state = false;
  MissionSafetyState safety;
  MissionSafetyEvent event;
  MissionSafetySideEffects side_effects;
  std::string response_message;
  std::string response_state;
  std::string response_blockage_id;
  double response_runtime_speed_limit_mps = 0.0;
  double response_effective_speed_limit_mps = 0.0;
};

/**
 * @brief 任务安全速度限值请求（MissionSafetySpeedLimitRequest）
 */
struct MissionSafetySpeedLimitRequest {
  MissionSafetyState current;
  double speed_limit_mps = 0.0;
  std::string reason;
};

/**
 * @brief MissionSafetyReportBlockageRequest 数据结构定义
 */
struct MissionSafetyReportBlockageRequest {
  MissionSafetyState current;
  std::string blockage_id;
  std::string reason;
  std::string active_mission_id;
  std::string runner_previous_state = "UNKNOWN";
  bool pause_active = false;
  bool pause_on_blockage = true;
  bool mission_active = false;
  bool runner_paused = false;
};

/**
 * @brief MissionSafetyClearBlockageRequest 数据结构定义
 */
struct MissionSafetyClearBlockageRequest {
  MissionSafetyState current;
  std::string blockage_id;
  std::string resolution;
  std::string runner_state;
  bool resume_active = false;
  bool mission_active = false;
  bool runner_paused = false;
};

double EffectiveSafetySpeedLimit(const MissionSafetyState& state);
/**
 * @brief MissionSafetyStateName。
 * @param state 输入状态（字段见同名结构体）
 * @return 描述性字符串。
 */
std::string MissionSafetyStateName(const MissionSafetyState& state);

/**
 * @brief PlanSetDynamicSpeedLimit。
 * @param request 输入请求（字段见同名结构体）
 * @return 计算结果。
 */
MissionSafetyDecision PlanSetDynamicSpeedLimit(
    const MissionSafetySpeedLimitRequest& request);
/**
 * @brief PlanReportObstacleBlockage。
 * @param request 输入请求（字段见同名结构体）
 * @return 计算结果。
 */
MissionSafetyDecision PlanReportObstacleBlockage(
    const MissionSafetyReportBlockageRequest& request);
/**
 * @brief PlanClearObstacleBlockage。
 * @param request 输入请求（字段见同名结构体）
 * @return 计算结果。
 */
MissionSafetyDecision PlanClearObstacleBlockage(
    const MissionSafetyClearBlockageRequest& request);

}  // namespace robot_tasks
