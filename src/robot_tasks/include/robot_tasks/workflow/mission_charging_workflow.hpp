/**
 * @file mission_charging_workflow.hpp
 * @brief 任务充电工作流（业务流程层）。
 * @details 所属包：`robot_tasks`；所在层次：业务流程层。
 * @note 教学说明见 `docs/教学/` 对应章节。
 */

#pragma once

#include <string>

#include "robot_tasks/policy/mission_preflight.hpp"

namespace robot_tasks {

/**
 * @brief OpportunityChargingPlanRequest 数据结构定义
 */
struct OpportunityChargingPlanRequest {
  double requested_battery_voltage = 0.0;
  double current_battery_voltage = 0.0;
  double requested_opportunity_threshold_voltage = 0.0;
  double default_opportunity_threshold_voltage = 0.0;
  double requested_critical_threshold_voltage = 0.0;
  double default_critical_threshold_voltage = 0.0;
  bool requested_require_idle_queue = false;
  bool default_require_idle_queue = false;
  bool mission_active = false;
  bool queue_empty = true;
  int requested_priority = 0;
  int default_priority = 0;
  bool requested_preempt_current = false;
};

/**
 * @brief OpportunityChargingDecision 数据结构定义
 */
struct OpportunityChargingDecision {
  bool success = true;
  bool queue_charging = false;
  bool critical_battery = false;
  bool preempt_current = false;
  bool require_idle_queue = false;
  int priority = 0;
  double evaluated_battery_voltage = 0.0;
  double opportunity_threshold_voltage = 0.0;
  double critical_threshold_voltage = 0.0;
  std::string branch = "skip";
  std::string message;
  std::string queued_message_prefix = "opportunity ";
};

/**
 * @brief LowBatteryDockRequest 数据结构定义
 */
struct LowBatteryDockRequest {
  bool enabled = false;
  double battery_voltage = 0.0;
  double threshold_voltage = 0.0;
  bool request_already_sent = false;
};

/**
 * @brief LowBatteryDockDecision 数据结构定义
 */
struct LowBatteryDockDecision {
  bool low_battery_request_sent = false;
  bool request_dock_return = false;
  std::string reason;
  std::string branch = "disabled";
};

/**
 * @brief 充电请求闸门请求（ChargingRequestGateRequest）
 */
struct ChargingRequestGateRequest {
  std::string mission_id;
  std::string charger_id;
  std::string charger_station_id;
  bool station_catalog_loaded = true;
  std::string station_catalog_error_message;
  bool charger_station_known = true;
  bool mission_already_active_or_queued = false;
  MissionPreflightResult preflight;
};

/**
 * @brief 充电请求闸门判定（ChargingRequestGateDecision）
 */
struct ChargingRequestGateDecision {
  bool accepted = false;
  std::string mission_id;
  std::string charger_id;
  std::string message;
};

/**
 * @brief PlanOpportunityCharging。
 * @param request 输入请求（字段见同名结构体）
 * @return 计算结果。
 */
OpportunityChargingDecision PlanOpportunityCharging(
    const OpportunityChargingPlanRequest& request);

/**
 * @brief PlanLowBatteryDock。
 * @param request 输入请求（字段见同名结构体）
 * @return 计算结果。
 */
LowBatteryDockDecision PlanLowBatteryDock(const LowBatteryDockRequest& request);

/**
 * @brief PlanChargingRequestGate。
 * @param request 输入请求（字段见同名结构体）
 * @return 计算结果。
 */
ChargingRequestGateDecision PlanChargingRequestGate(
    const ChargingRequestGateRequest& request);

}  // namespace robot_tasks
