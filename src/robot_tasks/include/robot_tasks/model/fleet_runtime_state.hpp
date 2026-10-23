/**
 * @file fleet_runtime_state.hpp
 * @brief 车队运行时状态（数据模型层）。
 * @details 所属包：`robot_tasks`；所在层次：数据模型层。
 * @note 教学说明见 `docs/教学/` 对应章节。
 */

#pragma once

#include <string>
#include <unordered_map>
#include <vector>

#include "robot_tasks/model/fleet_catalog.hpp"
#include "robot_tasks/model/station_catalog.hpp"

namespace robot_tasks {

/**
 * @brief RuntimeFleetRobotState 数据结构定义
 */
struct RuntimeFleetRobotState {
  bool update_enabled = false;
  bool enabled = true;
  bool update_state = false;
  std::string state;
  bool update_battery_voltage = false;
  double battery_voltage = 0.0;
  bool update_current_station_id = false;
  std::string current_station_id;
};

/**
 * @brief FleetRuntimeUpdateRequest 数据结构定义
 */
struct FleetRuntimeUpdateRequest {
  std::string robot_id;
  bool enabled = true;
  std::string state;
  double battery_voltage = 0.0;
  std::string current_station_id;
  bool update_enabled = false;
  bool update_state = false;
  bool update_battery_voltage = false;
  bool update_current_station_id = false;
};

/**
 * @brief FleetRuntimeUpdateDecision 数据结构定义
 */
struct FleetRuntimeUpdateDecision {
  bool success = false;
  bool mutate_state = false;
  RuntimeFleetRobotState runtime_state;
  FleetRobot effective_robot;
  std::string message;
};

/**
 * @brief FleetRobotListRequest 数据结构定义
 */
struct FleetRobotListRequest {
  bool include_disabled = false;
  std::string capability;
};

/**
 * @brief FleetRobotListProjection 数据结构定义
 */
struct FleetRobotListProjection {
  bool success = true;
  std::vector<std::string> robot_ids;
  std::vector<bool> enabled;
  std::vector<std::string> states;
  std::vector<double> battery_voltage;
  std::vector<std::string> current_station_ids;
  std::vector<std::string> capabilities;
  std::string message;
};

/**
 * @brief RemoteFleetTaskState 数据结构定义
 */
struct RemoteFleetTaskState {
  std::string task_id;
  std::string assigned_robot_id;
  std::string pickup_station;
  std::string dropoff_station;
  std::string state = "DISPATCHED_REMOTE";
  std::string message;
};

using RemoteFleetTaskMap = std::unordered_map<std::string, RemoteFleetTaskState>;

/**
 * @brief RemoteFleetTaskUpdateRequest 数据结构定义
 */
struct RemoteFleetTaskUpdateRequest {
  std::string task_id;
  std::string state;
  std::string message;
};

/**
 * @brief RemoteFleetTaskUpdateDecision 数据结构定义
 */
struct RemoteFleetTaskUpdateDecision {
  bool success = false;
  bool mutate_state = false;
  RemoteFleetTaskState task;
  std::string previous_state;
  std::string response_message;
  std::string event_mission_id;
  std::string event_state;
  std::string event_previous_state;
  std::string event_message;
  bool event_recoverable = true;
};

/**
 * @brief FleetStationTaskDispatchRequest 数据结构定义
 */
struct FleetStationTaskDispatchRequest {
  std::string task_id;
  std::string assigned_robot_id;
  std::string local_robot_id;
  std::string pickup_station;
  std::string dropoff_station;
  std::string previous_mission_state;
  double estimated_total_distance_m = 0.0;
};

/**
 * @brief FleetStationTaskDispatchDecision 数据结构定义
 */
struct FleetStationTaskDispatchDecision {
  bool success = false;
  bool dispatch_remote = false;
  bool mutate_remote_tasks = false;
  RemoteFleetTaskState remote_task;
  std::string station_order_id;
  std::string mission_id;
  std::string response_message;
  std::string event_mission_id;
  std::string event_state;
  std::string event_previous_state;
  std::string event_message;
  bool event_recoverable = true;
};

/**
 * @brief 应用运行时车队状态。
 * @param catalog 参数
 * @param runtime_state 字符串
 */
void ApplyRuntimeFleetState(
    FleetCatalog& catalog,
    const std::unordered_map<std::string, RuntimeFleetRobotState>& runtime_state);

/**
 * @brief PlanFleetRuntimeStateUpdate。
 * @param request 输入请求（字段见同名结构体）
 * @param base_catalog 参数
 * @param station_catalog 参数
 * @param runtime_state 字符串
 * @return 计算结果。
 */
FleetRuntimeUpdateDecision PlanFleetRuntimeStateUpdate(
    const FleetRuntimeUpdateRequest& request, const FleetCatalog& base_catalog,
    const StationCatalog* station_catalog,
    const std::unordered_map<std::string, RuntimeFleetRobotState>& runtime_state);

/**
 * @brief 构建 BuildFleetRobotListProjection。
 * @param catalog 参数
 * @param request 输入请求（字段见同名结构体）
 * @return 计算结果。
 */
FleetRobotListProjection BuildFleetRobotListProjection(
    const FleetCatalog& catalog, const FleetRobotListRequest& request);

/**
 * @brief PlanRemoteFleetTaskStateUpdate。
 * @param request 输入请求（字段见同名结构体）
 * @param remote_tasks 参数
 * @return 计算结果。
 */
RemoteFleetTaskUpdateDecision PlanRemoteFleetTaskStateUpdate(
    const RemoteFleetTaskUpdateRequest& request, const RemoteFleetTaskMap& remote_tasks);

/**
 * @brief PlanFleetStationTaskDispatch。
 * @param request 输入请求（字段见同名结构体）
 * @param remote_tasks 参数
 * @return 计算结果。
 */
FleetStationTaskDispatchDecision PlanFleetStationTaskDispatch(
    const FleetStationTaskDispatchRequest& request, const RemoteFleetTaskMap& remote_tasks);

}  // namespace robot_tasks
