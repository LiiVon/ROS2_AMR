/**
 * @file operator_snapshot.hpp
 * @brief 操作员快照（视图输出层）。
 * @details 所属包：`robot_tasks`；所在层次：视图输出层。
 * @note 教学说明见 `docs/教学/` 对应章节。
 */

#pragma once

#include <string>
#include <unordered_set>
#include <vector>

#include "robot_tasks/model/business_order_catalog.hpp"
#include "robot_tasks/model/facility_catalog.hpp"
#include "robot_tasks/model/facility_reservation.hpp"
#include "robot_tasks/model/fleet_catalog.hpp"
#include "robot_tasks/model/fleet_runtime_state.hpp"
#include "robot_tasks/model/mission_event_log.hpp"
#include "robot_tasks/model/mission_queue.hpp"

namespace robot_tasks {

/**
 * @brief 操作员快照请求（OperatorSnapshotRequest）
 */
struct OperatorSnapshotRequest {
  std::string mission_state;
  std::string previous_mission_state;
  std::string active_mission_id;
  std::string mission_message;
  bool recoverable = true;
  bool mission_active = false;
  bool paused = false;
  std::vector<QueuedMission> mission_queue;
  std::unordered_set<std::string> paused_workflow_order_ids;
  const FleetCatalog* fleet = nullptr;
  const RemoteFleetTaskMap* remote_tasks = nullptr;
  const FacilityCatalog* facilities = nullptr;
  const FacilityReservationMap* facility_reservations = nullptr;
  const BusinessOrderCatalog* business_orders = nullptr;
  std::vector<MissionEvent> mission_events;
  int event_limit = 0;
};

/**
 * @brief OperatorSnapshotProjection 数据结构定义
 */
struct OperatorSnapshotProjection {
  bool success = true;
  std::string mission_state;
  std::string previous_mission_state;
  std::string active_mission_id;
  std::string mission_message;
  bool recoverable = true;
  bool mission_active = false;
  bool paused = false;
  int queue_size = 0;
  std::vector<std::string> queued_mission_ids;
  std::vector<int> queued_priorities;
  std::vector<std::string> paused_workflow_order_ids;
  std::vector<std::string> fleet_robot_ids;
  std::vector<bool> fleet_enabled;
  std::vector<std::string> fleet_states;
  std::vector<double> fleet_battery_voltage;
  std::vector<std::string> fleet_current_station_ids;
  std::vector<std::string> fleet_capabilities;
  std::vector<std::string> remote_task_ids;
  std::vector<std::string> remote_task_robot_ids;
  std::vector<std::string> remote_task_states;
  std::vector<std::string> facility_resource_ids;
  std::vector<std::string> facility_resource_types;
  std::vector<std::string> facility_station_ids;
  std::vector<bool> facility_available;
  std::vector<std::string> facility_status;
  std::vector<std::string> business_types;
  std::vector<std::string> business_scenario_ids;
  std::vector<std::string> business_workflow_ids;
  std::vector<std::string> recent_event_stamp;
  std::vector<std::string> recent_event_mission_ids;
  std::vector<std::string> recent_event_states;
  std::vector<std::string> recent_event_messages;
  std::string message;
};

/**
 * @brief 构建 BuildOperatorSnapshotProjection。
 * @param request 输入请求（字段见同名结构体）
 * @return 计算结果。
 */
OperatorSnapshotProjection BuildOperatorSnapshotProjection(
    const OperatorSnapshotRequest& request);

}  // namespace robot_tasks
