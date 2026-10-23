/**
 * @file mission_resource_cleanup.hpp
 * @brief 任务资源清理（业务流程层）。
 * @details 所属包：`robot_tasks`；所在层次：业务流程层。
 * @note 教学说明见 `docs/教学/` 对应章节。
 */

#pragma once

#include <string>

#include "robot_tasks/model/facility_reservation.hpp"
#include "robot_tasks/model/traffic_reservation.hpp"

namespace robot_tasks {

/**
 * @brief 任务资源清理结果（MissionResourceCleanupResult）
 */
struct MissionResourceCleanupResult {
  bool facility_resource_released = false;
  bool route_locks_released = false;
};

/**
 * @brief 释放 ReleaseMissionResources。
 * @param facility_reservations 参数
 * @param facility_resource_by_mission 参数
 * @param route_locks 参数
 * @param route_locks_by_mission 参数
 * @param mission_id 对应标识
 * @return 计算结果。
 */
MissionResourceCleanupResult ReleaseMissionResources(
    FacilityReservationMap& facility_reservations, MissionResourceMap& facility_resource_by_mission,
    RouteLockMap& route_locks, RouteLockReservationMap& route_locks_by_mission,
    const std::string& mission_id);

}  // namespace robot_tasks
