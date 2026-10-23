/**
 * @file mission_resource_cleanup.cpp
 * @brief 任务资源清理（业务流程层）。
 * @details 所属包：`robot_tasks`；所在层次：业务流程层。
 * @note 教学说明见 `docs/教学/` 对应章节。
 */

#include "robot_tasks/workflow/mission_resource_cleanup.hpp"

namespace robot_tasks {

MissionResourceCleanupResult ReleaseMissionResources(
    FacilityReservationMap& facility_reservations, MissionResourceMap& facility_resource_by_mission,
    RouteLockMap& route_locks, RouteLockReservationMap& route_locks_by_mission,
    const std::string& mission_id) {
  MissionResourceCleanupResult result;
  result.facility_resource_released =
      ReleaseResourceForMission(facility_reservations, facility_resource_by_mission, mission_id);
  result.route_locks_released =
      ReleaseRouteLocksForMission(route_locks, route_locks_by_mission, mission_id);
  return result;
}

}  // namespace robot_tasks
