/**
 * @file facility_workflow.hpp
 * @brief 设施工作流（业务流程层）。
 * @details 所属包：`robot_tasks`；所在层次：业务流程层。
 * @note 教学说明见 `docs/教学/` 对应章节。
 */

#pragma once

#include <string>

#include "robot_tasks/model/facility_catalog.hpp"
#include "robot_tasks/model/facility_reservation.hpp"

namespace robot_tasks {

/**
 * @brief FacilityActionResourceSelection 数据结构定义
 */
struct FacilityActionResourceSelection {
  bool success = false;
  const FacilityResource* resource = nullptr;
  std::string message;
};

/**
 * @brief LiftSessionResult 数据结构定义
 */
struct LiftSessionResult {
  bool success = false;
  std::string session_id;
  std::string lift_id;
  std::string status;
  std::string message;
};

/**
 * @brief FacilityActionSupported。
 * @param resource_type 字符串
 * @param action 字符串
 * @return 满足条件返回 true，否则返回 false。
 */
bool FacilityActionSupported(const std::string& resource_type, const std::string& action);

/**
 * @brief 选择 SelectFacilityActionResource。
 * @param catalog 参数
 * @param reservations 参数
 * @param requested_resource_id 对应标识
 * @param requested_resource_type 字符串
 * @return 计算结果。
 */
FacilityActionResourceSelection SelectFacilityActionResource(
    const FacilityCatalog& catalog, const FacilityReservationMap& reservations,
    const std::string& requested_resource_id, const std::string& requested_resource_type);

/**
 * @brief 构建 BuildFacilityActionMissionId。
 * @param request_id 对应标识
 * @param resource_id 对应标识
 * @param action 字符串
 * @return 描述性字符串。
 */
std::string BuildFacilityActionMissionId(
    const std::string& request_id, const std::string& resource_id, const std::string& action);

/**
 * @brief 预约 ReserveFacilityActionResource。
 * @param reservations 参数
 * @param resource_by_mission 参数
 * @param resource 参数
 * @param mission_id 对应标识
 * @param hold_after_action 布尔标志
 */
void ReserveFacilityActionResource(
    FacilityReservationMap& reservations, MissionResourceMap& resource_by_mission,
    const FacilityResource& resource, const std::string& mission_id, bool hold_after_action);

/**
 * @brief 构建 BuildLiftSessionId。
 * @param requested_session_id 对应标识
 * @param lift_id 对应标识
 * @return 描述性字符串。
 */
std::string BuildLiftSessionId(const std::string& requested_session_id, const std::string& lift_id);

/**
 * @brief 构建 BuildLiftSessionHolderId。
 * @param requester_id 对应标识
 * @param session_id 对应标识
 * @return 描述性字符串。
 */
std::string BuildLiftSessionHolderId(
    const std::string& requester_id, const std::string& session_id);

/**
 * @brief 预约 ReserveLiftSessionResource。
 * @param reservations 参数
 * @param lift 参数
 * @param session_id 对应标识
 * @param holder_id 对应标识
 * @param release_existing_for_holder 布尔标志
 * @return 计算结果。
 */
LiftSessionResult ReserveLiftSessionResource(
    FacilityReservationMap& reservations, const FacilityResource& lift,
    const std::string& session_id, const std::string& holder_id,
    bool release_existing_for_holder);

/**
 * @brief 释放 ReleaseLiftSessionResource。
 * @param reservations 参数
 * @param lift_id 对应标识
 * @param session_id 对应标识
 * @return 计算结果。
 */
LiftSessionResult ReleaseLiftSessionResource(
    FacilityReservationMap& reservations, const std::string& lift_id,
    const std::string& session_id);

}  // namespace robot_tasks
