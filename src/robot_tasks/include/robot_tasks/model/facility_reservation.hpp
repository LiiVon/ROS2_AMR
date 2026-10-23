/**
 * @file facility_reservation.hpp
 * @brief 设施预约（数据模型层）。
 * @details 所属包：`robot_tasks`；所在层次：数据模型层。
 * @note 教学说明见 `docs/教学/` 对应章节。
 */

#pragma once

#include <string>
#include <unordered_map>
#include <vector>

#include "robot_tasks/model/facility_catalog.hpp"

namespace robot_tasks {

/**
 * @brief 资源预约（ResourceReservation）
 */
struct ResourceReservation {
  std::string holder_id;
  std::string mission_id;
  std::string status = "reserved";
  bool hold_after_completion = true;
};

using FacilityReservationMap = std::unordered_map<std::string, ResourceReservation>;
using MissionResourceMap = std::unordered_map<std::string, std::string>;

/**
 * @brief 设施预约结果（FacilityReservationResult）
 */
struct FacilityReservationResult {
  bool success = false;
  std::string message;
  std::string resource_id;
  std::string holder_id;
  std::string status;
};

/**
 * @brief FacilityResourceListRequest 数据结构定义
 */
struct FacilityResourceListRequest {
  bool include_disabled = false;
  std::string resource_type;
};

/**
 * @brief FacilityResourceListProjection 数据结构定义
 */
struct FacilityResourceListProjection {
  bool success = true;
  std::vector<std::string> resource_ids;
  std::vector<std::string> resource_types;
  std::vector<std::string> station_ids;
  std::vector<bool> enabled;
  std::vector<bool> available;
  std::vector<std::string> holder_ids;
  std::vector<std::string> status;
  std::string message;
};

/**
 * @brief FacilityResourceStateProjection 数据结构定义
 */
struct FacilityResourceStateProjection {
  bool success = true;
  std::string resource_id;
  std::string station_id;
  bool available = false;
  std::string holder_id;
  std::string status;
  std::string message;
};

/**
 * @brief FacilityResourceAvailable。
 * @param resource 参数
 * @param reservations 参数
 * @return 满足条件返回 true，否则返回 false。
 */
bool FacilityResourceAvailable(
    const FacilityResource& resource, const FacilityReservationMap& reservations);
/**
 * @brief 设施资源状态。
 * @param resource 参数
 * @param reservations 参数
 * @return 描述性字符串。
 */
std::string FacilityResourceStatus(
    const FacilityResource& resource, const FacilityReservationMap& reservations);
/**
 * @brief 构建 BuildFacilityResourceListProjection。
 * @param catalog 参数
 * @param reservations 参数
 * @param request 输入请求（字段见同名结构体）
 * @return 计算结果。
 */
FacilityResourceListProjection BuildFacilityResourceListProjection(
    const FacilityCatalog& catalog,
    const FacilityReservationMap& reservations,
    const FacilityResourceListRequest& request);
/**
 * @brief 构建 BuildFacilityResourceStateProjection。
 * @param resource 参数
 * @param reservations 参数
 * @param resource_label 字符串
 * @return 计算结果。
 */
FacilityResourceStateProjection BuildFacilityResourceStateProjection(
    const FacilityResource& resource,
    const FacilityReservationMap& reservations,
    const std::string& resource_label);
/**
 * @brief ReservedResourceIds。
 * @param reservations 参数
 * @return 描述性字符串。
 */
std::vector<std::string> ReservedResourceIds(const FacilityReservationMap& reservations);
/**
 * @brief 释放 ReleaseResourcesForHolder。
 * @param reservations 参数
 * @param holder_id 对应标识
 * @return 计算结果。
 */
std::size_t ReleaseResourcesForHolder(
    FacilityReservationMap& reservations, const std::string& holder_id);
/**
 * @brief 预约设施资源。
 * @param reservations 参数
 * @param resource 参数
 * @param holder_id 对应标识
 * @param mission_id 对应标识
 * @param release_existing_for_holder 布尔标志
 * @return 计算结果。
 */
FacilityReservationResult ReserveFacilityResource(
    FacilityReservationMap& reservations, const FacilityResource& resource,
    const std::string& holder_id, const std::string& mission_id,
    bool release_existing_for_holder);
/**
 * @brief 释放设施资源。
 * @param reservations 参数
 * @param resource_id 对应标识
 * @param holder_id 对应标识
 * @return 计算结果。
 */
FacilityReservationResult ReleaseFacilityResource(
    FacilityReservationMap& reservations, const std::string& resource_id,
    const std::string& holder_id);
/**
 * @brief MarkResourceOccupiedForMission。
 * @param reservations 参数
 * @param resource_by_mission 参数
 * @param mission_id 对应标识
 * @return 满足条件返回 true，否则返回 false。
 */
bool MarkResourceOccupiedForMission(
    FacilityReservationMap& reservations, MissionResourceMap& resource_by_mission,
    const std::string& mission_id);
/**
 * @brief 释放 ReleaseResourceForMission。
 * @param reservations 参数
 * @param resource_by_mission 参数
 * @param mission_id 对应标识
 * @return 满足条件返回 true，否则返回 false。
 */
bool ReleaseResourceForMission(
    FacilityReservationMap& reservations, MissionResourceMap& resource_by_mission,
    const std::string& mission_id);

}  // namespace robot_tasks
