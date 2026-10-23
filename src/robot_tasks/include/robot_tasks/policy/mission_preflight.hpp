/**
 * @file mission_preflight.hpp
 * @brief 任务预检（决策策略层）。
 * @details 所属包：`robot_tasks`；所在层次：决策策略层。
 * @note 教学说明见 `docs/教学/` 对应章节。
 */

#pragma once

#include <optional>
#include <string>

#include "robot_tasks/model/mission_estimator.hpp"
#include "robot_tasks/model/mission_profile.hpp"
#include "robot_tasks/model/station_catalog.hpp"

namespace robot_navigation {
/**
 * @brief 区域目录（ZoneCatalog）
 */
class ZoneCatalog;
}

namespace robot_tasks {

/**
 * @brief 任务预检配置（MissionPreflightConfig）
 */
struct MissionPreflightConfig {
  bool enabled = true;
  std::string map_name = "indoor_room";
  double default_speed_limit_mps = 0.6;
  double nominal_speed_mps = 0.35;
  double battery_voltage = 24.0;
  double battery_drop_per_meter = 0.015;
  double minimum_battery_voltage = 22.0;
  bool require_station_route = true;
};

/**
 * @brief 任务预检结果（MissionPreflightResult）
 */
struct MissionPreflightResult {
  bool allowed = true;
  std::string message = "preflight ok";
  MissionCostEstimate cost;
};

/**
 * @brief 校验任务预检。
 * @param profile 参数
 * @param zone_catalog 参数
 * @param config 配置参数（字段见同名结构体）
 * @param distance_override_m 长度量（米）
 * @return 计算结果。
 */
MissionPreflightResult ValidateMissionPreflight(
    const MissionProfile& profile, const robot_navigation::ZoneCatalog& zone_catalog,
    const MissionPreflightConfig& config,
    std::optional<double> distance_override_m = std::nullopt);

/**
 * @brief 校验站点运输预检。
 * @param profile 参数
 * @param station_catalog 参数
 * @param pickup_station 字符串
 * @param dropoff_station 字符串
 * @param zone_catalog 参数
 * @param config 配置参数（字段见同名结构体）
 * @param path_distance_override_m 长度量（米）
 * @return 计算结果。
 */
MissionPreflightResult ValidateStationTransportPreflight(
    const MissionProfile& profile, const StationCatalog& station_catalog,
    const std::string& pickup_station, const std::string& dropoff_station,
    const robot_navigation::ZoneCatalog& zone_catalog, const MissionPreflightConfig& config,
    std::optional<double> path_distance_override_m = std::nullopt);

}  // namespace robot_tasks
