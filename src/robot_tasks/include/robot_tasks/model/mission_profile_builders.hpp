/**
 * @file mission_profile_builders.hpp
 * @brief （数据模型层）。
 * @details 所属包：`robot_tasks`；所在层次：数据模型层。
 * @note 教学说明见 `docs/教学/` 对应章节。
 */

#pragma once

#include <optional>
#include <string>
#include <vector>

#include "robot_tasks/model/mission_profile.hpp"
#include "robot_tasks/model/station_catalog.hpp"

namespace robot_tasks {

/**
 * @brief StationPair 数据结构定义
 */
struct StationPair {
  Station pickup;
  Station dropoff;
};

/**
 * @brief StationSequenceLeg 数据结构定义
 */
struct StationSequenceLeg {
  MissionProfile profile;
  std::string pickup_id;
  std::string dropoff_id;
};

/**
 * @brief 构建 BuildTwoWaypointProfile。
 * @param mission_id 对应标识
 * @param frame_id 对应标识
 * @param pickup_x 数值参数
 * @param pickup_y 数值参数
 * @param pickup_yaw 数值参数
 * @param dropoff_x 数值参数
 * @param dropoff_y 数值参数
 * @param dropoff_yaw 数值参数
 * @return 计算结果。
 */
MissionProfile BuildTwoWaypointProfile(
    const std::string& mission_id, const std::string& frame_id, double pickup_x,
    double pickup_y, double pickup_yaw, double dropoff_x, double dropoff_y,
    double dropoff_yaw);

/**
 * @brief 构建 BuildStationWaypointProfile。
 * @param mission_id 对应标识
 * @param station 参数
 * @return 计算结果。
 */
MissionProfile BuildStationWaypointProfile(const std::string& mission_id, const Station& station);

/**
 * @brief ResolveStationPair。
 * @param catalog 参数
 * @param pickup_station 字符串
 * @param dropoff_station 字符串
 * @param message 字符串
 * @return 计算结果。
 */
std::optional<StationPair> ResolveStationPair(
    const StationCatalog& catalog, const std::string& pickup_station,
    const std::string& dropoff_station, std::string* message);

/**
 * @brief 构建站点序列配置档。
 * @param catalog 参数
 * @param station_ids 字符串
 * @param base_id 对应标识
 * @param message 字符串
 * @return 计算结果。
 */
std::optional<std::vector<StationSequenceLeg>> BuildStationSequenceProfiles(
    const StationCatalog& catalog, const std::vector<std::string>& station_ids,
    const std::string& base_id, std::string* message);

/**
 * @brief 构建 BuildStationSequenceEstimateProfile。
 * @param catalog 参数
 * @param station_ids 字符串
 * @param message 字符串
 * @return 计算结果。
 */
std::optional<MissionProfile> BuildStationSequenceEstimateProfile(
    const StationCatalog& catalog, const std::vector<std::string>& station_ids,
    std::string* message);

}  // namespace robot_tasks
