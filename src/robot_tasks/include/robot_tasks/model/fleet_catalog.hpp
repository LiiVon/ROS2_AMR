/**
 * @file fleet_catalog.hpp
 * @brief 车队目录（数据模型层）。
 * @details 所属包：`robot_tasks`；所在层次：数据模型层。
 * @note 教学说明见 `docs/教学/` 对应章节。
 */

#pragma once

#include <filesystem>
#include <optional>
#include <string>
#include <vector>

#include "robot_tasks/model/station_catalog.hpp"

namespace robot_tasks {

/**
 * @brief FleetRobot 数据结构定义
 */
struct FleetRobot {
  std::string id;
  bool enabled = true;
  std::string state = "available";
  double battery_voltage = 0.0;
  double max_payload_kg = 0.0;
  std::string current_station_id;
  std::vector<std::string> capabilities;
};

/**
 * @brief 车队目录（FleetCatalog）
 */
struct FleetCatalog {
  std::vector<FleetRobot> robots;
};

/**
 * @brief FleetRobotTaskScore 数据结构定义
 */
struct FleetRobotTaskScore {
  std::string robot_id;
  double approach_distance_m = 0.0;
  double task_distance_m = 0.0;
  double total_distance_m = 0.0;
  double battery_voltage = 0.0;
};

/**
 * @brief 加载车队目录。
 * @param path 参数
 * @return 计算结果。
 */
std::optional<FleetCatalog> LoadFleetCatalog(const std::filesystem::path& path);
/**
 * @brief 查找 FindFleetRobot。
 * @param catalog 参数
 * @param robot_id 对应标识
 * @return 计算结果。
 */
const FleetRobot* FindFleetRobot(const FleetCatalog& catalog, const std::string& robot_id);
/**
 * @brief RobotHasCapability。
 * @param robot 参数
 * @param capability 字符串
 * @return 满足条件返回 true，否则返回 false。
 */
bool RobotHasCapability(const FleetRobot& robot, const std::string& capability);
/**
 * @brief 选择 SelectFleetRobot。
 * @param catalog 参数
 * @param required_capability 字符串
 * @param unavailable_robot_ids 字符串
 * @return 计算结果。
 */
const FleetRobot* SelectFleetRobot(
    const FleetCatalog& catalog, const std::string& required_capability,
    const std::vector<std::string>& unavailable_robot_ids);
/**
 * @brief 选择 SelectFleetRobotForStationTask。
 * @param fleet_catalog 参数
 * @param station_catalog 参数
 * @param required_capability 字符串
 * @param pickup_station 字符串
 * @param dropoff_station 字符串
 * @param unavailable_robot_ids 字符串
 * @param selected_score 参数
 * @return 计算结果。
 */
const FleetRobot* SelectFleetRobotForStationTask(
    const FleetCatalog& fleet_catalog, const StationCatalog& station_catalog,
    const std::string& required_capability, const std::string& pickup_station,
    const std::string& dropoff_station, const std::vector<std::string>& unavailable_robot_ids,
    FleetRobotTaskScore* selected_score);
/**
 * @brief 校验车队目录。
 * @param catalog 参数
 * @param message 字符串
 * @return 满足条件返回 true，否则返回 false。
 */
bool ValidateFleetCatalog(const FleetCatalog& catalog, std::string* message);

}  // namespace robot_tasks
