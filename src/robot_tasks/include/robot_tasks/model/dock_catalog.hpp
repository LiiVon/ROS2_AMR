/**
 * @file dock_catalog.hpp
 * @brief 码头目录（数据模型层）。
 * @details 所属包：`robot_tasks`；所在层次：数据模型层。
 * @note 教学说明见 `docs/教学/` 对应章节。
 */

#pragma once

#include <filesystem>
#include <optional>
#include <string>
#include <vector>

#include "robot_tasks/workflow/docking_workflow.hpp"

namespace robot_tasks {

/**
 * @brief 码头（Dock）
 */
struct Dock {
  std::string id;
  std::string station_id;
  std::string approach_station_id;
  std::string charger_resource_id;
  bool enabled = true;
};

/**
 * @brief 码头目录（DockCatalog）
 */
struct DockCatalog {
  std::vector<Dock> docks;
};

/**
 * @brief DockStateProjection 数据结构定义
 */
struct DockStateProjection {
  std::string dock_id;
  std::string station_id;
  std::string approach_station_id;
  std::string charger_resource_id;
  bool enabled = false;
  bool available = false;
  std::string state;
  std::string mission_id;
};

/**
 * @brief DockListProjection 数据结构定义
 */
struct DockListProjection {
  std::vector<std::string> dock_ids;
  std::vector<std::string> station_ids;
  std::vector<std::string> approach_station_ids;
  std::vector<std::string> charger_resource_ids;
  std::vector<bool> enabled;
  std::vector<bool> available;
  std::vector<std::string> states;
  std::vector<std::string> mission_ids;
  std::string message;
};

/**
 * @brief 加载码头目录。
 * @param path 参数
 * @return 计算结果。
 */
std::optional<DockCatalog> LoadDockCatalog(const std::filesystem::path& path);
/**
 * @brief 查找码头。
 * @param catalog 参数
 * @param dock_id 对应标识
 * @return 计算结果。
 */
const Dock* FindDock(const DockCatalog& catalog, const std::string& dock_id);
/**
 * @brief 查找 FindFirstEnabledDock。
 * @param catalog 参数
 * @return 计算结果。
 */
const Dock* FindFirstEnabledDock(const DockCatalog& catalog);
/**
 * @brief 校验码头目录。
 * @param catalog 参数
 * @param message 字符串
 * @return 满足条件返回 true，否则返回 false。
 */
bool ValidateDockCatalog(const DockCatalog& catalog, std::string* message);
/**
 * @brief 构建 BuildDockStateProjection。
 * @param dock 参数
 * @param dock_states 参数
 * @param reservations 参数
 * @return 计算结果。
 */
DockStateProjection BuildDockStateProjection(
    const Dock& dock, const DockingStateMap& dock_states,
    const FacilityReservationMap& reservations);
/**
 * @brief 构建 BuildDockListProjection。
 * @param catalog 参数
 * @param dock_states 参数
 * @param reservations 参数
 * @param include_disabled 布尔标志
 * @return 计算结果。
 */
DockListProjection BuildDockListProjection(
    const DockCatalog& catalog, const DockingStateMap& dock_states,
    const FacilityReservationMap& reservations, bool include_disabled);

}  // namespace robot_tasks
