/**
 * @file facility_catalog.hpp
 * @brief 设施目录（数据模型层）。
 * @details 所属包：`robot_tasks`；所在层次：数据模型层。
 * @note 教学说明见 `docs/教学/` 对应章节。
 */

#pragma once

#include <filesystem>
#include <optional>
#include <string>
#include <vector>

namespace robot_tasks {

/**
 * @brief 设施资源（FacilityResource）
 */
struct FacilityResource {
  std::string id;
  std::string type;
  std::string station_id;
  bool enabled = true;
  bool exclusive = true;
};

/**
 * @brief 设施目录（FacilityCatalog）
 */
struct FacilityCatalog {
  std::vector<FacilityResource> resources;
};

/**
 * @brief 加载设施目录。
 * @param path 参数
 * @return 计算结果。
 */
std::optional<FacilityCatalog> LoadFacilityCatalog(const std::filesystem::path& path);
/**
 * @brief 查找设施资源。
 * @param catalog 参数
 * @param resource_id 对应标识
 * @return 计算结果。
 */
const FacilityResource* FindFacilityResource(
    const FacilityCatalog& catalog, const std::string& resource_id);
/**
 * @brief 查找 FindFirstAvailableResource。
 * @param catalog 参数
 * @param resource_type 字符串
 * @param unavailable_resource_ids 字符串
 * @return 计算结果。
 */
const FacilityResource* FindFirstAvailableResource(
    const FacilityCatalog& catalog, const std::string& resource_type,
    const std::vector<std::string>& unavailable_resource_ids);
/**
 * @brief 校验设施目录。
 * @param catalog 参数
 * @param message 字符串
 * @return 满足条件返回 true，否则返回 false。
 */
bool ValidateFacilityCatalog(const FacilityCatalog& catalog, std::string* message);

}  // namespace robot_tasks
