/**
 * @file zone_filter_masks.hpp
 * @brief 区域过滤掩膜（数据过滤层）。
 * @details 所属包：`robot_navigation`；所在层次：数据过滤层。
 * @note 教学说明见 `docs/教学/` 对应章节。
 */

#pragma once

#include <filesystem>
#include <string>
#include <vector>

#include "robot_navigation/model/zone_catalog.hpp"

namespace robot_navigation {

/**
 * @brief OccupancyMapInfo 数据结构定义
 */
struct OccupancyMapInfo {
  std::filesystem::path yaml_path;
  std::filesystem::path image_path;
  std::string map_name;
  int width = 0;
  int height = 0;
  double resolution = 0.0;
  double origin_x = 0.0;
  double origin_y = 0.0;
  double origin_yaw = 0.0;
};

/**
 * @brief ZoneFilterMaskGrid 数据结构定义
 */
struct ZoneFilterMaskGrid {
  int width = 0;
  int height = 0;
  double resolution = 0.0;
  double origin_x = 0.0;
  double origin_y = 0.0;
  double origin_yaw = 0.0;
  std::vector<int> keepout_values;
  std::vector<int> speed_values;
  int keepout_cells = 0;
  int speed_limit_cells = 0;
};

/**
 * @brief ZoneFilterExportSummary 数据结构定义
 */
struct ZoneFilterExportSummary {
  std::filesystem::path keepout_mask_yaml;
  std::filesystem::path speed_mask_yaml;
  std::filesystem::path nav2_profile_yaml;
  int keepout_cells = 0;
  int speed_limit_cells = 0;
};

/**
 * @brief 加载 LoadOccupancyMapInfo。
 * @param map_yaml 参数
 * @return 计算结果。
 */
OccupancyMapInfo LoadOccupancyMapInfo(const std::filesystem::path& map_yaml);
/**
 * @brief 构建 BuildZoneFilterMaskGrid。
 * @param map_info 参数
 * @param zones 元素列表
 * @return 计算结果。
 */
ZoneFilterMaskGrid BuildZoneFilterMaskGrid(
    const OccupancyMapInfo& map_info, const std::vector<MapZone>& zones);
/**
 * @brief WriteZoneFilterMaskAssets。
 * @param map_info 参数
 * @param grid 参数
 * @param output_dir 参数
 * @return 计算结果。
 */
ZoneFilterExportSummary WriteZoneFilterMaskAssets(
    const OccupancyMapInfo& map_info, const ZoneFilterMaskGrid& grid,
    const std::filesystem::path& output_dir);

}  // namespace robot_navigation
