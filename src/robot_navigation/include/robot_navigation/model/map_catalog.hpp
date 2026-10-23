/**
 * @file map_catalog.hpp
 * @brief 地图目录（数据模型层）。
 * @details 所属包：`robot_navigation`；所在层次：数据模型层。
 * @note 教学说明见 `docs/教学/` 对应章节。
 */

#pragma once

#include <filesystem>
#include <optional>
#include <string>
#include <vector>

namespace robot_navigation {

/**
 * @brief MapInfo 数据结构定义
 */
struct MapInfo {
  std::string name;
  std::filesystem::path yaml_path;
  std::filesystem::path image_path;
  double resolution = 0.0;
  bool valid = false;
  std::string message;
};

/**
 * @brief 地图目录（MapCatalog）
 */
class MapCatalog {
 public:
  /**
   * @brief 映射目录。
   * @param maps_directory 参数
   * @return 计算结果。
   */
  explicit MapCatalog(std::filesystem::path maps_directory);

  /**
   * @brief MapsDirectory。
   * @return 计算结果。
   */
  const std::filesystem::path& MapsDirectory() const;
  /**
   * @brief 扫描。
   * @return 计算结果。
   */
  std::vector<MapInfo> Scan() const;
  /**
   * @brief 查找 FindByName。
   * @param name 字符串
   * @return 计算结果。
   */
  std::optional<MapInfo> FindByName(const std::string& name) const;

 private:
  std::filesystem::path maps_directory_;
};

/**
 * @brief 加载 LoadMapInfo。
 * @param yaml_path 路径
 * @return 计算结果。
 */
std::optional<MapInfo> LoadMapInfo(const std::filesystem::path& yaml_path);

}  // namespace robot_navigation
