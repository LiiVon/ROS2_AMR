/**
 * @file zone_catalog.hpp
 * @brief 区域目录（数据模型层）。
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
 * @brief 地图区域（MapZone）
 */
struct MapZone {
  std::string zone_id;
  std::string map_name;
  std::string type;
  std::string frame_id = "map";
  bool enabled = true;
  double speed_limit_mps = 0.0;
  std::vector<double> polygon_x;
  std::vector<double> polygon_y;
  std::string rule;
};

/**
 * @brief ZoneEvaluation 数据结构定义
 */
struct ZoneEvaluation {
  bool config_valid = false;
  std::string message;
  bool allowed = true;
  double speed_limit_mps = 0.0;
  std::vector<MapZone> matched_zones;
};

/**
 * @brief 区域目录（ZoneCatalog）
 */
class ZoneCatalog {
 public:
  /**
   * @brief 区域目录。
   * @param zones_file 文件路径
   * @return 计算结果。
   */
  explicit ZoneCatalog(std::filesystem::path zones_file);

  /**
   * @brief ZonesFile。
   * @return 计算结果。
   */
  const std::filesystem::path& ZonesFile() const;
  /**
   * @brief 加载 Load。
   * @param include_disabled 布尔标志
   * @return 计算结果。
   */
  std::vector<MapZone> Load(bool include_disabled = false) const;
  /**
   * @brief ForMap。
   * @param map_name 对应名称
   * @param include_disabled 布尔标志
   * @return 计算结果。
   */
  std::vector<MapZone> ForMap(const std::string& map_name, bool include_disabled = false) const;
  /**
   * @brief 评估点。
   * @param map_name 对应名称
   * @param x 数值参数
   * @param y 数值参数
   * @param default_speed_limit_mps 线速度（米/秒）
   * @return 计算结果。
   */
  ZoneEvaluation EvaluatePoint(
      const std::string& map_name, double x, double y, double default_speed_limit_mps) const;

 private:
  std::filesystem::path zones_file_;
};

/**
 * @brief 加载地图区域。
 * @param zones_file 文件路径
 * @return 计算结果。
 */
std::vector<MapZone> LoadMapZones(const std::filesystem::path& zones_file);
/**
 * @brief 判断是否包含点。
 * @param zone 参数
 * @param x 数值参数
 * @param y 数值参数
 * @return 满足条件返回 true，否则返回 false。
 */
bool ContainsPoint(const MapZone& zone, double x, double y);
/**
 * @brief 校验区域。
 * @param zone 参数
 * @return 描述性字符串。
 */
std::optional<std::string> ValidateZone(const MapZone& zone);

}  // namespace robot_navigation
