/**
 * @file station_catalog.hpp
 * @brief 站点目录（数据模型层）。
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
 * @brief 站点（Station）
 */
struct Station {
  std::string id;
  std::string frame_id = "map";
  double x = 0.0;
  double y = 0.0;
  double yaw = 0.0;
};

/**
 * @brief StationEdge 数据结构定义
 */
struct StationEdge {
  std::string from;
  std::string to;
  double distance_m = 0.0;
  bool bidirectional = true;
  bool enabled = true;
};

/**
 * @brief 站点目录（StationCatalog）
 */
struct StationCatalog {
  std::vector<Station> stations;
  std::vector<StationEdge> edges;
};

/**
 * @brief StationListProjection 数据结构定义
 */
struct StationListProjection {
  std::vector<std::string> station_ids;
  std::vector<std::string> frame_ids;
  std::vector<double> x;
  std::vector<double> y;
  std::vector<double> yaw;
  std::string message;
};

/**
 * @brief StationRouteListProjection 数据结构定义
 */
struct StationRouteListProjection {
  std::vector<std::string> from_station;
  std::vector<std::string> to_station;
  std::vector<double> distance_m;
  std::vector<bool> bidirectional;
  std::vector<bool> enabled;
  std::string message;
};

/**
 * @brief 加载站点目录。
 * @param path 参数
 * @return 计算结果。
 */
std::optional<StationCatalog> LoadStationCatalog(const std::filesystem::path& path);
/**
 * @brief 查找站点。
 * @param catalog 参数
 * @param id 字符串
 * @return 计算结果。
 */
const Station* FindStation(const StationCatalog& catalog, const std::string& id);
/**
 * @brief 估算 EstimateRouteDistance。
 * @param catalog 参数
 * @param from 字符串
 * @param to 字符串
 * @return 计算结果（数值）。
 */
std::optional<double> EstimateRouteDistance(
    const StationCatalog& catalog, const std::string& from, const std::string& to);
/**
 * @brief 查找 FindRoutePath。
 * @param catalog 参数
 * @param from 字符串
 * @param to 字符串
 * @return 描述性字符串。
 */
std::optional<std::vector<std::string>> FindRoutePath(
    const StationCatalog& catalog, const std::string& from, const std::string& to);
/**
 * @brief 校验站点目录。
 * @param catalog 参数
 * @param message 字符串
 * @return 满足条件返回 true，否则返回 false。
 */
bool ValidateStationCatalog(const StationCatalog& catalog, std::string* message);
/**
 * @brief 构建 BuildStationListProjection。
 * @param catalog 参数
 * @return 计算结果。
 */
StationListProjection BuildStationListProjection(const StationCatalog& catalog);
/**
 * @brief 构建 BuildStationRouteListProjection。
 * @param catalog 参数
 * @return 计算结果。
 */
StationRouteListProjection BuildStationRouteListProjection(const StationCatalog& catalog);

}  // namespace robot_tasks
