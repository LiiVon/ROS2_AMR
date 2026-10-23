/**
 * @file traffic_reservation.hpp
 * @brief 交通预约（数据模型层）。
 * @details 所属包：`robot_tasks`；所在层次：数据模型层。
 * @note 教学说明见 `docs/教学/` 对应章节。
 */

#pragma once

#include <optional>
#include <string>
#include <unordered_map>
#include <utility>
#include <vector>

namespace robot_tasks {

/**
 * @brief RouteLockReservation 数据结构定义
 */
struct RouteLockReservation {
  std::string mission_id;
  std::vector<std::string> route_lock_ids;
};

using RouteLockMap = std::unordered_map<std::string, std::string>;
using RouteLockReservationMap = std::unordered_map<std::string, RouteLockReservation>;

/**
 * @brief TrafficBlockRequest 数据结构定义
 */
struct TrafficBlockRequest {
  std::string from_station;
  std::string to_station;
  std::string reason;
  bool stations_known = false;
  bool route_exists = false;
};

/**
 * @brief 交通预约判定（TrafficReservationDecision）
 */
struct TrafficReservationDecision {
  bool success = false;
  bool mutate_state = false;
  std::string resource_id;
  std::string owner_id;
  std::string message;
};

/**
 * @brief TrafficClearDecision 数据结构定义
 */
struct TrafficClearDecision {
  bool success = false;
  bool erase_resource = false;
  std::string resource_id;
  std::string owner_id;
  std::string release_mission_id;
  std::string message;
};

/**
 * @brief TrafficDeadlockDecision 数据结构定义
 */
struct TrafficDeadlockDecision {
  bool deadlocked = false;
  std::vector<std::string> mission_ids;
  std::vector<std::string> conflict_resource_ids;
  std::vector<std::string> conflict_station_ids;
  std::vector<std::string> descriptions;
  std::string message = "no traffic deadlock detected";
};

/**
 * @brief RouteLockId。
 * @param from 字符串
 * @param to 字符串
 * @return 描述性字符串。
 */
std::string RouteLockId(const std::string& from, const std::string& to);
/**
 * @brief RouteNodeLockId。
 * @param station_id 对应标识
 * @return 描述性字符串。
 */
std::string RouteNodeLockId(const std::string& station_id);
/**
 * @brief 解析 ParseRouteLock。
 * @param resource_id 对应标识
 * @return 描述性字符串。
 */
std::optional<std::pair<std::string, std::string>> ParseRouteLock(
    const std::string& resource_id);
/**
 * @brief 判断是否 IsTrafficBlockOwner。
 * @param owner_id 对应标识
 * @return 满足条件返回 true，否则返回 false。
 */
bool IsTrafficBlockOwner(const std::string& owner_id);
/**
 * @brief TrafficBlockOwnerId。
 * @param reason 字符串
 * @return 描述性字符串。
 */
std::string TrafficBlockOwnerId(const std::string& reason);
/**
 * @brief PlanTrafficBlock。
 * @param request 输入请求（字段见同名结构体）
 * @param route_locks 参数
 * @return 计算结果。
 */
TrafficReservationDecision PlanTrafficBlock(
    const TrafficBlockRequest& request, const RouteLockMap& route_locks);
/**
 * @brief PlanTrafficUnblock。
 * @param from_station 字符串
 * @param to_station 字符串
 * @param route_locks 参数
 * @return 计算结果。
 */
TrafficReservationDecision PlanTrafficUnblock(
    const std::string& from_station, const std::string& to_station,
    const RouteLockMap& route_locks);
/**
 * @brief PlanTrafficReservationClear。
 * @param resource_id 对应标识
 * @param route_locks 参数
 * @return 计算结果。
 */
TrafficClearDecision PlanTrafficReservationClear(
    const std::string& resource_id, const RouteLockMap& route_locks);
/**
 * @brief PlanTrafficDeadlockDetection。
 * @param route_locks 参数
 * @return 计算结果。
 */
TrafficDeadlockDecision PlanTrafficDeadlockDetection(const RouteLockMap& route_locks);
/**
 * @brief 构建 BuildRouteLockIds。
 * @param station_path 路径
 * @param include_intersection_locks 布尔标志
 * @return 描述性字符串。
 */
std::vector<std::string> BuildRouteLockIds(
    const std::vector<std::string>& station_path, bool include_intersection_locks);
/**
 * @brief 预约 ReserveRouteLocksForMission。
 * @param route_locks 参数
 * @param route_locks_by_mission 参数
 * @param mission_id 对应标识
 * @param route_lock_ids 字符串
 * @param message 字符串
 * @return 满足条件返回 true，否则返回 false。
 */
bool ReserveRouteLocksForMission(
    RouteLockMap& route_locks, RouteLockReservationMap& route_locks_by_mission,
    const std::string& mission_id, const std::vector<std::string>& route_lock_ids,
    std::string* message);
/**
 * @brief 释放 ReleaseRouteLocksForMission。
 * @param route_locks 参数
 * @param route_locks_by_mission 参数
 * @param mission_id 对应标识
 * @return 满足条件返回 true，否则返回 false。
 */
bool ReleaseRouteLocksForMission(
    RouteLockMap& route_locks, RouteLockReservationMap& route_locks_by_mission,
    const std::string& mission_id);

}  // namespace robot_tasks
