/**
 * @file mission_profile.hpp
 * @brief 任务配置档（数据模型层）。
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
 * @brief MissionWaypoint 数据结构定义
 */
struct MissionWaypoint {
  double x = 0.0;
  double y = 0.0;
  double yaw = 0.0;
};

/**
 * @brief 任务配置档（MissionProfile）
 */
struct MissionProfile {
  std::string mission_id;
  std::string frame_id = "map";
  bool loop = false;
  std::vector<MissionWaypoint> waypoints;
};

/**
 * @brief 加载任务配置档。
 * @param path 参数
 * @return 计算结果。
 */
std::optional<MissionProfile> LoadMissionProfile(const std::filesystem::path& path);
/**
 * @brief 校验任务配置档。
 * @param profile 参数
 * @param message 字符串
 * @return 满足条件返回 true，否则返回 false。
 */
bool ValidateMissionProfile(const MissionProfile& profile, std::string* message);

}  // namespace robot_tasks
