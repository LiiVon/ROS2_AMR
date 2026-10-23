/**
 * @file station_sequence_behavior_tree.hpp
 * @brief 站点序列行为树（行为编排层）。
 * @details 所属包：`robot_tasks`；所在层次：行为编排层。
 * @note 教学说明见 `docs/教学/` 对应章节。
 */

#pragma once

#include <string>
#include <unordered_set>
#include <vector>

#include "robot_tasks/workflow/station_sequence_workflow.hpp"

namespace robot_tasks {

/**
 * @brief StationSequenceBehaviorTreeInput 数据结构定义
 */
struct StationSequenceBehaviorTreeInput {
  std::vector<StationSequenceLeg> legs;
  std::unordered_set<std::string> active_or_queued_mission_ids;
  std::vector<MissionPreflightResult> preflights;
  int priority = 0;
  std::uint64_t first_sequence = 0;
};

/**
 * @brief 站点序列行为树结果（StationSequenceBehaviorTreeResult）
 */
struct StationSequenceBehaviorTreeResult {
  bool success = false;
  std::string branch;
  std::string message;
  std::vector<std::string> mission_ids;
  std::vector<QueuedMission> queued_missions;
};

/**
 * @brief DefaultStationSequenceBehaviorTreeXml。
 * @return 描述性字符串。
 */
std::string DefaultStationSequenceBehaviorTreeXml();
/**
 * @brief 周期推进站点序列行为树。
 * @param input 参数
 * @return 计算结果。
 */
StationSequenceBehaviorTreeResult TickStationSequenceBehaviorTree(
    const StationSequenceBehaviorTreeInput& input);

}  // namespace robot_tasks
