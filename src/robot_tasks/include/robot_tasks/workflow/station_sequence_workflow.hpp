/**
 * @file station_sequence_workflow.hpp
 * @brief 站点序列工作流（业务流程层）。
 * @details 所属包：`robot_tasks`；所在层次：业务流程层。
 * @note 教学说明见 `docs/教学/` 对应章节。
 */

#pragma once

#include <cstddef>
#include <cstdint>
#include <string>
#include <unordered_set>
#include <vector>

#include "robot_tasks/policy/mission_preflight.hpp"
#include "robot_tasks/model/mission_profile_builders.hpp"
#include "robot_tasks/model/mission_queue.hpp"

namespace robot_tasks {

/**
 * @brief StationSequenceAdmissionResult 数据结构定义
 */
struct StationSequenceAdmissionResult {
  bool accepted = true;
  std::string message = "station sequence accepted";
};

/**
 * @brief 检查 CheckStationSequenceConflicts。
 * @param legs 元素列表
 * @param active_or_queued_mission_ids 字符串
 * @return 计算结果。
 */
StationSequenceAdmissionResult CheckStationSequenceConflicts(
    const std::vector<StationSequenceLeg>& legs,
    const std::unordered_set<std::string>& active_or_queued_mission_ids);

/**
 * @brief 检查 CheckStationSequenceLegPreflight。
 * @param leg 参数
 * @param preflight 参数
 * @return 计算结果。
 */
StationSequenceAdmissionResult CheckStationSequenceLegPreflight(
    const StationSequenceLeg& leg, const MissionPreflightResult& preflight);

/**
 * @brief StationSequenceMissionIds。
 * @param legs 元素列表
 * @return 描述性字符串。
 */
std::vector<std::string> StationSequenceMissionIds(
    const std::vector<StationSequenceLeg>& legs);

/**
 * @brief 构建 BuildStationSequenceQueuedMissions。
 * @param legs 元素列表
 * @param priority 参数
 * @param first_sequence 整数参数
 * @return 计算结果。
 */
std::vector<QueuedMission> BuildStationSequenceQueuedMissions(
    const std::vector<StationSequenceLeg>& legs, int priority,
    std::uint64_t first_sequence);

/**
 * @brief 构建站点序列已排队消息。
 * @param queued_count 数量
 * @return 描述性字符串。
 */
std::string BuildStationSequenceQueuedMessage(std::size_t queued_count);

}  // namespace robot_tasks
