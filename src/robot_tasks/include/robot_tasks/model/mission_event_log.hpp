/**
 * @file mission_event_log.hpp
 * @brief 任务事件日志（数据模型层）。
 * @details 所属包：`robot_tasks`；所在层次：数据模型层。
 * @note 教学说明见 `docs/教学/` 对应章节。
 */

#pragma once

#include <cstddef>
#include <string>
#include <vector>

namespace robot_tasks {

/**
 * @brief 任务事件（MissionEvent）
 */
struct MissionEvent {
  std::string stamp;
  std::string mission_id;
  std::string state;
  std::string previous_state;
  std::string message;
  bool recoverable = true;
};

/**
 * @brief 构造任务事件。
 * @param stamp 时间戳
 * @param mission_id 对应标识
 * @param state 输入状态（字段见同名结构体）
 * @param previous_state 字符串
 * @param message 字符串
 * @param recoverable 布尔标志
 * @return 计算结果。
 */
MissionEvent MakeMissionEvent(
    const std::string& stamp, const std::string& mission_id, const std::string& state,
    const std::string& previous_state, const std::string& message, bool recoverable);

/**
 * @brief 归一化事件限值。
 * @param requested_limit 参数
 * @param default_limit 数量/下标
 * @param max_limit 数量/下标
 * @return 计算结果。
 */
std::size_t NormalizeEventLimit(
    int requested_limit, std::size_t default_limit, std::size_t max_limit);

/**
 * @brief TrimMissionEvents。
 * @param events 元素列表
 * @param max_events 数量/下标
 * @return 计算结果。
 */
std::size_t TrimMissionEvents(std::vector<MissionEvent>& events, std::size_t max_events);

/**
 * @brief 追加任务事件。
 * @param events 元素列表
 * @param event 参数
 * @param max_events 数量/下标
 */
void AppendMissionEvent(
    std::vector<MissionEvent>& events, MissionEvent event, std::size_t max_events);

/**
 * @brief 选择 SelectRecentMissionEvents。
 * @param events 元素列表
 * @param limit 数量/下标
 * @param state_filter 字符串
 * @param mission_id_filter 字符串
 * @return 计算结果。
 */
std::vector<MissionEvent> SelectRecentMissionEvents(
    const std::vector<MissionEvent>& events, std::size_t limit,
    const std::string& state_filter = "", const std::string& mission_id_filter = "");

}  // namespace robot_tasks
