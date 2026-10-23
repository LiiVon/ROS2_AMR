/**
 * @file mission_queue.hpp
 * @brief 任务队列（数据模型层）。
 * @details 所属包：`robot_tasks`；所在层次：数据模型层。
 * @note 教学说明见 `docs/教学/` 对应章节。
 */

#pragma once

#include <cstddef>
#include <cstdint>
#include <optional>
#include <string>
#include <unordered_set>
#include <vector>

#include "robot_tasks/policy/mission_preflight.hpp"
#include "robot_tasks/model/mission_profile.hpp"

namespace robot_tasks {

/**
 * @brief 已排队任务（QueuedMission）
 */
struct QueuedMission {
  MissionProfile profile;
  std::string mission_file;
  int priority = 0;
  std::uint64_t sequence = 0;
};

/**
 * @brief 已排队任务启动请求（QueuedMissionStartRequest）
 */
struct QueuedMissionStartRequest {
  bool start_if_idle = false;
  bool preempt_current = false;
  bool mission_active = false;
};

/**
 * @brief 已排队任务启动判定（QueuedMissionStartDecision）
 */
struct QueuedMissionStartDecision {
  bool run_queue = false;
  bool request_queue_start = false;
  bool cancel_active = false;
};

/**
 * @brief MissionQueueDispatchResult 数据结构定义
 */
struct MissionQueueDispatchResult {
  bool success = false;
  bool queue_empty = false;
  bool all_paused = false;
  std::string message;
  std::optional<QueuedMission> mission;
  std::vector<QueuedMission> remaining_queue;
};

/**
 * @brief MissionQueueAdmissionRequest 数据结构定义
 */
struct MissionQueueAdmissionRequest {
  bool mission_active = false;
  std::size_t queue_size = 0;
  std::string message;
  bool set_state_when_idle = true;
};

/**
 * @brief MissionQueueAdmissionDecision 数据结构定义
 */
struct MissionQueueAdmissionDecision {
  bool success = true;
  int queue_size = 0;
  std::string message;
  bool set_state = false;
  std::string state = "QUEUED";
  std::string state_message;
  bool state_recoverable = true;
};

/**
 * @brief 任务提交闸门请求（MissionSubmitGateRequest）
 */
struct MissionSubmitGateRequest {
  std::string mission_id;
  std::string duplicate_message_prefix = "mission";
  std::string preflight_message_prefix = "mission";
  bool mission_already_active_or_queued = false;
  MissionPreflightResult preflight;
};

/**
 * @brief 任务提交闸门判定（MissionSubmitGateDecision）
 */
struct MissionSubmitGateDecision {
  bool accepted = false;
  std::string mission_id;
  std::string reject_reason;
  std::string message;
};

/**
 * @brief QueuedMissionCancelRequest 数据结构定义
 */
struct QueuedMissionCancelRequest {
  std::vector<QueuedMission> queue;
  std::string mission_id;
  bool cancel_active = false;
  bool mission_active = false;
  std::string active_mission_id;
};

/**
 * @brief QueuedMissionCancelDecision 数据结构定义
 */
struct QueuedMissionCancelDecision {
  bool success = false;
  bool active_cancel_requested = false;
  int queue_size = 0;
  std::string message;
  std::vector<QueuedMission> remaining_queue;
  std::vector<std::string> release_mission_ids;
  bool set_state = false;
  std::string state = "QUEUED";
  std::string state_message;
  bool state_recoverable = true;
};

/**
 * @brief StationOrderBatchCancelRequest 数据结构定义
 */
struct StationOrderBatchCancelRequest {
  std::vector<QueuedMission> queue;
  std::string batch_id;
  bool cancel_active = false;
  bool mission_active = false;
  std::string active_mission_id;
};

/**
 * @brief StationOrderBatchCancelDecision 数据结构定义
 */
struct StationOrderBatchCancelDecision {
  bool success = false;
  bool active_cancel_requested = false;
  int queue_size = 0;
  std::string message;
  std::vector<QueuedMission> remaining_queue;
  std::vector<std::string> canceled_mission_ids;
  std::vector<std::string> release_mission_ids;
  bool set_state = false;
  std::string state = "QUEUED";
  std::string state_message;
  bool state_recoverable = true;
};

/**
 * @brief StationOrderBatchSubmitRequest 数据结构定义
 */
struct StationOrderBatchSubmitRequest {
  std::string batch_id;
  std::vector<std::string> original_order_ids;
  std::vector<std::string> sanitized_order_ids;
  std::vector<std::string> pickup_station_ids;
  std::vector<std::string> dropoff_station_ids;
  int priority = 0;
  bool start_if_idle = false;
  bool preempt_current = false;
  bool continue_on_error = false;
};

/**
 * @brief StationOrderBatchItemPlan 数据结构定义
 */
struct StationOrderBatchItemPlan {
  bool accepted = false;
  std::string original_order_id;
  std::string order_id;
  std::string station_order_id;
  std::string pickup_station;
  std::string dropoff_station;
  int priority = 0;
  bool start_if_idle = false;
  bool preempt_current = false;
  std::string reject_order_id;
  std::string reject_message;
};

/**
 * @brief StationOrderBatchSubmitPlan 数据结构定义
 */
struct StationOrderBatchSubmitPlan {
  bool accepted = false;
  std::string message;
  std::vector<StationOrderBatchItemPlan> items;
};

/**
 * @brief StationOrderBatchSubmitSummary 数据结构定义
 */
struct StationOrderBatchSubmitSummary {
  bool success = false;
  int accepted_count = 0;
  int rejected_count = 0;
  std::string message;
};

/**
 * @brief QueuedMissionReprioritizeRequest 数据结构定义
 */
struct QueuedMissionReprioritizeRequest {
  std::vector<QueuedMission> queue;
  std::string mission_id;
  int priority = 0;
  bool mission_active = false;
};

/**
 * @brief QueuedMissionBatchReprioritizeRequest 数据结构定义
 */
struct QueuedMissionBatchReprioritizeRequest {
  std::vector<QueuedMission> queue;
  std::vector<std::string> mission_ids;
  std::vector<int> priorities;
  bool mission_active = false;
};

/**
 * @brief QueuedMissionReprioritizeDecision 数据结构定义
 */
struct QueuedMissionReprioritizeDecision {
  bool success = false;
  int queue_size = 0;
  std::string message;
  std::vector<QueuedMission> remaining_queue;
  std::vector<std::string> updated_mission_ids;
  bool set_state = false;
  std::string state = "QUEUED";
  std::string state_message;
  bool state_recoverable = true;
};

/**
 * @brief SortMissionQueue。
 * @param queue 元素列表
 */
void SortMissionQueue(std::vector<QueuedMission>& queue);
/**
 * @brief MissionIdQueued。
 * @param queue 元素列表
 * @param mission_id 对应标识
 * @return 满足条件返回 true，否则返回 false。
 */
bool MissionIdQueued(const std::vector<QueuedMission>& queue, const std::string& mission_id);
/**
 * @brief 移除 RemoveQueuedMissionById。
 * @param queue 元素列表
 * @param mission_id 对应标识
 * @return 计算结果。
 */
std::size_t RemoveQueuedMissionById(
    std::vector<QueuedMission>& queue, const std::string& mission_id);
/**
 * @brief ReprioritizeQueuedMission。
 * @param queue 元素列表
 * @param mission_id 对应标识
 * @param priority 参数
 * @return 满足条件返回 true，否则返回 false。
 */
bool ReprioritizeQueuedMission(
    std::vector<QueuedMission>& queue, const std::string& mission_id, int priority);
/**
 * @brief DecideQueuedMissionStart。
 * @param request 输入请求（字段见同名结构体）
 * @return 计算结果。
 */
QueuedMissionStartDecision DecideQueuedMissionStart(
    const QueuedMissionStartRequest& request);
/**
 * @brief MissionBelongsToWorkflowOrder。
 * @param mission_id 对应标识
 * @param order_id 对应标识
 * @return 满足条件返回 true，否则返回 false。
 */
bool MissionBelongsToWorkflowOrder(
    const std::string& mission_id, const std::string& order_id);
/**
 * @brief MissionBelongsToStationOrderBatch。
 * @param mission_id 对应标识
 * @param batch_id 对应标识
 * @return 满足条件返回 true，否则返回 false。
 */
bool MissionBelongsToStationOrderBatch(
    const std::string& mission_id, const std::string& batch_id);
/**
 * @brief PausedWorkflowForMission。
 * @param mission_id 对应标识
 * @param paused_order_ids 字符串
 * @return 描述性字符串。
 */
std::optional<std::string> PausedWorkflowForMission(
    const std::string& mission_id, const std::unordered_set<std::string>& paused_order_ids);
/**
 * @brief QueuedWorkflowMissionIds。
 * @param queue 元素列表
 * @param order_id 对应标识
 * @return 描述性字符串。
 */
std::vector<std::string> QueuedWorkflowMissionIds(
    const std::vector<QueuedMission>& queue, const std::string& order_id);
/**
 * @brief 移除 RemoveQueuedWorkflowMissions。
 * @param queue 元素列表
 * @param order_id 对应标识
 * @param removed_mission_ids 字符串
 * @return 计算结果。
 */
std::size_t RemoveQueuedWorkflowMissions(
    std::vector<QueuedMission>& queue, const std::string& order_id,
    std::vector<std::string>* removed_mission_ids);
/**
 * @brief 移除 RemoveQueuedStationOrderBatchMissions。
 * @param queue 元素列表
 * @param batch_id 对应标识
 * @param removed_mission_ids 字符串
 * @return 计算结果。
 */
std::size_t RemoveQueuedStationOrderBatchMissions(
    std::vector<QueuedMission>& queue, const std::string& batch_id,
    std::vector<std::string>* removed_mission_ids);
/**
 * @brief ReprioritizeQueuedMissionBatch。
 * @param queue 元素列表
 * @param mission_ids 字符串
 * @param priorities 元素列表
 * @return 描述性字符串。
 */
std::vector<std::string> ReprioritizeQueuedMissionBatch(
    std::vector<QueuedMission>& queue, const std::vector<std::string>& mission_ids,
    const std::vector<int>& priorities);
/**
 * @brief 查找 FindNextRunnableMissionIndex。
 * @param queue 元素列表
 * @param paused_order_ids 字符串
 * @return 计算结果。
 */
std::optional<std::size_t> FindNextRunnableMissionIndex(
    const std::vector<QueuedMission>& queue,
    const std::unordered_set<std::string>& paused_order_ids);
/**
 * @brief 取出 PopNextRunnableMission。
 * @param queue 元素列表
 * @param paused_order_ids 字符串
 * @return 计算结果。
 */
MissionQueueDispatchResult PopNextRunnableMission(
    std::vector<QueuedMission> queue,
    const std::unordered_set<std::string>& paused_order_ids);
/**
 * @brief PlanMissionQueueAdmission。
 * @param request 输入请求（字段见同名结构体）
 * @return 计算结果。
 */
MissionQueueAdmissionDecision PlanMissionQueueAdmission(
    const MissionQueueAdmissionRequest& request);
/**
 * @brief PlanMissionSubmitGate。
 * @param request 输入请求（字段见同名结构体）
 * @return 计算结果。
 */
MissionSubmitGateDecision PlanMissionSubmitGate(
    const MissionSubmitGateRequest& request);
/**
 * @brief PlanQueuedMissionCancel。
 * @param request 输入请求（字段见同名结构体）
 * @return 计算结果。
 */
QueuedMissionCancelDecision PlanQueuedMissionCancel(
    const QueuedMissionCancelRequest& request);
/**
 * @brief PlanStationOrderBatchCancel。
 * @param request 输入请求（字段见同名结构体）
 * @return 计算结果。
 */
StationOrderBatchCancelDecision PlanStationOrderBatchCancel(
    const StationOrderBatchCancelRequest& request);
/**
 * @brief PlanStationOrderBatchSubmit。
 * @param request 输入请求（字段见同名结构体）
 * @return 计算结果。
 */
StationOrderBatchSubmitPlan PlanStationOrderBatchSubmit(
    const StationOrderBatchSubmitRequest& request);
/**
 * @brief PlanStationOrderBatchSubmitSummary。
 * @param batch_id 对应标识
 * @param continue_on_error 布尔标志
 * @param accepted_count 数量
 * @param rejected_count 数量
 * @return 计算结果。
 */
StationOrderBatchSubmitSummary PlanStationOrderBatchSubmitSummary(
    const std::string& batch_id, bool continue_on_error, int accepted_count,
    int rejected_count);
/**
 * @brief PlanQueuedMissionReprioritize。
 * @param request 输入请求（字段见同名结构体）
 * @return 计算结果。
 */
QueuedMissionReprioritizeDecision PlanQueuedMissionReprioritize(
    const QueuedMissionReprioritizeRequest& request);
/**
 * @brief PlanQueuedMissionBatchReprioritize。
 * @param request 输入请求（字段见同名结构体）
 * @return 计算结果。
 */
QueuedMissionReprioritizeDecision PlanQueuedMissionBatchReprioritize(
    const QueuedMissionBatchReprioritizeRequest& request);

}  // namespace robot_tasks
