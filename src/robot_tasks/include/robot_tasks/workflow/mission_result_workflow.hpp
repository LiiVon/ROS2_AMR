/**
 * @file mission_result_workflow.hpp
 * @brief 任务结果工作流（业务流程层）。
 * @details 所属包：`robot_tasks`；所在层次：业务流程层。
 * @note 教学说明见 `docs/教学/` 对应章节。
 */

#pragma once

#include <string>

namespace robot_tasks {

/**
 * @brief MissionActionResultCode 枚举定义
 */
enum class MissionActionResultCode {
  kSucceeded,
  kCanceled,
  kFailed,
};

/**
 * @brief MissionResultWorkflowInput 数据结构定义
 */
struct MissionResultWorkflowInput {
  MissionActionResultCode code = MissionActionResultCode::kFailed;
  bool result_success = false;
  std::string result_message;
  bool cancellation_requested = false;
  bool completed_dock = false;
};

/**
 * @brief 任务结果工作流判定（MissionResultWorkflowDecision）
 */
struct MissionResultWorkflowDecision {
  std::string branch;
  bool mission_succeeded = false;
  bool clear_cancellation_requested = false;
  bool mark_docking_terminal = false;
  std::string docking_terminal_state;
  std::string docking_terminal_message;
  bool release_all_resources = false;
  bool release_route_locks = false;
  bool probe_docking_contact_failure = false;
  bool reset_failure_retry_count = false;
  bool mark_resource_occupied = false;
  bool complete_docking = false;
  std::string state;
  std::string state_message;
  bool recoverable = false;
  bool handle_failure = false;
  std::string failure_message;
};

/**
 * @brief PlanMissionResultWorkflow。
 * @param input 参数
 * @return 计算结果。
 */
MissionResultWorkflowDecision PlanMissionResultWorkflow(
    const MissionResultWorkflowInput& input);

/**
 * @brief MissionDockAfterResultRequest 数据结构定义
 */
struct MissionDockAfterResultRequest {
  bool mission_succeeded = false;
  bool return_to_dock_after_mission = false;
  bool completed_dock = false;
  bool mission_queue_empty = true;
  bool dock_profile_available = false;
};

/**
 * @brief ShouldQueueDockAfterResult。
 * @param request 输入请求（字段见同名结构体）
 * @return 满足条件返回 true，否则返回 false。
 */
bool ShouldQueueDockAfterResult(const MissionDockAfterResultRequest& request);

/**
 * @brief MissionPostResultQueueRequest 数据结构定义
 */
struct MissionPostResultQueueRequest {
  bool existing_queue_start_requested = false;
  bool auto_start_queue = true;
  bool queue_running = false;
  bool mission_queue_empty = true;
};

/**
 * @brief MissionPostResultQueueDecision 数据结构定义
 */
struct MissionPostResultQueueDecision {
  bool queue_start_requested = false;
  bool queue_running = false;
};

/**
 * @brief PlanMissionPostResultQueue。
 * @param request 输入请求（字段见同名结构体）
 * @return 计算结果。
 */
MissionPostResultQueueDecision PlanMissionPostResultQueue(
    const MissionPostResultQueueRequest& request);

/**
 * @brief MissionResultPostActionRequest 数据结构定义
 */
struct MissionResultPostActionRequest {
  bool mission_succeeded = false;
  bool return_to_dock_after_mission = false;
  bool completed_dock = false;
  bool mission_queue_empty = true;
  bool dock_profile_available = false;
  bool existing_queue_start_requested = false;
  bool auto_start_queue = true;
  bool queue_running = false;
};

/**
 * @brief MissionResultPostActionDecision 数据结构定义
 */
struct MissionResultPostActionDecision {
  bool queue_dock_after_result = false;
  MissionPostResultQueueDecision queue;
};

/**
 * @brief PlanMissionResultPostActions。
 * @param request 输入请求（字段见同名结构体）
 * @return 计算结果。
 */
MissionResultPostActionDecision PlanMissionResultPostActions(
    const MissionResultPostActionRequest& request);

}  // namespace robot_tasks
