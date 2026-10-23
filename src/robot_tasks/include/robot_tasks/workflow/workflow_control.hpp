/**
 * @file workflow_control.hpp
 * @brief 工作流控制（业务流程层）。
 * @details 所属包：`robot_tasks`；所在层次：业务流程层。
 * @note 教学说明见 `docs/教学/` 对应章节。
 */

#pragma once

#include <string>
#include <unordered_set>
#include <vector>

#include "robot_tasks/model/mission_event_log.hpp"
#include "robot_tasks/model/mission_queue.hpp"

namespace robot_tasks {

/**
 * @brief WorkflowActiveMission 数据结构定义
 */
struct WorkflowActiveMission {
  bool active = false;
  bool paused = false;
  std::string mission_id;
};

/**
 * @brief WorkflowCancelRequest 数据结构定义
 */
struct WorkflowCancelRequest {
  std::string order_id;
  bool cancel_active = false;
  std::vector<QueuedMission> queue;
  WorkflowActiveMission active;
};

/**
 * @brief WorkflowCancelDecision 数据结构定义
 */
struct WorkflowCancelDecision {
  bool success = false;
  bool active_cancel_requested = false;
  std::string message;
  std::vector<std::string> canceled_mission_ids;
  std::vector<QueuedMission> remaining_queue;
};

/**
 * @brief WorkflowPauseRequest 数据结构定义
 */
struct WorkflowPauseRequest {
  std::string order_id;
  bool pause_active = false;
  std::vector<QueuedMission> queue;
  std::unordered_set<std::string> paused_order_ids;
  WorkflowActiveMission active;
};

/**
 * @brief WorkflowPauseDecision 数据结构定义
 */
struct WorkflowPauseDecision {
  bool success = false;
  bool active_pause_requested = false;
  bool runner_paused = false;
  bool clear_queue_start_requested = false;
  std::string message;
  std::vector<std::string> affected_mission_ids;
  std::unordered_set<std::string> paused_order_ids;
};

/**
 * @brief WorkflowResumeRequest 数据结构定义
 */
struct WorkflowResumeRequest {
  std::string order_id;
  bool resume_active = false;
  std::vector<QueuedMission> queue;
  std::unordered_set<std::string> paused_order_ids;
  WorkflowActiveMission active;
};

/**
 * @brief WorkflowResumeDecision 数据结构定义
 */
struct WorkflowResumeDecision {
  bool success = false;
  bool active_resume_requested = false;
  bool runner_paused = false;
  bool queue_running = false;
  bool queue_start_requested = false;
  std::string message;
  std::vector<std::string> affected_mission_ids;
  std::unordered_set<std::string> paused_order_ids;
};

/**
 * @brief 工作流状态请求（WorkflowStatusRequest）
 */
struct WorkflowStatusRequest {
  std::string order_id;
  std::vector<QueuedMission> queue;
  WorkflowActiveMission active;
  std::unordered_set<std::string> paused_order_ids;
  std::vector<MissionEvent> events;
  std::string current_state;
};

/**
 * @brief 工作流状态快照（WorkflowStatusSnapshot）
 */
struct WorkflowStatusSnapshot {
  bool success = false;
  bool paused = false;
  std::string message;
  std::string state;
  std::string running_mission_id;
  std::vector<std::string> queued_mission_ids;
  std::vector<std::string> finished_mission_ids;
  int total_steps = 0;
  int finished_steps = 0;
  int queued_steps = 0;
};

/**
 * @brief PlanWorkflowCancel。
 * @param request 输入请求（字段见同名结构体）
 * @return 计算结果。
 */
WorkflowCancelDecision PlanWorkflowCancel(const WorkflowCancelRequest& request);
/**
 * @brief PlanWorkflowPause。
 * @param request 输入请求（字段见同名结构体）
 * @return 计算结果。
 */
WorkflowPauseDecision PlanWorkflowPause(const WorkflowPauseRequest& request);
/**
 * @brief PlanWorkflowResume。
 * @param request 输入请求（字段见同名结构体）
 * @return 计算结果。
 */
WorkflowResumeDecision PlanWorkflowResume(const WorkflowResumeRequest& request);
/**
 * @brief 构建工作流状态快照。
 * @param request 输入请求（字段见同名结构体）
 * @return 计算结果。
 */
WorkflowStatusSnapshot BuildWorkflowStatusSnapshot(const WorkflowStatusRequest& request);

}  // namespace robot_tasks
