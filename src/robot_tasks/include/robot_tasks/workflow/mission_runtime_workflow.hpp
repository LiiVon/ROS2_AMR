/**
 * @file mission_runtime_workflow.hpp
 * @brief 任务运行时工作流（业务流程层）。
 * @details 所属包：`robot_tasks`；所在层次：业务流程层。
 * @note 教学说明见 `docs/教学/` 对应章节。
 */

#pragma once

#include <string>

namespace robot_tasks {

/**
 * @brief MissionRuntimeTickInput 数据结构定义
 */
struct MissionRuntimeTickInput {
  bool autostart = false;
  bool autostart_sent = false;
  bool mission_active = false;
  bool queue_start_requested = false;
};

/**
 * @brief MissionRuntimeTickDecision 数据结构定义
 */
struct MissionRuntimeTickDecision {
  bool start_default_mission = false;
  bool start_queued_mission = false;
  bool clear_queue_start_request = false;
  std::string branch = "publish_only";
};

/**
 * @brief DecideMissionRuntimeTick。
 * @param input 参数
 * @return 计算结果。
 */
MissionRuntimeTickDecision DecideMissionRuntimeTick(const MissionRuntimeTickInput& input);

}  // namespace robot_tasks
