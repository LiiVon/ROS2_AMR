/**
 * @file mission_runtime_workflow.cpp
 * @brief 任务运行时工作流（业务流程层）。
 * @details 所属包：`robot_tasks`；所在层次：业务流程层。
 * @note 教学说明见 `docs/教学/` 对应章节。
 */

#include "robot_tasks/workflow/mission_runtime_workflow.hpp"

namespace robot_tasks {

MissionRuntimeTickDecision DecideMissionRuntimeTick(const MissionRuntimeTickInput& input) {
  MissionRuntimeTickDecision decision;
  decision.start_default_mission =
      input.autostart && !input.autostart_sent && !input.mission_active;
  decision.start_queued_mission = input.queue_start_requested && !input.mission_active;
  decision.clear_queue_start_request = decision.start_queued_mission;

  if (decision.start_default_mission && decision.start_queued_mission) {
    decision.branch = "autostart_then_queue_if_still_idle";
  } else if (decision.start_default_mission) {
    decision.branch = "autostart";
  } else if (decision.start_queued_mission) {
    decision.branch = "queue_start";
  }
  return decision;
}

}  // namespace robot_tasks
