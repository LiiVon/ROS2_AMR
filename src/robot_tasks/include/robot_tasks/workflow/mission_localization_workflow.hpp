/**
 * @file mission_localization_workflow.hpp
 * @brief 任务定位工作流（业务流程层）。
 * @details 所属包：`robot_tasks`；所在层次：业务流程层。
 * @note 教学说明见 `docs/教学/` 对应章节。
 */

#pragma once

#include <string>

namespace robot_tasks {

/**
 * @brief 任务定位状态（MissionLocalizationState）
 */
struct MissionLocalizationState {
  bool localized = true;
  std::string state = "OK";
  std::string station_id;
  std::string message = "localization covariance ok";
  double covariance_xy = 0.0;
  bool paused_mission = false;
};

/**
 * @brief 任务定位事件（MissionLocalizationEvent）
 */
struct MissionLocalizationEvent {
  bool emit = false;
  std::string mission_id = "localization";
  std::string state;
  std::string previous_state;
  std::string message;
  bool recoverable = false;
};

/**
 * @brief MissionLocalizationSideEffects 数据结构定义
 */
struct MissionLocalizationSideEffects {
  bool request_pause = false;
  bool request_resume = false;
  bool set_runner_state = false;
  std::string runner_state;
  std::string runner_message;
  bool runner_recoverable = true;
};

/**
 * @brief 任务定位判定（MissionLocalizationDecision）
 */
struct MissionLocalizationDecision {
  MissionLocalizationState localization;
  MissionLocalizationEvent event;
  MissionLocalizationSideEffects side_effects;
};

/**
 * @brief MissionLocalizationPoseUpdateRequest 数据结构定义
 */
struct MissionLocalizationPoseUpdateRequest {
  MissionLocalizationState current;
  double covariance_x = 0.0;
  double covariance_y = 0.0;
  double covariance_threshold = 0.5;
  bool pause_on_lost = true;
  bool mission_active = false;
  bool runner_paused = false;
};

/**
 * @brief MissionRelocalizationRequest 数据结构定义
 */
struct MissionRelocalizationRequest {
  MissionLocalizationState current;
  std::string reason;
};

/**
 * @brief MissionInitialPoseRecoveryRequest 数据结构定义
 */
struct MissionInitialPoseRecoveryRequest {
  MissionLocalizationState current;
  std::string station_id;
  bool mission_active = false;
  bool runner_paused = false;
  double covariance_xy = 0.05;
};

/**
 * @brief PlanLocalizationPoseUpdate。
 * @param request 输入请求（字段见同名结构体）
 * @return 计算结果。
 */
MissionLocalizationDecision PlanLocalizationPoseUpdate(
    const MissionLocalizationPoseUpdateRequest& request);
/**
 * @brief PlanRelocalizationRequest。
 * @param request 输入请求（字段见同名结构体）
 * @return 计算结果。
 */
MissionLocalizationDecision PlanRelocalizationRequest(
    const MissionRelocalizationRequest& request);
/**
 * @brief PlanInitialPoseRecovery。
 * @param request 输入请求（字段见同名结构体）
 * @return 计算结果。
 */
MissionLocalizationDecision PlanInitialPoseRecovery(
    const MissionInitialPoseRecoveryRequest& request);

}  // namespace robot_tasks
