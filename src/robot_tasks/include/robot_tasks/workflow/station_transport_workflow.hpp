/**
 * @file station_transport_workflow.hpp
 * @brief 站点运输工作流（业务流程层）。
 * @details 所属包：`robot_tasks`；所在层次：业务流程层。
 * @note 教学说明见 `docs/教学/` 对应章节。
 */

#pragma once

#include <cstdint>
#include <optional>
#include <string>
#include <vector>

#include "robot_tasks/policy/mission_preflight.hpp"

namespace robot_tasks {

/**
 * @brief 站点运输提交闸门请求（StationTransportSubmitGateRequest）
 */
struct StationTransportSubmitGateRequest {
  std::string mission_id;
  bool mission_already_active_or_queued = false;
  bool station_catalog_loaded = true;
  MissionPreflightResult preflight;
  std::optional<std::vector<std::string>> route_path;
  bool traffic_intersection_locks_enabled = false;
};

/**
 * @brief 站点运输提交闸门判定（StationTransportSubmitGateDecision）
 */
struct StationTransportSubmitGateDecision {
  bool accepted = false;
  std::string mission_id;
  std::string message;
  std::vector<std::string> route_lock_ids;
};

/**
 * @brief PlanStationTransportSubmitGate。
 * @param request 输入请求（字段见同名结构体）
 * @return 计算结果。
 */
StationTransportSubmitGateDecision PlanStationTransportSubmitGate(
    const StationTransportSubmitGateRequest& request);

/**
 * @brief StationTransportPhase 枚举定义
 */
enum class StationTransportPhase {
  kQueued,
  kNavigatePickup,
  kWaitLoad,
  kNavigateDropoff,
  kWaitUnload,
  kCompleted,
  kFailed,
  kCanceled,
  kNeedsOperator,
};

/**
 * @brief StationTransportLeg 枚举定义
 */
enum class StationTransportLeg {
  kNone,
  kPickup,
  kDropoff,
};

/**
 * @brief 站点运输运行时状态（StationTransportRuntimeState）
 */
struct StationTransportRuntimeState {
  std::string mission_id;
  std::string pickup_station_id;
  std::string dropoff_station_id;
  std::string payload_id;
  StationTransportPhase phase = StationTransportPhase::kQueued;
  StationTransportLeg active_leg = StationTransportLeg::kNone;
  StationTransportLeg arrived_leg = StationTransportLeg::kNone;
  std::uint64_t dispatch_token = 0;
  bool payload_loaded = false;
};

/**
 * @brief StationTransportTransition 数据结构定义
 */
struct StationTransportTransition {
  bool accepted = false;
  StationTransportRuntimeState next_state;
  std::string message;
  std::optional<StationTransportLeg> dispatch_leg;
  bool release_resources = false;
};

/**
 * @brief StationTransportPhaseName。
 * @param phase 参数
 * @return 计算结果。
 */
const char* StationTransportPhaseName(StationTransportPhase phase);
/**
 * @brief StationTransportLegName。
 * @param leg 参数
 * @return 计算结果。
 */
const char* StationTransportLegName(StationTransportLeg leg);
/**
 * @brief 判断是否 IsStationTransportTerminal。
 * @param phase 参数
 * @return 满足条件返回 true，否则返回 false。
 */
bool IsStationTransportTerminal(StationTransportPhase phase);
/**
 * @brief 判断是否 IsStationTransportBusy。
 * @param phase 参数
 * @return 满足条件返回 true，否则返回 false。
 */
bool IsStationTransportBusy(StationTransportPhase phase);

/**
 * @brief PlanStationTransportDispatch。
 * @param state 输入状态（字段见同名结构体）
 * @param leg 参数
 * @param dispatch_token 整数参数
 * @return 计算结果。
 */
StationTransportTransition PlanStationTransportDispatch(
    const StationTransportRuntimeState& state,
    StationTransportLeg leg,
    std::uint64_t dispatch_token);

/**
 * @brief PlanStationTransportNavigationResult。
 * @param state 输入状态（字段见同名结构体）
 * @param leg 参数
 * @param dispatch_token 整数参数
 * @param succeeded 布尔标志
 * @param canceled 布尔标志
 * @return 计算结果。
 */
StationTransportTransition PlanStationTransportNavigationResult(
    const StationTransportRuntimeState& state,
    StationTransportLeg leg,
    std::uint64_t dispatch_token,
    bool succeeded,
    bool canceled);

/**
 * @brief PlanStationTransportConfirmLoad。
 * @param state 输入状态（字段见同名结构体）
 * @param station_id 对应标识
 * @param payload_id 对应标识
 * @return 计算结果。
 */
StationTransportTransition PlanStationTransportConfirmLoad(
    const StationTransportRuntimeState& state,
    const std::string& station_id,
    const std::string& payload_id);

/**
 * @brief PlanStationTransportConfirmUnload。
 * @param state 输入状态（字段见同名结构体）
 * @param station_id 对应标识
 * @param payload_id 对应标识
 * @return 计算结果。
 */
StationTransportTransition PlanStationTransportConfirmUnload(
    const StationTransportRuntimeState& state,
    const std::string& station_id,
    const std::string& payload_id);

/**
 * @brief PlanStationTransportCancel。
 * @param state 输入状态（字段见同名结构体）
 * @return 计算结果。
 */
StationTransportTransition PlanStationTransportCancel(
    const StationTransportRuntimeState& state);

}  // namespace robot_tasks
