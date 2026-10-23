/**
 * @file fulfillment_workflow.hpp
 * @brief 履约工作流（业务流程层）。
 * @details 所属包：`robot_tasks`；所在层次：业务流程层。
 * @note 教学说明见 `docs/教学/` 对应章节。
 */

#pragma once

#include <cstddef>
#include <string>
#include <vector>

namespace robot_tasks {

/**
 * @brief 履约订单运行时状态（FulfillmentOrderRuntimeState）
 */
struct FulfillmentOrderRuntimeState {
  std::string fulfillment_id;
  std::string order_type;
  std::string sku_id;
  std::string tote_id;
  std::string state = "CREATED";
  std::string pick_station;
  std::string dropoff_station;
  int requested_quantity = 0;
  int picked_quantity = 0;
  int short_quantity = 0;
  std::vector<std::string> mission_ids;
  std::vector<std::string> confirmation_ids;
};

/**
 * @brief 履约提交请求（FulfillmentSubmitRequest）
 */
struct FulfillmentSubmitRequest {
  std::string fulfillment_id;
  std::string order_type;
  std::size_t station_count = 0;
  bool duplicate = false;
  bool require_two_stations = false;
};

/**
 * @brief 履约提交判定（FulfillmentSubmitDecision）
 */
struct FulfillmentSubmitDecision {
  bool success = false;
  std::string fulfillment_id;
  std::string state = "REJECTED";
  std::string message;
};

/**
 * @brief PlanFulfillmentSubmit。
 * @param request 输入请求（字段见同名结构体）
 * @return 计算结果。
 */
FulfillmentSubmitDecision PlanFulfillmentSubmit(
    const FulfillmentSubmitRequest& request);

/**
 * @brief FulfillmentPickRequest 数据结构定义
 */
struct FulfillmentPickRequest {
  std::string fulfillment_id;
  std::string order_type;
  std::string current_state;
  int requested_quantity = 0;
  int picked_quantity = 0;
};

/**
 * @brief FulfillmentPickDecision 数据结构定义
 */
struct FulfillmentPickDecision {
  bool success = false;
  std::string fulfillment_id;
  std::string state = "REJECTED";
  std::string message;
  int picked_quantity = 0;
  int short_quantity = 0;
  bool resolve_pick_confirmation = false;
  std::string event_state;
  std::string event_previous_state;
  std::string event_message;
  bool event_recoverable = true;
};

/**
 * @brief FulfillmentDropoffRequest 数据结构定义
 */
struct FulfillmentDropoffRequest {
  std::string fulfillment_id;
  std::string order_type;
  std::string current_state;
  int short_quantity = 0;
  std::size_t confirmation_count = 0;
};

/**
 * @brief FulfillmentDropoffDecision 数据结构定义
 */
struct FulfillmentDropoffDecision {
  bool success = false;
  std::string fulfillment_id;
  std::string state = "REJECTED";
  std::string message;
  bool resolve_dropoff_confirmation = false;
  bool resolve_all_confirmations = false;
  std::string event_state;
  std::string event_previous_state;
  bool event_recoverable = true;
};

/**
 * @brief FulfillmentShortPickRequest 数据结构定义
 */
struct FulfillmentShortPickRequest {
  std::string fulfillment_id;
  std::string order_type;
  std::string current_state;
  int requested_quantity = 0;
  int short_quantity = 0;
};

/**
 * @brief FulfillmentShortPickDecision 数据结构定义
 */
struct FulfillmentShortPickDecision {
  bool success = false;
  std::string fulfillment_id;
  std::string state = "REJECTED";
  std::string message;
  int picked_quantity = 0;
  int short_quantity = 0;
  std::string event_state;
  std::string event_previous_state;
  bool event_recoverable = false;
};

/**
 * @brief FulfillmentStatusProjection 数据结构定义
 */
struct FulfillmentStatusProjection {
  bool success = true;
  std::string fulfillment_id;
  std::string order_type;
  std::string sku_id;
  std::string tote_id;
  std::string state;
  std::string pick_station;
  std::string dropoff_station;
  int requested_quantity = 0;
  int picked_quantity = 0;
  int short_quantity = 0;
  std::vector<std::string> mission_ids;
  std::vector<std::string> confirmation_ids;
  std::string message;
};

/**
 * @brief PlanFulfillmentPick。
 * @param request 输入请求（字段见同名结构体）
 * @return 计算结果。
 */
FulfillmentPickDecision PlanFulfillmentPick(
    const FulfillmentPickRequest& request);
/**
 * @brief PlanFulfillmentDropoff。
 * @param request 输入请求（字段见同名结构体）
 * @return 计算结果。
 */
FulfillmentDropoffDecision PlanFulfillmentDropoff(
    const FulfillmentDropoffRequest& request);
/**
 * @brief PlanFulfillmentShortPick。
 * @param request 输入请求（字段见同名结构体）
 * @return 计算结果。
 */
FulfillmentShortPickDecision PlanFulfillmentShortPick(
    const FulfillmentShortPickRequest& request);
/**
 * @brief 构建 BuildFulfillmentStatusProjection。
 * @param order 参数
 * @return 计算结果。
 */
FulfillmentStatusProjection BuildFulfillmentStatusProjection(
    const FulfillmentOrderRuntimeState& order);

}  // namespace robot_tasks
