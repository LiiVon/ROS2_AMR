/**
 * @file submit_order_router.hpp
 * @brief 提交订单路由器（入口路由层）。
 * @details 所属包：`robot_tasks`；所在层次：入口路由层。
 * @note 教学说明见 `docs/教学/` 对应章节。
 */

#pragma once

#include <optional>
#include <string>
#include <vector>

namespace robot_tasks {

/**
 * @brief SubmitOrderKind 枚举定义
 */
enum class SubmitOrderKind {
  kTransport,
  kStationTransport,
  kStationOrderBatch,
  kStationSequence,
  kFleetStation,
  kBusiness,
  kScenarioTask,
  kScenarioWorkflow,
  kVda5050,
  kPickingWave,
  kReplenishment,
  kMilkRun,
};

/**
 * @brief SubmitOrderInput 数据结构定义
 */
struct SubmitOrderInput {
  std::string order_id;
  std::string order_type;
  int priority{0};
  std::string payload_json;
  std::vector<std::string> tags;
};

/**
 * @brief SubmitOrderRoute 数据结构定义
 */
struct SubmitOrderRoute {
  SubmitOrderKind kind{SubmitOrderKind::kTransport};
  std::string normalized_order_type;
  std::string order_id;
  int priority{0};
  bool start_if_idle{true};
  bool preempt_current{false};
  bool continue_on_error{false};
  int quantity{1};
  std::string frame_id{"map"};
  double pickup_x{0.0};
  double pickup_y{0.0};
  double pickup_yaw{0.0};
  double dropoff_x{0.0};
  double dropoff_y{0.0};
  double dropoff_yaw{0.0};
  std::string pickup_station;
  std::string dropoff_station;
  std::string batch_id;
  std::vector<std::string> order_ids;
  std::vector<std::string> pickup_station_ids;
  std::vector<std::string> dropoff_station_ids;
  std::vector<std::string> station_ids;
  std::string task_id;
  std::string required_capability{"station_transport"};
  std::string business_order_id;
  std::string business_type;
  std::string scenario_id;
  std::string workflow_id;
  std::string serial_number;
  std::string pickup_node_id;
  std::string dropoff_node_id;
  std::string wave_id;
  std::string sku_id;
  std::string tote_id;
  std::string pick_station;
  std::string source_station;
  std::string line_station;
  std::string run_id;
};

/**
 * @brief Vda5050StationOrderRequest 数据结构定义
 */
struct Vda5050StationOrderRequest {
  std::string order_id;
  std::string serial_number;
  std::string pickup_node_id;
  std::string dropoff_node_id;
  int priority{0};
  bool start_if_idle{true};
  bool preempt_current{false};
};

/**
 * @brief Vda5050StationOrderDecision 数据结构定义
 */
struct Vda5050StationOrderDecision {
  bool success{false};
  std::string order_id;
  std::string serial_number;
  std::string station_order_id;
  std::string pickup_station;
  std::string dropoff_station;
  int priority{0};
  bool start_if_idle{true};
  bool preempt_current{false};
  std::string message;
};

/**
 * @brief 归一化提交订单类型。
 * @param value 字符串
 * @return 描述性字符串。
 */
std::string NormalizeSubmitOrderType(const std::string& value);
/**
 * @brief 构建 BuildSubmitOrderRoute。
 * @param input 参数
 * @param message 字符串
 * @return 计算结果。
 */
std::optional<SubmitOrderRoute> BuildSubmitOrderRoute(
    const SubmitOrderInput& input, std::string* message);
Vda5050StationOrderDecision PlanVda5050StationOrder(
    const Vda5050StationOrderRequest& request);

}  // namespace robot_tasks
