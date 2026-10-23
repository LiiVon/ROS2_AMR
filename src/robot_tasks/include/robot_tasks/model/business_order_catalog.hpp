/**
 * @file business_order_catalog.hpp
 * @brief 业务订单目录（数据模型层）。
 * @details 所属包：`robot_tasks`；所在层次：数据模型层。
 * @note 教学说明见 `docs/教学/` 对应章节。
 */

#pragma once

#include <filesystem>
#include <optional>
#include <string>
#include <vector>

namespace robot_tasks {

/**
 * @brief 业务订单类型（BusinessOrderType）
 */
struct BusinessOrderType {
  std::string business_type;
  std::string label;
  std::string scenario_id;
  std::string workflow_id;
};

/**
 * @brief 业务订单目录（BusinessOrderCatalog）
 */
struct BusinessOrderCatalog {
  std::vector<BusinessOrderType> order_types;
};

/**
 * @brief 业务订单提交请求（BusinessOrderSubmitRequest）
 */
struct BusinessOrderSubmitRequest {
  std::string business_order_id;
  std::string business_type;
  int priority{0};
  bool start_if_idle{true};
  bool preempt_current{false};
};

/**
 * @brief 业务订单提交判定（BusinessOrderSubmitDecision）
 */
struct BusinessOrderSubmitDecision {
  bool success{false};
  std::string business_order_id;
  std::string business_type;
  std::string routed_service{"/submit_scenario_workflow"};
  std::string routed_scenario_id;
  std::string routed_workflow_id;
  std::string workflow_order_id;
  int priority{0};
  bool start_if_idle{true};
  bool preempt_current{false};
  std::string message;
};

/**
 * @brief 加载业务订单目录。
 * @param path 参数
 * @return 计算结果。
 */
std::optional<BusinessOrderCatalog> LoadBusinessOrderCatalog(const std::filesystem::path& path);
/**
 * @brief 查找业务订单类型。
 * @param catalog 参数
 * @param business_type 字符串
 * @return 计算结果。
 */
const BusinessOrderType* FindBusinessOrderType(
    const BusinessOrderCatalog& catalog, const std::string& business_type);
/**
 * @brief PlanBusinessOrderSubmit。
 * @param request 输入请求（字段见同名结构体）
 * @param catalog 参数
 * @param catalog_error_message 字符串
 * @return 计算结果。
 */
BusinessOrderSubmitDecision PlanBusinessOrderSubmit(
    const BusinessOrderSubmitRequest& request,
    const BusinessOrderCatalog* catalog,
    const std::string& catalog_error_message);

}  // namespace robot_tasks
