/**
 * @file scenario_catalog.hpp
 * @brief 场景目录（数据模型层）。
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
 * @brief ScenarioTask 数据结构定义
 */
struct ScenarioTask {
  std::string scenario_id;
  std::string task_id;
  std::string label;
  std::string task_type = "station_transport";
  std::string pickup_station_id;
  std::string dropoff_station_id;
  std::vector<std::string> station_sequence_ids;
  std::string required_capability = "station_transport";
  std::string resource_id;
  std::string resource_type;
  std::string action;
  bool hold_after_action = false;
  int default_priority = 30;
};

/**
 * @brief 场景工作流（ScenarioWorkflow）
 */
struct ScenarioWorkflow {
  std::string scenario_id;
  std::string workflow_id;
  std::string label;
  std::vector<std::string> task_ids;
  int default_priority = 30;
};

/**
 * @brief 场景目录（ScenarioCatalog）
 */
struct ScenarioCatalog {
  std::vector<ScenarioTask> tasks;
  std::vector<ScenarioWorkflow> workflows;
};

/**
 * @brief 场景工作流提交请求（ScenarioWorkflowSubmitRequest）
 */
struct ScenarioWorkflowSubmitRequest {
  std::string scenario_id;
  std::string workflow_id;
  std::string order_id;
  int priority{0};
  bool start_if_idle{true};
  bool preempt_current{false};
};

/**
 * @brief ScenarioWorkflowStepPlan 数据结构定义
 */
struct ScenarioWorkflowStepPlan {
  std::string scenario_id;
  std::string task_id;
  std::string order_id;
  int priority{0};
  bool start_if_idle{true};
  bool preempt_current{false};
};

/**
 * @brief 场景工作流提交判定（ScenarioWorkflowSubmitDecision）
 */
struct ScenarioWorkflowSubmitDecision {
  bool success{false};
  std::string scenario_id;
  std::string workflow_id;
  std::vector<ScenarioWorkflowStepPlan> steps;
  std::string message;
};

/**
 * @brief ScenarioTaskDispatchKind 枚举定义
 */
enum class ScenarioTaskDispatchKind {
  kFleetStation,
  kFacilityAction,
  kStationSequence,
};

/**
 * @brief ScenarioTaskSubmitRequest 数据结构定义
 */
struct ScenarioTaskSubmitRequest {
  std::string scenario_id;
  std::string task_id;
  std::string order_id;
  int priority{0};
  bool start_if_idle{true};
  bool preempt_current{false};
};

/**
 * @brief ScenarioTaskSubmitDecision 数据结构定义
 */
struct ScenarioTaskSubmitDecision {
  bool success{false};
  ScenarioTaskDispatchKind dispatch_kind{ScenarioTaskDispatchKind::kFleetStation};
  std::string scenario_id;
  std::string task_id;
  std::string order_id;
  int priority{0};
  bool start_if_idle{true};
  bool preempt_current{false};
  std::string resource_id;
  std::string resource_type;
  std::string action;
  bool hold_after_action{false};
  std::vector<std::string> station_ids;
  std::string required_capability{"station_transport"};
  std::string pickup_station;
  std::string dropoff_station;
  std::string message;
};

/**
 * @brief 加载场景目录。
 * @param path 参数
 * @return 计算结果。
 */
std::optional<ScenarioCatalog> LoadScenarioCatalog(const std::filesystem::path& path);
/**
 * @brief 查找 FindScenarioTask。
 * @param catalog 参数
 * @param scenario_id 对应标识
 * @param task_id 对应标识
 * @return 计算结果。
 */
const ScenarioTask* FindScenarioTask(
    const ScenarioCatalog& catalog, const std::string& scenario_id, const std::string& task_id);
/**
 * @brief 查找场景工作流。
 * @param catalog 参数
 * @param scenario_id 对应标识
 * @param workflow_id 对应标识
 * @return 计算结果。
 */
const ScenarioWorkflow* FindScenarioWorkflow(
    const ScenarioCatalog& catalog, const std::string& scenario_id,
    const std::string& workflow_id);
/**
 * @brief 校验场景目录。
 * @param catalog 参数
 * @param message 字符串
 * @return 满足条件返回 true，否则返回 false。
 */
bool ValidateScenarioCatalog(const ScenarioCatalog& catalog, std::string* message);
/**
 * @brief PlanScenarioWorkflowSubmit。
 * @param request 输入请求（字段见同名结构体）
 * @param catalog 参数
 * @param catalog_error_message 字符串
 * @return 计算结果。
 */
ScenarioWorkflowSubmitDecision PlanScenarioWorkflowSubmit(
    const ScenarioWorkflowSubmitRequest& request,
    const ScenarioCatalog* catalog,
    const std::string& catalog_error_message);
/**
 * @brief PlanScenarioTaskSubmit。
 * @param request 输入请求（字段见同名结构体）
 * @param catalog 参数
 * @param catalog_error_message 字符串
 * @return 计算结果。
 */
ScenarioTaskSubmitDecision PlanScenarioTaskSubmit(
    const ScenarioTaskSubmitRequest& request,
    const ScenarioCatalog* catalog,
    const std::string& catalog_error_message);

}  // namespace robot_tasks
