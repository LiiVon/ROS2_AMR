/**
 * @file submit_order_behavior_tree.hpp
 * @brief 提交订单行为树（行为编排层）。
 * @details 所属包：`robot_tasks`；所在层次：行为编排层。
 * @note 教学说明见 `docs/教学/` 对应章节。
 */

#pragma once

#include <optional>
#include <string>

#include "robot_tasks/routing/submit_order_router.hpp"

namespace robot_tasks {

/**
 * @brief SubmitOrderBehaviorTreeInput 数据结构定义
 */
struct SubmitOrderBehaviorTreeInput {
  SubmitOrderInput request;
};

/**
 * @brief 提交订单行为树结果（SubmitOrderBehaviorTreeResult）
 */
struct SubmitOrderBehaviorTreeResult {
  bool success = false;
  std::string branch;
  std::string message;
  std::optional<SubmitOrderRoute> route;
};

/**
 * @brief DefaultSubmitOrderBehaviorTreeXml。
 * @return 描述性字符串。
 */
std::string DefaultSubmitOrderBehaviorTreeXml();
/**
 * @brief 周期推进提交订单行为树。
 * @param input 参数
 * @return 计算结果。
 */
SubmitOrderBehaviorTreeResult TickSubmitOrderBehaviorTree(
    const SubmitOrderBehaviorTreeInput& input);

}  // namespace robot_tasks
