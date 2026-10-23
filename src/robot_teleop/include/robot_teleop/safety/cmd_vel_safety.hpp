/**
 * @file cmd_vel_safety.hpp
 * @brief 速度指令安全（安全逻辑层）。
 * @details 所属包：`robot_teleop`；所在层次：安全逻辑层。
 * @note 教学说明见 `docs/教学/` 对应章节。
 */

#pragma once

#include <algorithm>
#include <cmath>
#include <string>

#include "geometry_msgs/msg/twist.hpp"

namespace robot_teleop {

/**
 * @brief 构造 MakeZeroTwist。
 * @return 计算结果。
 */
inline geometry_msgs::msg::Twist MakeZeroTwist() {
  geometry_msgs::msg::Twist twist;
  return twist;
}

/**
 * @brief 应用动态速度限值。
 * @param twist 速度指令
 * @param linear_limit_mps 线速度（米/秒）
 * @param angular_limit_radps 角速度（弧度/秒）
 * @return 计算结果。
 */
inline geometry_msgs::msg::Twist ApplyDynamicSpeedLimit(
    geometry_msgs::msg::Twist twist, const double linear_limit_mps,
    const double angular_limit_radps) {
  if (linear_limit_mps <= 0.0) {
    return twist;
  }

  const double linear_speed = std::hypot(twist.linear.x, twist.linear.y);
  if (linear_speed > linear_limit_mps && linear_speed > 0.0) {
    const double scale = linear_limit_mps / linear_speed;
    twist.linear.x *= scale;
    twist.linear.y *= scale;
    twist.angular.z *= scale;
  }

  if (angular_limit_radps > 0.0) {
    twist.angular.z = std::clamp(twist.angular.z, -angular_limit_radps, angular_limit_radps);
  }
  return twist;
}

/**
 * @brief CmdVelStopReason 枚举定义
 */
enum class CmdVelStopReason {
  kNone,
  kEmergencyStop,
  kRuntimeSafetyStop,
  kInputUnavailable,
  kInputStale,
};

/**
 * @brief 速度指令安全闸门请求（CmdVelSafetyGateRequest）
 */
struct CmdVelSafetyGateRequest {
  geometry_msgs::msg::Twist input_twist;
  bool emergency_stop = false;
  bool runtime_safety_stop = false;
  bool input_available = true;
  bool input_fresh = true;
  double dynamic_speed_limit_mps = 0.0;
  double dynamic_angular_limit_radps = 0.6;
};

/**
 * @brief 速度指令安全闸门判定（CmdVelSafetyGateDecision）
 */
struct CmdVelSafetyGateDecision {
  geometry_msgs::msg::Twist output_twist;
  CmdVelStopReason stop_reason = CmdVelStopReason::kNone;

  /**
   * @brief stopped。
   * @return 满足条件返回 true，否则返回 false。
   */
  bool stopped() const { return stop_reason != CmdVelStopReason::kNone; }
};

/**
 * @brief 选择指令来源。
 * @param manual_takeover 布尔标志
 * @param active_source 字符串
 * @return 描述性字符串。
 */
inline std::string SelectCommandSource(const bool manual_takeover,
                                       const std::string& active_source) {
  return manual_takeover ? "teleop" : active_source;
}

/**
 * @brief DescribeCommandState。
 * @param emergency_stop 布尔标志
 * @param runtime_safety_stop 布尔标志
 * @param manual_takeover 布尔标志
 * @param active_source 字符串
 * @return 描述性字符串。
 */
inline std::string DescribeCommandState(const bool emergency_stop,
                                        const bool runtime_safety_stop,
                                        const bool manual_takeover,
                                        const std::string& active_source) {
  if (emergency_stop) {
    return "emergency_stop";
  }
  if (runtime_safety_stop) {
    return "safety_stop";
  }
  if (manual_takeover) {
    return "manual_takeover:teleop";
  }
  return active_source;
}

/**
 * @brief 评估速度指令安全闸门。
 * @param request 输入请求（字段见同名结构体）
 * @return 计算结果。
 */
inline CmdVelSafetyGateDecision EvaluateCmdVelSafetyGate(
    const CmdVelSafetyGateRequest& request) {
  CmdVelSafetyGateDecision decision;
  if (request.emergency_stop) {
    decision.output_twist = MakeZeroTwist();
    decision.stop_reason = CmdVelStopReason::kEmergencyStop;
    return decision;
  }
  if (request.runtime_safety_stop) {
    decision.output_twist = MakeZeroTwist();
    decision.stop_reason = CmdVelStopReason::kRuntimeSafetyStop;
    return decision;
  }
  if (!request.input_available) {
    decision.output_twist = MakeZeroTwist();
    decision.stop_reason = CmdVelStopReason::kInputUnavailable;
    return decision;
  }
  if (!request.input_fresh) {
    decision.output_twist = MakeZeroTwist();
    decision.stop_reason = CmdVelStopReason::kInputStale;
    return decision;
  }

  decision.output_twist =
      ApplyDynamicSpeedLimit(request.input_twist, request.dynamic_speed_limit_mps,
                             request.dynamic_angular_limit_radps);
  decision.stop_reason = CmdVelStopReason::kNone;
  return decision;
}

}  // namespace robot_teleop
