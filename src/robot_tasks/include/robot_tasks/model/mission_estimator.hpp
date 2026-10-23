/**
 * @file mission_estimator.hpp
 * @brief 任务估算器（数据模型层）。
 * @details 所属包：`robot_tasks`；所在层次：数据模型层。
 * @note 教学说明见 `docs/教学/` 对应章节。
 */

#pragma once

#include "robot_tasks/model/mission_profile.hpp"

#include <string>

namespace robot_tasks {

/**
 * @brief MissionCostEstimate 数据结构定义
 */
struct MissionCostEstimate {
  int waypoint_count = 0;
  double distance_m = 0.0;
  double eta_sec = 0.0;
  double battery_drop_v = 0.0;
  double projected_battery_voltage = 0.0;
  bool battery_sufficient = true;
  std::string distance_source = "waypoint_polyline";
};

double EstimateMissionPathDistance(const MissionProfile& profile);
/**
 * @brief 估算 EstimateMissionCost。
 * @param profile 参数
 * @param nominal_speed_mps 线速度（米/秒）
 * @param battery_voltage 数值参数
 * @param battery_drop_per_meter 数值参数
 * @param minimum_battery_voltage 数值参数
 * @return 计算结果。
 */
MissionCostEstimate EstimateMissionCost(
    const MissionProfile& profile, double nominal_speed_mps, double battery_voltage,
    double battery_drop_per_meter, double minimum_battery_voltage);
/**
 * @brief 估算 EstimateDistanceCost。
 * @param distance_m 长度量（米）
 * @param waypoint_count 数量
 * @param nominal_speed_mps 线速度（米/秒）
 * @param battery_voltage 数值参数
 * @param battery_drop_per_meter 数值参数
 * @param minimum_battery_voltage 数值参数
 * @return 计算结果。
 */
MissionCostEstimate EstimateDistanceCost(
    double distance_m, int waypoint_count, double nominal_speed_mps, double battery_voltage,
    double battery_drop_per_meter, double minimum_battery_voltage);
/**
 * @brief 估算 EstimateDistanceCost。
 * @param distance_m 长度量（米）
 * @param waypoint_count 数量
 * @param nominal_speed_mps 线速度（米/秒）
 * @param battery_voltage 数值参数
 * @param battery_drop_per_meter 数值参数
 * @param minimum_battery_voltage 数值参数
 * @param distance_source 字符串
 * @return 计算结果。
 */
MissionCostEstimate EstimateDistanceCost(
    double distance_m, int waypoint_count, double nominal_speed_mps, double battery_voltage,
    double battery_drop_per_meter, double minimum_battery_voltage,
    const std::string& distance_source);

}  // namespace robot_tasks
