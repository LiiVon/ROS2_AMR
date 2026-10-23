/**
 * @file mission_runner_node.cpp
 * @brief 任务运行器节点（节点入口层）。
 * @details 所属包：`robot_tasks`；所在层次：节点入口层。
 * @note 教学说明见 `docs/教学/` 对应章节。
 */

#include <chrono>
#include <cmath>
#include <algorithm>
#include <cctype>
#include <cstddef>
#include <filesystem>
#include <cstdint>
#include <memory>
#include <mutex>
#include <optional>
#include <string>
#include <unordered_set>
#include <unordered_map>
#include <utility>
#include <vector>

#include "ament_index_cpp/get_package_share_directory.hpp"
#include "geometry_msgs/msg/pose_stamped.hpp"
#include "geometry_msgs/msg/pose_with_covariance_stamped.hpp"
#include "nav2_msgs/action/compute_path_to_pose.hpp"
#include "nav_msgs/msg/path.hpp"
#include "rclcpp/executors/single_threaded_executor.hpp"
#include "rclcpp/rclcpp.hpp"
#include "rclcpp_action/rclcpp_action.hpp"
#include "robot_interfaces_navigation/action/navigate_sequence.hpp"
#include "robot_interfaces_core/msg/chassis_state.hpp"
#include "robot_interfaces_navigation/msg/localization_health.hpp"
#include "robot_interfaces_facility/msg/payload_state.hpp"
#include "robot_interfaces_core/msg/robot_state.hpp"
#include "robot_interfaces_navigation/msg/safety_state.hpp"
#include "robot_interfaces_business/srv/cancel_workflow.hpp"
#include "robot_interfaces_business/srv/confirm_dropoff.hpp"
#include "robot_interfaces_business/srv/confirm_pick.hpp"
#include "robot_interfaces_business/srv/get_operator_snapshot.hpp"
#include "robot_interfaces_business/srv/get_order_fulfillment_status.hpp"
#include "robot_interfaces_business/srv/get_workflow_status.hpp"
#include "robot_interfaces_business/srv/list_business_order_types.hpp"
#include "robot_interfaces_business/srv/list_scenario_tasks.hpp"
#include "robot_interfaces_business/srv/list_scenario_workflows.hpp"
#include "robot_interfaces_business/srv/pause_workflow.hpp"
#include "robot_interfaces_business/srv/report_short_pick.hpp"
#include "robot_interfaces_business/srv/resume_workflow.hpp"
#include "robot_interfaces_business/srv/submit_business_order.hpp"
#include "robot_interfaces_business/srv/submit_milk_run.hpp"
#include "robot_interfaces_business/srv/submit_order.hpp"
#include "robot_interfaces_business/srv/submit_picking_wave.hpp"
#include "robot_interfaces_business/srv/submit_replenishment_order.hpp"
#include "robot_interfaces_business/srv/submit_scenario_task.hpp"
#include "robot_interfaces_business/srv/submit_scenario_workflow.hpp"
#include "robot_interfaces_business/srv/submit_transport_order.hpp"
#include "robot_interfaces_business/srv/submit_vda5050_order.hpp"
#include "robot_interfaces_facility/srv/confirm_load.hpp"
#include "robot_interfaces_facility/srv/confirm_station_action.hpp"
#include "robot_interfaces_facility/srv/confirm_unload.hpp"
#include "robot_interfaces_facility/srv/execute_facility_action.hpp"
#include "robot_interfaces_facility/srv/execute_top_module_action.hpp"
#include "robot_interfaces_facility/srv/get_door_state.hpp"
#include "robot_interfaces_facility/srv/get_lift_state.hpp"
#include "robot_interfaces_facility/srv/get_pending_confirmations.hpp"
#include "robot_interfaces_facility/srv/list_facility_resources.hpp"
#include "robot_interfaces_facility/srv/reject_station_action.hpp"
#include "robot_interfaces_facility/srv/release_facility_resource.hpp"
#include "robot_interfaces_facility/srv/release_lift_session.hpp"
#include "robot_interfaces_facility/srv/request_lift_session.hpp"
#include "robot_interfaces_facility/srv/request_station_confirmation.hpp"
#include "robot_interfaces_facility/srv/reserve_facility_resource.hpp"
#include "robot_interfaces_facility/srv/set_payload_loaded.hpp"
#include "robot_interfaces_fleet/srv/list_fleet_robots.hpp"
#include "robot_interfaces_fleet/srv/preview_fleet_station_task.hpp"
#include "robot_interfaces_fleet/srv/submit_fleet_station_task.hpp"
#include "robot_interfaces_fleet/srv/update_fleet_robot_state.hpp"
#include "robot_interfaces_fleet/srv/update_remote_task_state.hpp"
#include "robot_interfaces_mission/srv/block_station_route.hpp"
#include "robot_interfaces_mission/srv/cancel_queued_mission.hpp"
#include "robot_interfaces_mission/srv/cancel_station_order_batch.hpp"
#include "robot_interfaces_mission/srv/clear_traffic_reservation.hpp"
#include "robot_interfaces_mission/srv/detect_traffic_deadlock.hpp"
#include "robot_interfaces_mission/srv/enqueue_mission.hpp"
#include "robot_interfaces_mission/srv/estimate_mission_cost.hpp"
#include "robot_interfaces_mission/srv/estimate_station_sequence.hpp"
#include "robot_interfaces_mission/srv/estimate_station_transport.hpp"
#include "robot_interfaces_mission/srv/get_dock_state.hpp"
#include "robot_interfaces_mission/srv/get_mission_recovery_state.hpp"
#include "robot_interfaces_mission/srv/list_docks.hpp"
#include "robot_interfaces_mission/srv/list_mission_events.hpp"
#include "robot_interfaces_mission/srv/list_station_routes.hpp"
#include "robot_interfaces_mission/srv/list_stations.hpp"
#include "robot_interfaces_mission/srv/list_traffic_reservations.hpp"
#include "robot_interfaces_mission/srv/preempt_mission.hpp"
#include "robot_interfaces_mission/srv/reprioritize_batch.hpp"
#include "robot_interfaces_mission/srv/reprioritize_queued_mission.hpp"
#include "robot_interfaces_mission/srv/request_charging.hpp"
#include "robot_interfaces_mission/srv/request_docking.hpp"
#include "robot_interfaces_mission/srv/request_opportunity_charging.hpp"
#include "robot_interfaces_mission/srv/request_undocking.hpp"
#include "robot_interfaces_mission/srv/submit_station_order_batch.hpp"
#include "robot_interfaces_mission/srv/submit_station_sequence_task.hpp"
#include "robot_interfaces_mission/srv/submit_station_transport_order.hpp"
#include "robot_interfaces_mission/srv/unblock_station_route.hpp"
#include "robot_interfaces_navigation/srv/clear_obstacle_blockage.hpp"
#include "robot_interfaces_navigation/srv/report_obstacle_blockage.hpp"
#include "robot_interfaces_navigation/srv/request_relocalization.hpp"
#include "robot_interfaces_navigation/srv/set_dynamic_speed_limit.hpp"
#include "robot_interfaces_navigation/srv/set_initial_pose_from_station.hpp"
#include "robot_navigation/model/zone_catalog.hpp"
#include "robot_tasks/model/business_order_catalog.hpp"
#include "robot_tasks/model/dock_catalog.hpp"
#include "robot_tasks/workflow/docking_workflow.hpp"
#include "robot_tasks/model/facility_catalog.hpp"
#include "robot_tasks/model/facility_reservation.hpp"
#include "robot_tasks/workflow/facility_workflow.hpp"
#include "robot_tasks/model/fleet_catalog.hpp"
#include "robot_tasks/model/fleet_runtime_state.hpp"
#include "robot_tasks/workflow/fulfillment_workflow.hpp"
#include "robot_tasks/workflow/mission_charging_workflow.hpp"
#include "robot_tasks/model/mission_event_log.hpp"
#include "robot_tasks/model/mission_estimator.hpp"
#include "robot_tasks/workflow/mission_localization_workflow.hpp"
#include "robot_tasks/policy/mission_preflight.hpp"
#include "robot_tasks/model/mission_profile.hpp"
#include "robot_tasks/model/mission_profile_builders.hpp"
#include "robot_tasks/behavior/mission_queue_behavior_tree.hpp"
#include "robot_tasks/model/mission_queue.hpp"
#include "robot_tasks/behavior/mission_recovery_behavior_tree.hpp"
#include "robot_tasks/policy/mission_recovery_policy.hpp"
#include "robot_tasks/behavior/mission_resource_cleanup_behavior_tree.hpp"
#include "robot_tasks/workflow/mission_resource_cleanup.hpp"
#include "robot_tasks/behavior/mission_result_behavior_tree.hpp"
#include "robot_tasks/workflow/mission_result_workflow.hpp"
#include "robot_tasks/behavior/mission_runtime_behavior_tree.hpp"
#include "robot_tasks/workflow/mission_safety_workflow.hpp"
#include "robot_tasks/report/operator_snapshot.hpp"
#include "robot_tasks/workflow/payload_workflow.hpp"
#include "robot_tasks/behavior/queued_mission_start_behavior_tree.hpp"
#include "robot_tasks/model/scenario_catalog.hpp"
#include "robot_tasks/workflow/station_confirmation_workflow.hpp"
#include "robot_tasks/workflow/station_sequence_workflow.hpp"
#include "robot_tasks/model/station_catalog.hpp"
#include "robot_tasks/workflow/station_transport_workflow.hpp"
#include "robot_tasks/behavior/submit_order_behavior_tree.hpp"
#include "robot_tasks/routing/submit_order_router.hpp"
#include "robot_tasks/model/traffic_reservation.hpp"
#include "robot_tasks/workflow/workflow_control.hpp"
#include "std_srvs/srv/trigger.hpp"

using namespace std::chrono_literals;

using NavigateSequence = robot_interfaces_navigation::action::NavigateSequence;
using GoalHandleNavigateSequence = rclcpp_action::ClientGoalHandle<NavigateSequence>;
using ComputePathToPose = nav2_msgs::action::ComputePathToPose;

namespace {

geometry_msgs::msg::Quaternion QuaternionFromYaw(const double yaw) {
  geometry_msgs::msg::Quaternion q;
  q.z = std::sin(yaw * 0.5);
  q.w = std::cos(yaw * 0.5);
  return q;
}

geometry_msgs::msg::PoseStamped PoseStampedFromWaypoint(
    const robot_tasks::MissionWaypoint& waypoint, const std::string& frame_id,
    const rclcpp::Time& stamp) {
  geometry_msgs::msg::PoseStamped pose;
  pose.header.stamp = stamp;
  pose.header.frame_id = frame_id.empty() ? "map" : frame_id;
  pose.pose.position.x = waypoint.x;
  pose.pose.position.y = waypoint.y;
  pose.pose.orientation = QuaternionFromYaw(waypoint.yaw);
  return pose;
}

double PathLength(const nav_msgs::msg::Path& path) {
  double distance_m = 0.0;
  for (std::size_t i = 1; i < path.poses.size(); ++i) {
    const auto& previous = path.poses[i - 1].pose.position;
    const auto& current = path.poses[i].pose.position;
    distance_m += std::hypot(current.x - previous.x, current.y - previous.y);
  }
  return distance_m;
}

std::string SanitizeId(const std::string& value) {
  std::string result;
  result.reserve(value.size());
  for (const char ch : value) {
    const auto uch = static_cast<unsigned char>(ch);
    if (std::isalnum(uch) || ch == '_' || ch == '-') {
      result.push_back(ch);
    } else {
      result.push_back('_');
    }
  }
  return result;
}

robot_tasks::MissionActionResultCode ToMissionActionResultCode(
    const rclcpp_action::ResultCode code) {
  switch (code) {
    case rclcpp_action::ResultCode::SUCCEEDED:
      return robot_tasks::MissionActionResultCode::kSucceeded;
    case rclcpp_action::ResultCode::CANCELED:
      return robot_tasks::MissionActionResultCode::kCanceled;
    default:
      return robot_tasks::MissionActionResultCode::kFailed;
  }
}

}  // namespace

class MissionRunnerNode final : public rclcpp::Node {
 public:
#include "../impl/part_1.inc"  // lines 213-1398
#include "../impl/part_2.inc"  // lines 1399-2785
#include "../impl/part_3.inc"  // lines 2786-4084
#include "../impl/part_4.inc"  // lines 4085-5474
#include "../impl/part_5.inc"  // lines 5475-6216
};

int main(int argc, char** argv) {
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<MissionRunnerNode>());
  rclcpp::shutdown();
  return 0;
}
