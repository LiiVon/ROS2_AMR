/**
 * @file navigate_sequence_server_node.cpp
 * @brief （节点入口层）。
 * @details 所属包：`robot_tasks`；所在层次：节点入口层。
 */

#include <algorithm>
#include <atomic>
#include <chrono>
#include <cstdint>
#include <memory>
#include <mutex>
#include <string>
#include <thread>
#include <unordered_set>
#include <vector>

#include "geometry_msgs/msg/twist.hpp"
#include "nav2_msgs/action/follow_waypoints.hpp"
#include "nav2_msgs/action/navigate_to_pose.hpp"
#include "nav_msgs/msg/odometry.hpp"
#include "nav_msgs/msg/path.hpp"
#include "rclcpp/rclcpp.hpp"
#include "rclcpp_action/rclcpp_action.hpp"
#include "robot_interfaces_navigation/action/navigate_sequence.hpp"
#include "robot_interfaces_core/msg/navigation_statistics.hpp"
#include "robot_interfaces_core/msg/robot_state.hpp"
#include "robot_interfaces_core/msg/task_status.hpp"
#include "robot_interfaces_core/srv/get_statistics.hpp"
#include "std_srvs/srv/trigger.hpp"

using namespace std::chrono_literals;

using NavigateSequence = robot_interfaces_navigation::action::NavigateSequence;
using GoalHandleNavigateSequence = rclcpp_action::ServerGoalHandle<NavigateSequence>;
using FollowWaypoints = nav2_msgs::action::FollowWaypoints;
using GoalHandleFollowWaypoints = rclcpp_action::ClientGoalHandle<FollowWaypoints>;
using NavigateToPose = nav2_msgs::action::NavigateToPose;
using GoalHandleNavigateToPose = rclcpp_action::ClientGoalHandle<NavigateToPose>;

class NavigateSequenceServerNode final : public rclcpp::Node {
 public:
  NavigateSequenceServerNode() : Node("navigate_sequence_server_node") {
    use_nav2_action_ = declare_parameter<bool>("use_nav2_action", true);
    use_waypoint_follower_ = declare_parameter<bool>("use_waypoint_follower", true);
    // 默认关闭“无 Nav2 仿真降级”：调用方必须显式选择，避免任务假成功。
    simulate_without_nav2_ = declare_parameter<bool>("simulate_without_nav2", false);
    waypoint_follower_action_name_ =
        declare_parameter<std::string>("waypoint_follower_action_name", "/follow_waypoints");
    waypoint_follower_server_timeout_ms_ =
        declare_parameter<int>("waypoint_follower_server_timeout_ms", 1000);
    nav2_action_name_ = declare_parameter<std::string>("nav2_action_name", "/navigate_to_pose");
    nav2_server_timeout_ms_ = declare_parameter<int>("nav2_server_timeout_ms", 3000);
    simulated_motion_enabled_ = declare_parameter<bool>("simulated_motion_enabled", false);
    simulated_cmd_vel_topic_ =
        declare_parameter<std::string>("simulated_cmd_vel_topic", "/cmd_vel/virtual_rc");
    simulated_odom_topic_ =
        declare_parameter<std::string>("simulated_odom_topic", "/model/mobile_robot/odometry");
    simulated_goal_tolerance_m_ = declare_parameter<double>("simulated_goal_tolerance_m", 0.22);
    simulated_goal_timeout_s_ = declare_parameter<double>("simulated_goal_timeout_s", 45.0);
    simulated_max_linear_mps_ = declare_parameter<double>("simulated_max_linear_mps", 0.45);
    simulated_max_angular_radps_ = declare_parameter<double>("simulated_max_angular_radps", 1.0);
    current_goal_topic_ =
        declare_parameter<std::string>("current_goal_topic", "/navigation/current_goal");
    current_path_topic_ =
        declare_parameter<std::string>("current_path_topic", "/navigation/current_path");
    odom_topic_ = declare_parameter<std::string>("odom_topic", "/odom");

    waypoint_follower_client_ =
        rclcpp_action::create_client<FollowWaypoints>(this, waypoint_follower_action_name_);
    nav2_client_ = rclcpp_action::create_client<NavigateToPose>(this, nav2_action_name_);
    simulated_cmd_publisher_ = create_publisher<geometry_msgs::msg::Twist>(simulated_cmd_vel_topic_, 10);
    current_goal_publisher_ =
        create_publisher<geometry_msgs::msg::PoseStamped>(current_goal_topic_, rclcpp::QoS(1).transient_local());
    current_path_publisher_ =
        create_publisher<nav_msgs::msg::Path>(current_path_topic_, rclcpp::QoS(1).transient_local());
    simulated_odom_subscription_ = create_subscription<nav_msgs::msg::Odometry>(
        simulated_odom_topic_, 10, [this](nav_msgs::msg::Odometry::SharedPtr msg) {
          std::lock_guard<std::mutex> lock(simulated_pose_mutex_);
          simulated_x_ = msg->pose.pose.position.x;
          simulated_y_ = msg->pose.pose.position.y;
          simulated_yaw_ = YawFromQuaternion(msg->pose.pose.orientation);
          simulated_odom_received_ = true;
        });
    // 里程统计：任务执行期间累计位姿位移，供 NavigationStatistics.total_distance_m 使用。
    odom_subscription_ = create_subscription<nav_msgs::msg::Odometry>(
        odom_topic_, 10, [this](nav_msgs::msg::Odometry::SharedPtr msg) {
          if (!active_) {
            return;
          }
          AccumulateTravelDistance(msg->pose.pose.position.x, msg->pose.pose.position.y);
        });
    state_publisher_ = create_publisher<robot_interfaces_core::msg::RobotState>("/navigation/task_state", 10);
    task_publisher_ = create_publisher<robot_interfaces_core::msg::TaskStatus>("/navigation/task_status", 10);
    stats_service_ = create_service<robot_interfaces_core::srv::GetStatistics>(
        "/navigation/get_statistics",
        [this](const std::shared_ptr<robot_interfaces_core::srv::GetStatistics::Request>,
               std::shared_ptr<robot_interfaces_core::srv::GetStatistics::Response> response) {
          FillStatisticsResponse(response);
        });
    pause_service_ = create_service<std_srvs::srv::Trigger>(
        "/mission/pause",
        [this](const std::shared_ptr<std_srvs::srv::Trigger::Request>,
               std::shared_ptr<std_srvs::srv::Trigger::Response> response) {
          SetPaused(true, response);
        });
    resume_service_ = create_service<std_srvs::srv::Trigger>(
        "/mission/resume",
        [this](const std::shared_ptr<std_srvs::srv::Trigger::Request>,
               std::shared_ptr<std_srvs::srv::Trigger::Response> response) {
          SetPaused(false, response);
        });
    cancel_service_ = create_service<std_srvs::srv::Trigger>(
        "/mission/cancel",
        [this](const std::shared_ptr<std_srvs::srv::Trigger::Request>,
               std::shared_ptr<std_srvs::srv::Trigger::Response> response) {
          cancelled_ = true;
          response->success = true;
          response->message = "mission cancellation requested";
        });
    action_server_ = rclcpp_action::create_server<NavigateSequence>(
        this, "/mission/navigate_sequence",
        [this](const auto&, const auto goal) { return HandleGoal(goal); },
        [this](const auto) {
          cancelled_ = true;
          return rclcpp_action::CancelResponse::ACCEPT;
        },
        [this](const auto goal_handle) {
          std::thread{[this, goal_handle]() { Execute(goal_handle); }}.detach();
        });
    timer_ = create_wall_timer(200ms, [this]() { PublishState(); });
  }

 private:
  template <typename ResponseT>
  void SetPaused(const bool paused, const std::shared_ptr<ResponseT> response) {
    paused_ = paused;
    response->success = true;
    response->message = paused ? "mission paused" : "mission resumed";
  }

  template <typename ResponseT>
  void FillStatisticsResponse(const std::shared_ptr<ResponseT>& response) const {
    // 统计字段同时被 Execute 线程与执行器线程读写，统一加锁取值。
    std::lock_guard<std::mutex> lock(stats_mutex_);
    response->statistics.header = statistics_.header;
    response->statistics.total_goals = statistics_.total_goals;
    response->statistics.succeeded_goals = statistics_.succeeded_goals;
    response->statistics.failed_goals = statistics_.failed_goals;
    response->statistics.success_rate = statistics_.success_rate;
    response->statistics.total_distance_m = statistics_.total_distance_m;
    response->statistics.last_navigation_time_s = statistics_.last_navigation_time_s;
    response->statistics.mean_navigation_time_s = statistics_.mean_navigation_time_s;
    response->statistics.recovery_count = statistics_.recovery_count;
  }

  rclcpp_action::GoalResponse HandleGoal(
      const std::shared_ptr<const NavigateSequence::Goal> goal) {
    if (goal->goals.empty()) {
      RCLCPP_WARN(get_logger(), "reject empty navigate sequence goal");
      return rclcpp_action::GoalResponse::REJECT;
    }
    // 原子抢占 active_：HandleGoal 与 Execute 线程不同步，
    // 仅靠读取 active_ 会让两个 goal 同时通过检查。
    bool expected = false;
    if (!active_.compare_exchange_strong(expected, true)) {
      RCLCPP_WARN(get_logger(), "reject navigate sequence while another mission is active");
      return rclcpp_action::GoalResponse::REJECT;
    }
    return rclcpp_action::GoalResponse::ACCEPT_AND_EXECUTE;
  }

  void Execute(const std::shared_ptr<GoalHandleNavigateSequence> goal_handle) {
    const auto goal = goal_handle->get_goal();
    BeginMission(static_cast<int>(goal->goals.size()));
    PublishCurrentPath(goal->goals);
    const auto started = now();
    int succeeded = 0;
    int failed = 0;

    const bool waypoint_follower_available =
        use_waypoint_follower_ && waypoint_follower_client_->wait_for_action_server(
                                      std::chrono::milliseconds(
                                          waypoint_follower_server_timeout_ms_));
    if (use_waypoint_follower_ && !waypoint_follower_available) {
      RCLCPP_WARN(
          get_logger(), "Nav2 FollowWaypoints action server unavailable, falling back to %s",
          use_nav2_action_ ? "NavigateToPose sequence" : "simulated sequence");
    }
    const bool nav2_available =
        !waypoint_follower_available && use_nav2_action_ &&
        nav2_client_->wait_for_action_server(std::chrono::milliseconds(nav2_server_timeout_ms_));
    if (use_nav2_action_ && !nav2_available && !simulate_without_nav2_) {
      FinishFailure(goal_handle, "Nav2 action server is unavailable");
      return;
    }
    if (!waypoint_follower_available && use_nav2_action_ && !nav2_available) {
      RCLCPP_WARN(get_logger(), "Nav2 action server unavailable, using simulated sequence fallback");
    }

    if (waypoint_follower_available) {
      if (!RunWaypointFollowerSequence(goal->goals, goal_handle, succeeded, failed)) {
        FinishCancelled(goal_handle, succeeded, failed);
        return;
      }
    } else {
      for (int i = 0; rclcpp::ok() && i < TotalGoals(); ++i) {
        SetCurrentGoalIndex(i);
        PublishCurrentGoal(goal->goals[i]);
        if (goal_handle->is_canceling() || cancelled_) {
          FinishCancelled(goal_handle, succeeded, failed);
          return;
        }
        WaitWhilePaused(goal_handle, "paused before goal");
        const bool ok = nav2_available ? RunNav2Goal(goal->goals[i], goal_handle)
                                       : RunSimulatedGoal(goal->goals[i], goal_handle);
        if (ok) {
          ++succeeded;
          AddSucceededGoals(1);
        } else {
          ++failed;
          AddFailedGoals(1);
        }
      }
    }

    active_ = false;
    const double elapsed = (now() - started).seconds();
    {
      std::lock_guard<std::mutex> lock(stats_mutex_);
      statistics_.last_navigation_time_s = elapsed;
      accumulated_navigation_time_s_ += elapsed;
      ++navigation_sample_count_;
      statistics_.total_goals += total_goals_;
      statistics_.total_distance_m += mission_distance_m_;
    }
    UpdateStatistics();
    auto result = std::make_shared<NavigateSequence::Result>();
    result->success = failed == 0;
    result->message = failed == 0 ? "sequence finished" : "sequence finished with failures";
    result->succeeded_goals = succeeded;
    result->failed_goals = failed;
    result->state = failed == 0 ? "succeeded" : "failed";
    goal_handle->succeed(result);
  }

  void BeginMission(const int total_goals) {
    cancelled_ = false;
    {
      std::lock_guard<std::mutex> lock(stats_mutex_);
      current_goal_index_ = 0;
      total_goals_ = total_goals;
      mission_distance_m_ = 0.0;
      have_travel_reference_ = false;
      ++active_sequence_id_;
    }
    // active_ 已在 HandleGoal 中以 CAS 原子抢占，这里无需（也不应）重复置位。
    active_ = true;
  }

  void PublishCurrentGoal(geometry_msgs::msg::PoseStamped pose) {
    pose.header.stamp = now();
    if (pose.header.frame_id.empty()) {
      pose.header.frame_id = "map";
    }
    current_goal_publisher_->publish(pose);
  }

  void PublishCurrentPath(const std::vector<geometry_msgs::msg::PoseStamped>& goals) {
    nav_msgs::msg::Path path;
    path.header.stamp = now();
    path.header.frame_id = goals.empty() || goals.front().header.frame_id.empty()
                                ? "map"
                                : goals.front().header.frame_id;
    path.poses = goals;
    for (auto& pose : path.poses) {
      pose.header.stamp = path.header.stamp;
      if (pose.header.frame_id.empty()) {
        pose.header.frame_id = path.header.frame_id;
      }
    }
    current_path_publisher_->publish(path);
  }

  bool RunNav2Goal(const geometry_msgs::msg::PoseStamped& pose,
                   const std::shared_ptr<GoalHandleNavigateSequence>& sequence_handle) {
    NavigateToPose::Goal nav2_goal;
    nav2_goal.pose = pose;

    auto goal_options = rclcpp_action::Client<NavigateToPose>::SendGoalOptions();
    goal_options.feedback_callback =
        [this, sequence_handle](
            GoalHandleNavigateToPose::SharedPtr,
            const std::shared_ptr<const NavigateToPose::Feedback> feedback) {
          PublishSequenceFeedback(
              sequence_handle, "navigating",
              "distance remaining: " + std::to_string(feedback->distance_remaining));
        };

    auto goal_future = nav2_client_->async_send_goal(nav2_goal, goal_options);
    if (!WaitForFuture(goal_future, sequence_handle, "waiting for Nav2 goal acceptance")) {
      return false;
    }
    auto nav2_goal_handle = goal_future.get();
    if (!nav2_goal_handle) {
      PublishSequenceFeedback(sequence_handle, "failed", "Nav2 rejected goal");
      return false;
    }

    auto result_future = nav2_client_->async_get_result(nav2_goal_handle);
    while (rclcpp::ok()) {
      if (sequence_handle->is_canceling() || cancelled_) {
        nav2_client_->async_cancel_goal(nav2_goal_handle);
        return false;
      }
      WaitWhilePaused(sequence_handle, "paused during Nav2 goal");
      if (result_future.wait_for(100ms) == std::future_status::ready) {
        const auto wrapped = result_future.get();
        return wrapped.code == rclcpp_action::ResultCode::SUCCEEDED;
      }
      PublishSequenceFeedback(sequence_handle, "navigating", "Nav2 goal active");
    }
    return false;
  }

  bool RunWaypointFollowerSequence(
      const std::vector<geometry_msgs::msg::PoseStamped>& goals,
      const std::shared_ptr<GoalHandleNavigateSequence>& sequence_handle, int& succeeded,
      int& failed) {
    std::size_t start_index = 0;
    while (rclcpp::ok() && start_index < goals.size()) {
      if (sequence_handle->is_canceling() || cancelled_) {
        return false;
      }
      WaitWhilePaused(sequence_handle, "paused before waypoint follower goal");

      FollowWaypoints::Goal follower_goal;
      follower_goal.number_of_loops = 1;
      follower_goal.goal_index = 0;
      follower_goal.poses.assign(goals.begin() + static_cast<long>(start_index), goals.end());
      auto goal_options = rclcpp_action::Client<FollowWaypoints>::SendGoalOptions();
      goal_options.feedback_callback =
          [this, sequence_handle, start_index, &goals](
              GoalHandleFollowWaypoints::SharedPtr,
              const std::shared_ptr<const FollowWaypoints::Feedback> feedback) {
            const auto absolute_index =
                std::min(start_index + static_cast<std::size_t>(feedback->current_waypoint),
                         goals.empty() ? 0U : goals.size() - 1U);
            SetCurrentGoalIndex(static_cast<int>(absolute_index));
            PublishCurrentGoal(goals[absolute_index]);
            PublishSequenceFeedback(
                sequence_handle, "following_waypoints",
                "Nav2 FollowWaypoints active at waypoint " +
                    std::to_string(CurrentGoalIndex()));
          };

      auto goal_future = waypoint_follower_client_->async_send_goal(follower_goal, goal_options);
      if (!WaitForFuture(goal_future, sequence_handle, "waiting for Nav2 FollowWaypoints goal")) {
        return false;
      }
      auto follower_handle = goal_future.get();
      if (!follower_handle) {
        failed += static_cast<int>(goals.size() - start_index);
        AddFailedGoals(static_cast<int>(goals.size() - start_index));
        PublishSequenceFeedback(sequence_handle, "failed", "Nav2 FollowWaypoints rejected goal");
        return true;
      }

      auto result_future = waypoint_follower_client_->async_get_result(follower_handle);
      while (rclcpp::ok()) {
        if (sequence_handle->is_canceling() || cancelled_) {
          waypoint_follower_client_->async_cancel_goal(follower_handle);
          return false;
        }
        if (paused_) {
          waypoint_follower_client_->async_cancel_goal(follower_handle);
          // 必须等待取消真正落地：否则下一个目标会被 Nav2 以
          // “已有活动目标”拒绝，暂停+恢复会把剩余路点全判失败。
          const auto cancel_deadline = std::chrono::steady_clock::now() + std::chrono::seconds(5);
          while (rclcpp::ok() && std::chrono::steady_clock::now() < cancel_deadline) {
            if (result_future.wait_for(100ms) == std::future_status::ready) {
              break;
            }
          }
          WaitWhilePaused(sequence_handle, "paused during waypoint follower goal");
          start_index = static_cast<std::size_t>(std::max(0, CurrentGoalIndex()));
          break;
        }
        if (result_future.wait_for(100ms) == std::future_status::ready) {
          const auto wrapped = result_future.get();
          const auto remaining = goals.size() - start_index;
          if (wrapped.code != rclcpp_action::ResultCode::SUCCEEDED || !wrapped.result) {
            failed += static_cast<int>(remaining);
            AddFailedGoals(static_cast<int>(remaining));
            return true;
          }
          std::unordered_set<std::uint32_t> missed_indexes;
          for (const auto& missed : wrapped.result->missed_waypoints) {
            missed_indexes.insert(missed.index);
          }
          const auto missed_count =
              wrapped.result->error_code == FollowWaypoints::Result::NONE
                  ? static_cast<int>(std::min<std::size_t>(missed_indexes.size(), remaining))
                  : static_cast<int>(remaining);
          failed += missed_count;
          succeeded += static_cast<int>(remaining) - missed_count;
          AddFailedGoals(missed_count);
          AddSucceededGoals(static_cast<int>(remaining) - missed_count);
          return true;
        }
        PublishSequenceFeedback(
            sequence_handle, "following_waypoints", "Nav2 FollowWaypoints active");
      }
    }
    return true;
  }

  bool RunSimulatedGoal(
      const geometry_msgs::msg::PoseStamped& pose,
      const std::shared_ptr<GoalHandleNavigateSequence>& goal_handle) {
    if (simulated_motion_enabled_) {
      return RunSimulatedMotionGoal(pose, goal_handle);
    }
    // 没有运动能力的“仿真成功”是假成功：必须显式失败。
    RCLCPP_ERROR_THROTTLE(
        get_logger(), *get_clock(), 5000,
        "simulate_without_nav2 is enabled but simulated_motion_enabled is false; "
        "refusing to report navigation success without motion");
    PublishSequenceFeedback(goal_handle, "failed",
                            "simulated navigation unavailable: simulated_motion_enabled=false");
    return false;
  }

  bool RunSimulatedMotionGoal(
      const geometry_msgs::msg::PoseStamped& pose,
      const std::shared_ptr<GoalHandleNavigateSequence>& goal_handle) {
    const auto deadline = std::chrono::steady_clock::now() +
                          std::chrono::duration<double>(simulated_goal_timeout_s_);
    while (rclcpp::ok() && std::chrono::steady_clock::now() < deadline) {
      if (goal_handle->is_canceling() || cancelled_) {
        PublishZeroSimulatedCmd();
        return false;
      }
      WaitWhilePaused(goal_handle, "paused during simulated motion");
      const auto command = BuildSimulatedMotionCommand(pose);
      if (command.reached) {
        PublishZeroSimulatedCmd();
        return true;
      }
      simulated_cmd_publisher_->publish(command.twist);
      PublishSequenceFeedback(
          goal_handle, "navigating",
          "simulated motion distance remaining: " + std::to_string(command.distance_m));
      std::this_thread::sleep_for(50ms);
    }
    PublishZeroSimulatedCmd();
    PublishSequenceFeedback(goal_handle, "failed", "simulated motion timed out");
    return false;
  }

  struct SimulatedMotionCommand {
    geometry_msgs::msg::Twist twist;
    double distance_m = 0.0;
    bool reached = false;
  };

  SimulatedMotionCommand BuildSimulatedMotionCommand(
      const geometry_msgs::msg::PoseStamped& pose) const {
    double x = 0.0;
    double y = 0.0;
    double yaw = 0.0;
    bool has_odom = false;
    {
      std::lock_guard<std::mutex> lock(simulated_pose_mutex_);
      x = simulated_x_;
      y = simulated_y_;
      yaw = simulated_yaw_;
      has_odom = simulated_odom_received_;
    }
    SimulatedMotionCommand command;
    if (!has_odom) {
      return command;
    }
    const double dx = pose.pose.position.x - x;
    const double dy = pose.pose.position.y - y;
    command.distance_m = std::hypot(dx, dy);
    command.reached = command.distance_m <= simulated_goal_tolerance_m_;
    if (command.reached) {
      return command;
    }
    const double heading_error = NormalizeAngle(std::atan2(dy, dx) - yaw);
    const double linear_scale = std::max(0.0, std::cos(heading_error));
    command.twist.linear.x =
        std::clamp(command.distance_m * 0.8, 0.0, simulated_max_linear_mps_) * linear_scale;
    command.twist.angular.z =
        std::clamp(heading_error * 1.8, -simulated_max_angular_radps_, simulated_max_angular_radps_);
    return command;
  }

  template <typename FutureT>
  bool WaitForFuture(FutureT& future, const std::shared_ptr<GoalHandleNavigateSequence>& goal_handle,
                     const std::string& message,
                     const std::chrono::milliseconds timeout = std::chrono::seconds(5)) {
    // 等待必须有限：服务端不响应时不得让 Execute 线程永久挂起（active_ 无法释放，
    // 之后所有 goal 都会被拒）。
    const auto deadline = std::chrono::steady_clock::now() + timeout;
    while (rclcpp::ok()) {
      if (goal_handle->is_canceling() || cancelled_) {
        return false;
      }
      if (std::chrono::steady_clock::now() >= deadline) {
        PublishSequenceFeedback(goal_handle, "failed", message + " (timeout)");
        RCLCPP_ERROR(get_logger(), "%s: timed out after %ld ms", message.c_str(),
                     static_cast<long>(timeout.count()));
        return false;
      }
      WaitWhilePaused(goal_handle, message);
      if (future.wait_for(100ms) == std::future_status::ready) {
        return true;
      }
      PublishSequenceFeedback(goal_handle, "navigating", message);
    }
    return false;
  }

  void WaitWhilePaused(const std::shared_ptr<GoalHandleNavigateSequence>& goal_handle,
                       const std::string& message) {
    while (rclcpp::ok() && paused_ && !cancelled_ && !goal_handle->is_canceling()) {
      PublishZeroSimulatedCmd();
      PublishSequenceFeedback(goal_handle, "paused", message);
      std::this_thread::sleep_for(100ms);
    }
  }

  void PublishZeroSimulatedCmd() const {
    if (simulated_motion_enabled_) {
      geometry_msgs::msg::Twist zero;
      simulated_cmd_publisher_->publish(zero);
    }
  }

  static double NormalizeAngle(const double angle) {
    return std::atan2(std::sin(angle), std::cos(angle));
  }

  static double YawFromQuaternion(const geometry_msgs::msg::Quaternion& q) {
    const double siny_cosp = 2.0 * (q.w * q.z + q.x * q.y);
    const double cosy_cosp = 1.0 - 2.0 * (q.y * q.y + q.z * q.z);
    return std::atan2(siny_cosp, cosy_cosp);
  }

  void PublishSequenceFeedback(const std::shared_ptr<GoalHandleNavigateSequence>& goal_handle,
                               const std::string& state, const std::string& message) {
    auto feedback = std::make_shared<NavigateSequence::Feedback>();
    feedback->current_goal_index = CurrentGoalIndex();
    feedback->total_goals = TotalGoals();
    feedback->state = state;
    feedback->message = message;
    goal_handle->publish_feedback(feedback);
  }

  void FinishFailure(const std::shared_ptr<GoalHandleNavigateSequence>& goal_handle,
                     const std::string& message) {
    active_ = false;
    const int total = TotalGoals();
    AddFailedGoals(total);
    {
      std::lock_guard<std::mutex> lock(stats_mutex_);
      statistics_.total_goals += total;
      statistics_.total_distance_m += mission_distance_m_;
    }
    UpdateStatistics();
    auto result = std::make_shared<NavigateSequence::Result>();
    result->success = false;
    result->message = message;
    result->succeeded_goals = 0;
    result->failed_goals = total;
    result->state = "failed";
    goal_handle->abort(result);
  }

  void FinishCancelled(const std::shared_ptr<GoalHandleNavigateSequence>& goal_handle,
                       const int succeeded, const int failed) {
    active_ = false;
    // 取消只统计一次：剩余未完成路点计入失败，不再额外 +1（避免重复计数）。
    const int total = TotalGoals();
    const int cancelled_remaining = std::max(0, total - succeeded - failed);
    AddFailedGoals(cancelled_remaining);
    {
      std::lock_guard<std::mutex> lock(stats_mutex_);
      statistics_.total_goals += total;
      statistics_.total_distance_m += mission_distance_m_;
    }
    UpdateStatistics();
    auto result = std::make_shared<NavigateSequence::Result>();
    result->success = false;
    result->message = "sequence cancelled";
    result->succeeded_goals = succeeded;
    result->failed_goals = failed + cancelled_remaining;
    result->state = "cancelled";
    goal_handle->canceled(result);
  }

  int CurrentGoalIndex() const {
    std::lock_guard<std::mutex> lock(stats_mutex_);
    return current_goal_index_;
  }

  void SetCurrentGoalIndex(const int index) {
    std::lock_guard<std::mutex> lock(stats_mutex_);
    current_goal_index_ = index;
  }

  int TotalGoals() const {
    std::lock_guard<std::mutex> lock(stats_mutex_);
    return total_goals_;
  }

  void AddSucceededGoals(const int count) {
    std::lock_guard<std::mutex> lock(stats_mutex_);
    statistics_.succeeded_goals += count;
  }

  void AddFailedGoals(const int count) {
    std::lock_guard<std::mutex> lock(stats_mutex_);
    statistics_.failed_goals += count;
  }

  void AccumulateTravelDistance(double x, double y) {
    std::lock_guard<std::mutex> lock(stats_mutex_);
    if (have_travel_reference_) {
      mission_distance_m_ += std::hypot(x - travel_x_, y - travel_y_);
    }
    travel_x_ = x;
    travel_y_ = y;
    have_travel_reference_ = true;
  }

  void UpdateStatistics() {
    std::lock_guard<std::mutex> lock(stats_mutex_);
    const int finished = statistics_.succeeded_goals + statistics_.failed_goals;
    statistics_.header.stamp = now();
    statistics_.success_rate =
        finished == 0 ? 0.0 : static_cast<double>(statistics_.succeeded_goals) / finished;
    statistics_.mean_navigation_time_s =
        navigation_sample_count_ == 0
            ? 0.0
            : accumulated_navigation_time_s_ / static_cast<double>(navigation_sample_count_);
  }

  void PublishState() {
    robot_interfaces_core::msg::RobotState state;
    robot_interfaces_core::msg::TaskStatus task;
    {
      // 一次加锁取全部快照，避免定时器与 Execute 线程读到半新半旧的数据。
      std::lock_guard<std::mutex> lock(stats_mutex_);
      state.header.stamp = now();
      state.state = paused_ ? "paused" : (active_ ? "navigating" : "idle");
      state.previous_state = previous_state_;
      // active_goal_id 的语义是“任务标识”，空闲时必须为空；不能填路点下标。
      state.active_goal_id =
          active_ ? "sequence_" + std::to_string(active_sequence_id_) : "";
      state.message = active_ ? "mission active" : "waiting for mission";
      state.recoverable = true;

      task.header = state.header;
      task.state = state.state;
      task.current_goal_index = current_goal_index_;
      task.total_goals = total_goals_;
      task.succeeded_goals = statistics_.succeeded_goals;
      task.failed_goals = statistics_.failed_goals;
      task.message = state.message;
      task.paused = paused_;
      previous_state_ = state.state;
    }
    state_publisher_->publish(state);
    task_publisher_->publish(task);
  }

  bool use_nav2_action_;
  bool use_waypoint_follower_;
  bool simulate_without_nav2_;
  bool simulated_motion_enabled_;
  std::string nav2_action_name_;
  std::string waypoint_follower_action_name_;
  std::string simulated_cmd_vel_topic_;
  std::string simulated_odom_topic_;
  std::string current_goal_topic_;
  std::string current_path_topic_;
  std::string odom_topic_;
  int nav2_server_timeout_ms_;
  int waypoint_follower_server_timeout_ms_;
  double simulated_goal_tolerance_m_;
  double simulated_goal_timeout_s_;
  double simulated_max_linear_mps_;
  double simulated_max_angular_radps_;
  std::atomic_bool active_{false};
  std::atomic_bool paused_{false};
  std::atomic_bool cancelled_{false};
  // 以下共享字段由 stats_mutex_ 保护：Execute 在独立线程执行，
  // PublishState 定时器与服务回调在执行器线程执行。
  mutable std::mutex stats_mutex_;
  int current_goal_index_ = 0;
  int total_goals_ = 0;
  std::string previous_state_ = "initializing";
  robot_interfaces_core::msg::NavigationStatistics statistics_;
  double accumulated_navigation_time_s_ = 0.0;
  std::size_t navigation_sample_count_ = 0;
  double mission_distance_m_ = 0.0;
  double travel_x_ = 0.0;
  double travel_y_ = 0.0;
  bool have_travel_reference_ = false;
  std::size_t active_sequence_id_ = 0;
  rclcpp_action::Client<FollowWaypoints>::SharedPtr waypoint_follower_client_;
  rclcpp_action::Client<NavigateToPose>::SharedPtr nav2_client_;
  rclcpp_action::Server<NavigateSequence>::SharedPtr action_server_;
  rclcpp::Publisher<geometry_msgs::msg::Twist>::SharedPtr simulated_cmd_publisher_;
  rclcpp::Publisher<geometry_msgs::msg::PoseStamped>::SharedPtr current_goal_publisher_;
  rclcpp::Publisher<nav_msgs::msg::Path>::SharedPtr current_path_publisher_;
  rclcpp::Subscription<nav_msgs::msg::Odometry>::SharedPtr simulated_odom_subscription_;
  rclcpp::Subscription<nav_msgs::msg::Odometry>::SharedPtr odom_subscription_;
  rclcpp::Publisher<robot_interfaces_core::msg::RobotState>::SharedPtr state_publisher_;
  rclcpp::Publisher<robot_interfaces_core::msg::TaskStatus>::SharedPtr task_publisher_;
  rclcpp::Service<robot_interfaces_core::srv::GetStatistics>::SharedPtr stats_service_;
  rclcpp::Service<std_srvs::srv::Trigger>::SharedPtr pause_service_;
  rclcpp::Service<std_srvs::srv::Trigger>::SharedPtr resume_service_;
  rclcpp::Service<std_srvs::srv::Trigger>::SharedPtr cancel_service_;
  rclcpp::TimerBase::SharedPtr timer_;
  mutable std::mutex simulated_pose_mutex_;
  double simulated_x_ = 0.0;
  double simulated_y_ = 0.0;
  double simulated_yaw_ = 0.0;
  bool simulated_odom_received_ = false;
};

int main(int argc, char** argv) {
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<NavigateSequenceServerNode>());
  rclcpp::shutdown();
  return 0;
}
