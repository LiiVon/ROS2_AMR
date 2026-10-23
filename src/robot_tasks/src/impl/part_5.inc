/**
 * @file part_5.inc
 * @brief （实现细节）。
 * @details 所属包：`robot_tasks`；所在层次：实现细节。
 * @note 教学说明见 `docs/教学/` 对应章节。
 */

  void SendMissionGoal(const NavigateSequence::Goal& goal,
                       const std::shared_ptr<std_srvs::srv::Trigger::Response>& response,
                       const std::string& accepted_message) {
    auto options = rclcpp_action::Client<NavigateSequence>::SendGoalOptions();
    options.goal_response_callback = [this](GoalHandleNavigateSequence::SharedPtr goal_handle) {
      if (!goal_handle) {
        mission_active_ = false;
        current_goal_handle_.reset();
        SetState("ERROR", "navigate sequence rejected mission profile", true);
        return;
      }
      current_goal_handle_ = goal_handle;
      SetState("RUNNING", "mission accepted by navigate sequence", true);
    };
    options.result_callback = [this](const GoalHandleNavigateSequence::WrappedResult& result) {
      const auto dock_profile = LoadDockProfile(nullptr);
      const std::string completed_mission_id = profile_.has_value() ? profile_->mission_id : "";
      const bool completed_dock =
          !completed_mission_id.empty() &&
          ((dock_profile.has_value() && completed_mission_id == dock_profile->mission_id) ||
           dock_by_mission_.find(completed_mission_id) != dock_by_mission_.end());
      mission_active_ = false;
      paused_ = false;
      current_goal_handle_.reset();
      robot_tasks::MissionResultWorkflowInput result_input;
      result_input.code = ToMissionActionResultCode(result.code);
      result_input.result_success = result.result && result.result->success;
      result_input.result_message = result.result ? result.result->message : "";
      result_input.cancellation_requested = cancellation_requested_;
      result_input.completed_dock = completed_dock;
      const auto mission_result = robot_tasks::TickMissionResultBehaviorTree(result_input);
      if (!mission_result.success) {
        SetState("ERROR", "mission result behavior tree failed", true);
        return;
      }
      const auto& result_decision = mission_result.decision;

      if (!ApplyMissionResultDecision(
              result_decision, completed_mission_id, completed_dock)) {
        return;
      }
      ApplyMissionResultPostActions(result_decision, completed_dock, dock_profile);
    };
    options.feedback_callback =
        [this](
            GoalHandleNavigateSequence::SharedPtr,
            const std::shared_ptr<const NavigateSequence::Feedback> feedback) {
          current_goal_index_ = feedback->current_goal_index;
          if (feedback->state == "paused") {
            paused_ = true;
            SetState("PAUSED", feedback->message, true);
          } else if (paused_) {
            SetState("PAUSED", feedback->state + ": " + feedback->message, true);
          } else {
            SetState("RUNNING", feedback->state + ": " + feedback->message, true);
          }
        };

    mission_active_ = true;
    autostart_sent_ = true;
    SetState("STARTING", "sending mission profile to navigate sequence", true);
    action_client_->async_send_goal(goal, options);
    if (response != nullptr) {
      response->success = true;
      response->message = accepted_message;
    }
  }

  bool ApplyMissionResultDecision(
      const robot_tasks::MissionResultWorkflowDecision& result_decision,
      const std::string& completed_mission_id, const bool completed_dock) {
    if (result_decision.clear_cancellation_requested) {
      cancellation_requested_ = false;
    }
    if (result_decision.mark_docking_terminal) {
      MarkDockingMissionTerminal(
          completed_mission_id, result_decision.docking_terminal_state,
          result_decision.docking_terminal_message);
    }
    if (result_decision.release_all_resources) {
      ReleaseAllResourcesForMission(completed_mission_id);
    } else if (result_decision.release_route_locks) {
      ReleaseRouteLocksForMission(completed_mission_id);
    }

    if (result_decision.probe_docking_contact_failure) {
      if (HandleDockingContactFailure(completed_mission_id)) {
        HandleMissionFailure("docking contact failed", false);
        return false;
      }
    }
    if (result_decision.reset_failure_retry_count) {
      current_failure_retry_count_ = 0;
    }
    if (result_decision.mark_resource_occupied) {
      MarkResourceOccupiedForMission(completed_mission_id);
    }
    if (result_decision.complete_docking) {
      CompleteDockingMission(completed_mission_id);
    }

    if (result_decision.handle_failure) {
      HandleMissionFailure(result_decision.failure_message, completed_dock);
    } else {
      SetState(result_decision.state, result_decision.state_message, result_decision.recoverable);
    }

    return true;
  }

  void ApplyMissionResultPostActions(
      const robot_tasks::MissionResultWorkflowDecision& result_decision,
      const bool completed_dock,
      const std::optional<robot_tasks::MissionProfile>& dock_profile) {
    robot_tasks::MissionResultPostActionRequest request;
    request.mission_succeeded = result_decision.mission_succeeded;
    request.return_to_dock_after_mission = return_to_dock_after_mission_;
    request.completed_dock = completed_dock;
    request.mission_queue_empty = mission_queue_.empty();
    request.dock_profile_available = dock_profile.has_value();
    request.existing_queue_start_requested = queue_start_requested_;
    request.auto_start_queue = auto_start_queue_;
    request.queue_running = queue_running_;
    const auto post_actions = robot_tasks::PlanMissionResultPostActions(request);
    if (post_actions.queue_dock_after_result) {
      QueueDockMission(*dock_profile, "mission queue completed");
    }
    queue_start_requested_ = post_actions.queue.queue_start_requested;
    queue_running_ = post_actions.queue.queue_running;
  }

  void PauseMission(
      const std::shared_ptr<std_srvs::srv::Trigger::Response>& response) {
    if (!mission_active_) {
      response->success = false;
      response->message = "no active mission profile to pause";
      return;
    }
    paused_ = true;
    SetState("PAUSED", "mission profile pause requested", true);
    SendControlRequest<std_srvs::srv::Trigger>(pause_client_, pause_service_name_);
    response->success = true;
    response->message = "mission profile pause requested";
  }

  void ResumeMission(
      const std::shared_ptr<std_srvs::srv::Trigger::Response>& response) {
    if (!mission_active_ || !paused_) {
      response->success = false;
      response->message = "no paused mission profile to resume";
      return;
    }
    paused_ = false;
    SetState("RUNNING", "mission profile resume requested", true);
    SendControlRequest<std_srvs::srv::Trigger>(resume_client_, resume_service_name_);
    response->success = true;
    response->message = "mission profile resume requested";
  }

  void CancelMission(
      const std::shared_ptr<std_srvs::srv::Trigger::Response>& response) {
    if (!RunnerBusy()) {
      response->success = false;
      response->message = "no active mission profile to cancel";
      return;
    }
    RequestActiveMissionCancel("mission profile cancellation requested");
    response->success = true;
    response->message = "mission profile cancellation requested";
  }

  template <typename ResponseT>
  void GetMissionRecoveryState(const std::shared_ptr<ResponseT>& response) const {
    response->success = true;
    response->message = "mission recovery state";
    response->enabled = failure_recovery_enabled_;
    response->policy = failure_recovery_policy_;
    response->retry_limit = std::max(0, failure_retry_limit_);
    response->retry_count = current_failure_retry_count_;
    response->last_failed_mission_id = last_failed_mission_id_;
    response->last_failure_message = last_failure_message_;
    response->last_recovery_action = last_recovery_action_;
  }

  template <typename RequestT, typename ResponseT>
  void ListMissionEvents(
      const std::shared_ptr<RequestT>& request,
      const std::shared_ptr<ResponseT>& response) const {
    const auto limit = robot_tasks::NormalizeEventLimit(request->limit, 20U, 200U);
    const auto events = robot_tasks::SelectRecentMissionEvents(
        mission_events_, limit, request->state_filter, request->mission_id_filter);
    for (const auto& event : events) {
      response->stamp.push_back(event.stamp);
      response->mission_ids.push_back(event.mission_id);
      response->states.push_back(event.state);
      response->previous_states.push_back(event.previous_state);
      response->messages.push_back(event.message);
      response->recoverable.push_back(event.recoverable);
    }
    response->success = true;
    response->message = "loaded " + std::to_string(events.size()) + " mission event(s)";
  }

  void ResetMissionRecoveryState(
      const std::shared_ptr<std_srvs::srv::Trigger::Response>& response) {
    const auto* transport = FindActiveStationTransport();
    if (transport != nullptr &&
        transport->state.phase ==
            robot_tasks::StationTransportPhase::kNeedsOperator) {
      response->success = false;
      response->message =
          "clear the physical payload with /facility/set_payload_loaded before resetting recovery";
      return;
    }
    current_failure_retry_count_ = 0;
    active_recovery_mission_id_.clear();
    last_failed_mission_id_.clear();
    last_failure_message_.clear();
    last_recovery_action_.clear();
    if (current_state_ == "NEEDS_OPERATOR") {
      SetState("READY", "mission recovery state reset", true);
    }
    response->success = true;
    response->message = "mission recovery state reset";
  }

  void MarkResourceOccupiedForMission(const std::string& mission_id) {
    robot_tasks::MarkResourceOccupiedForMission(
        facility_reservations_, facility_resource_by_mission_, mission_id);
  }

  void ReleaseResourceForMission(const std::string& mission_id) {
    robot_tasks::ReleaseResourceForMission(
        facility_reservations_, facility_resource_by_mission_, mission_id);
  }

  void ReleaseAllResourcesForMission(const std::string& mission_id) {
    robot_tasks::TickMissionResourceCleanupBehaviorTree(
        robot_tasks::MissionResourceCleanupBehaviorTreeInput{
            facility_reservations_, facility_resource_by_mission_, route_locks_,
            route_locks_by_mission_, mission_id});
  }

  void CompleteDockingMission(const std::string& mission_id) {
    const auto transition = robot_tasks::CompleteDockingSubflow(
        dock_states_, dock_by_mission_, facility_reservations_, facility_resource_by_mission_,
        mission_id);
    if (!transition.changed) {
      return;
    }
    RecordMissionEventFor(
        mission_id, transition.event_state, transition.previous_state,
        transition.event_message, transition.event_recoverable);
  }

  bool HandleDockingContactFailure(const std::string& mission_id) {
    const auto transition = robot_tasks::StageDockingContactRetry(
        dock_states_, dock_by_mission_, facility_reservations_, facility_resource_by_mission_,
        mission_id);
    if (!transition.changed) {
      return false;
    }
    RecordMissionEventFor(
        mission_id, transition.event_state, transition.previous_state,
        transition.event_message, transition.event_recoverable);
    return true;
  }

  void MarkDockingMissionTerminal(
      const std::string& mission_id, const std::string& state, const std::string& message) {
    const auto transition = robot_tasks::MarkDockingSubflowTerminal(
        dock_states_, dock_by_mission_, mission_id, state, message);
    if (!transition.changed) {
      return;
    }
    RecordMissionEventFor(
        mission_id, transition.event_state, transition.previous_state,
        transition.event_message, transition.event_recoverable);
  }

  void HandleMissionFailure(const std::string& failure_message, const bool completed_dock) {
    const std::string mission_id = profile_.has_value() ? profile_->mission_id : "";
    last_failed_mission_id_ = mission_id;
    last_failure_message_ = failure_message;

    robot_tasks::MissionRecoveryConfig config;
    config.enabled = failure_recovery_enabled_;
    config.policy = failure_recovery_policy_;
    config.retry_limit = std::max(0, failure_retry_limit_);

    robot_tasks::MissionRecoveryBehaviorTreeInput input;
    input.config = config;
    input.retry_count = current_failure_retry_count_;
    input.is_dock_mission = completed_dock;
    input.mission_id = mission_id;
    input.failure_message = failure_message;
    input.mission_file = mission_file_;
    input.recovery_priority = failure_recovery_priority_;
    input.next_queue_sequence = next_queue_sequence_;
    if (profile_.has_value()) {
      input.active_profile = *profile_;
    }

    const auto recovery = robot_tasks::TickMissionRecoveryBehaviorTree(input);
    current_failure_retry_count_ = recovery.retry_count;
    next_queue_sequence_ = recovery.next_queue_sequence;
    last_recovery_action_ = recovery.decision.action;

    if (recovery.retry_mission.has_value()) {
      mission_queue_.push_back(*recovery.retry_mission);
      SortMissionQueue();
      queue_running_ = recovery.queue_running;
      queue_start_requested_ = recovery.queue_start_requested;
      SetState(recovery.state, recovery.state_message, recovery.recoverable);
      return;
    }
    if (recovery.request_dock_return) {
      SetState(recovery.state, recovery.state_message, recovery.recoverable);
      RequestDockReturn("mission failure recovery for " + mission_id, nullptr);
      return;
    }
    if (recovery.stop_queue) {
      queue_running_ = false;
      queue_start_requested_ = false;
      SetState(recovery.state, recovery.state_message, recovery.recoverable);
      return;
    }
    SetState(recovery.state, recovery.state_message, recovery.recoverable);
  }

  void RequestActiveMissionCancel(const std::string& message) {
    auto* transport = FindActiveStationTransport();
    if (transport != nullptr && !mission_active_) {
      const std::string mission_id = transport->state.mission_id;
      const auto transition =
          robot_tasks::PlanStationTransportCancel(transport->state);
      if (!transition.accepted) {
        SetState("ERROR", transition.message, false);
        return;
      }
      transport->state = transition.next_state;
      if (transition.release_resources) {
        ReleaseAllResourcesForMission(mission_id);
        FinishStationTransportTerminal(
            mission_id, "CANCELED", transition.message, false);
      } else {
        SetState("NEEDS_OPERATOR", transition.message, false);
      }
      return;
    }

    cancellation_requested_ = true;
    paused_ = false;
    SetState("CANCELING", message, true);
    if (current_goal_handle_) {
      action_client_->async_cancel_goal(current_goal_handle_);
    }
    SendControlRequest<std_srvs::srv::Trigger>(cancel_client_, cancel_service_name_);
  }

  template <typename ServiceT>
  void SendControlRequest(const typename rclcpp::Client<ServiceT>::SharedPtr& client,
                          const std::string& service_name) {
    if (!client->service_is_ready()) {
      RCLCPP_WARN(get_logger(), "control service %s is not ready", service_name.c_str());
      return;
    }
    client->async_send_request(std::make_shared<typename ServiceT::Request>());
  }

  void Tick() {
    robot_tasks::MissionRuntimeBehaviorTreeInput input;
    input.tick.autostart = autostart_;
    input.tick.autostart_sent = autostart_sent_;
    input.tick.mission_active = RunnerBusy();
    input.tick.queue_start_requested = queue_start_requested_;
    const auto runtime_tick = robot_tasks::TickMissionRuntimeBehaviorTree(input);
    if (!runtime_tick.success) {
      RCLCPP_WARN_THROTTLE(
          get_logger(), *get_clock(), 5000, "mission runtime behavior tree failed");
    }
    if (runtime_tick.success && runtime_tick.decision.start_default_mission) {
      StartMission(nullptr);
    }
    if (runtime_tick.success && runtime_tick.decision.start_queued_mission &&
        !RunnerBusy()) {
      if (runtime_tick.decision.clear_queue_start_request) {
        queue_start_requested_ = false;
      }
      StartNextQueuedMission(nullptr);
    }
    PublishState();
    PublishPayloadState();
    PublishLocalizationHealth();
    PublishSafetyState();
    ExpireStationConfirmations();
  }

  void SetState(const std::string& state, const std::string& message, const bool recoverable) {
    previous_state_ = current_state_;
    current_state_ = state;
    last_message_ = message;
    recoverable_ = recoverable;
    RecordMissionEvent();
    PublishState();
  }

  void RecordMissionEventFor(const std::string& mission_id, const std::string& state,
                             const std::string& previous_state, const std::string& message,
                             const bool recoverable) {
    robot_tasks::AppendMissionEvent(
        mission_events_,
        robot_tasks::MakeMissionEvent(
            std::to_string(now().seconds()), mission_id, state, previous_state, message,
            recoverable),
        500U);
  }

  void RecordMissionEvent() {
    robot_tasks::AppendMissionEvent(
        mission_events_,
        robot_tasks::MakeMissionEvent(
            std::to_string(now().seconds()),
            profile_.has_value() ? profile_->mission_id : "", current_state_, previous_state_,
            last_message_, recoverable_),
        500U);
  }

  void PublishState() {
    robot_interfaces_core::msg::RobotState state;
    state.header.stamp = now();
    state.state = current_state_;
    state.previous_state = previous_state_;
    state.active_goal_id = profile_.has_value() ? profile_->mission_id : "";
    state.message = last_message_;
    state.recoverable = recoverable_;
    state_publisher_->publish(state);
  }

  void PublishPayloadState() {
    robot_interfaces_facility::msg::PayloadState state;
    state.header.stamp = now();
    state.loaded = payload_loaded_;
    state.payload_id = payload_id_;
    state.weight_kg = payload_weight_kg_;
    state.capacity_kg = payload_capacity_kg_;
    state.top_module_type = top_module_type_;
    state.state = payload_state_;
    state.last_action = payload_last_action_;
    state.message = payload_message_;
    payload_state_publisher_->publish(state);
  }

  void PublishLocalizationHealth() {
    robot_interfaces_navigation::msg::LocalizationHealth state;
    state.header.stamp = now();
    state.localized = localization_localized_;
    state.state = localization_state_;
    state.covariance_xy = localization_covariance_xy_;
    state.covariance_threshold = localization_covariance_threshold_;
    state.station_id = localization_station_id_;
    state.message = localization_message_;
    localization_health_publisher_->publish(state);
  }

  void PublishSafetyState() {
    robot_interfaces_navigation::msg::SafetyState state;
    state.header.stamp = now();
    state.obstacle_blocked = safety_obstacle_blocked_;
    state.safety_stop = safety_obstacle_blocked_;
    state.state = SafetyStateName();
    state.runtime_speed_limit_mps = safety_runtime_speed_limit_mps_;
    state.zone_speed_limit_mps = safety_zone_speed_limit_mps_;
    state.effective_speed_limit_mps = EffectiveSafetySpeedLimit();
    state.stop_reason = safety_stop_reason_;
    state.blockage_id = safety_blockage_id_;
    state.mission_id = safety_mission_id_;
    state.message = safety_message_;
    safety_state_publisher_->publish(state);
  }

  std::string mission_file_;
  std::string dock_mission_file_;
  std::string station_file_;
  std::string facility_file_;
  std::string dock_file_;
  std::string fleet_file_;
  std::string scenario_file_;
  std::string business_order_file_;
  std::string robot_id_;
  std::string top_module_type_;
  std::string preflight_zones_file_;
  std::string preflight_map_name_;
  std::string action_name_;
  std::string pause_service_name_;
  std::string resume_service_name_;
  std::string cancel_service_name_;
  bool auto_start_queue_ = true;
  bool return_to_dock_after_mission_ = false;
  bool return_to_dock_on_low_battery_ = true;
  bool autostart_ = false;
  bool autostart_sent_ = false;
  bool mission_active_ = false;
  bool paused_ = false;
  bool cancellation_requested_ = false;
  bool queue_running_ = false;
  bool queue_start_requested_ = false;
  bool low_battery_dock_request_sent_ = false;
  bool recoverable_ = true;
  bool failure_recovery_enabled_ = true;
  bool preflight_enabled_ = true;
  bool preflight_require_station_route_ = true;
  bool estimate_use_nav2_path_ = true;
  bool opportunity_charge_require_idle_queue_ = true;
  int current_goal_index_ = 0;
  int dock_priority_ = 100;
  int server_timeout_ms_ = 5000;
  int estimate_nav2_server_timeout_ms_ = 50;
  int estimate_nav2_result_timeout_ms_ = 1500;
  int failure_retry_limit_ = 1;
  int failure_recovery_priority_ = 90;
  int opportunity_charge_priority_ = 75;
  double dock_low_battery_voltage_ = 22.0;
  double opportunity_charge_threshold_voltage_ = 23.2;
  double critical_charge_threshold_voltage_ = 22.0;
  double estimate_nominal_speed_mps_ = 0.35;
  double estimate_battery_drop_per_meter_ = 0.015;
  double estimate_min_battery_voltage_ = 22.0;
  double preflight_default_speed_limit_mps_ = 0.6;
  double current_battery_voltage_ = 0.0;
  double payload_capacity_kg_ = 30.0;
  double payload_weight_kg_ = 0.0;
  double localization_covariance_threshold_ = 0.5;
  double localization_covariance_xy_ = 0.0;
  double safety_runtime_speed_limit_mps_ = 0.0;
  double safety_zone_speed_limit_mps_ = 0.6;
  std::uint64_t next_queue_sequence_ = 0;
  std::uint64_t next_transport_order_sequence_ = 0;
  std::uint64_t next_station_order_sequence_ = 0;
  std::uint64_t next_station_sequence_order_sequence_ = 0;
  std::uint64_t next_charging_request_sequence_ = 0;
  std::uint64_t next_docking_request_sequence_ = 0;
  std::uint64_t next_fleet_task_sequence_ = 0;
  std::string current_state_ = "INITIALIZING";
  std::string previous_state_ = "UNKNOWN";
  std::string last_message_ = "initializing";
  std::string failure_recovery_policy_ = "retry_then_dock";
  std::string estimate_nav2_compute_path_action_ = "/compute_path_to_pose";
  std::string estimate_nav2_planner_id_;
  std::string active_recovery_mission_id_;
  std::string last_failed_mission_id_;
  std::string last_failure_message_;
  std::string last_recovery_action_;
  std::string payload_id_;
  std::string payload_state_ = "EMPTY";
  std::string payload_last_action_ = "none";
  std::string payload_message_ = "payload empty";
  std::string localization_state_ = "UNKNOWN";
  std::string localization_station_id_;
  std::string localization_message_ = "waiting for localization";
  std::string safety_stop_reason_;
  std::string safety_blockage_id_;
  std::string safety_mission_id_;
  std::string safety_message_ = "safety ok";
  int current_failure_retry_count_ = 0;
  bool payload_loaded_ = false;
  bool localization_localized_ = false;
  bool localization_pause_on_lost_ = true;
  bool localization_paused_mission_ = false;
  bool safety_obstacle_blocked_ = false;
  bool safety_obstacle_pause_on_blockage_ = true;
  bool safety_blockage_paused_mission_ = false;
  bool traffic_intersection_locks_enabled_ = true;
  std::optional<robot_tasks::MissionProfile> profile_;
  std::vector<QueuedMission> mission_queue_;
  std::unordered_map<std::string, StationTransportExecutionRecord>
      station_transport_executions_;
  std::string active_station_transport_id_;
  std::vector<robot_tasks::MissionEvent> mission_events_;
  std::unordered_set<std::string> paused_workflow_orders_;
  std::unordered_map<std::string, robot_tasks::RuntimeFleetRobotState> runtime_fleet_state_;
  std::unordered_map<std::string, RemoteTaskState> remote_tasks_;
  robot_tasks::FacilityReservationMap facility_reservations_;
  robot_tasks::MissionResourceMap facility_resource_by_mission_;
  std::unordered_map<std::string, StationConfirmation> station_confirmations_;
  std::unordered_map<std::string, FulfillmentOrder> fulfillment_orders_;
  robot_tasks::DockingStateMap dock_states_;
  robot_tasks::DockingMissionMap dock_by_mission_;
  robot_tasks::RouteLockMap route_locks_;
  robot_tasks::RouteLockReservationMap route_locks_by_mission_;
  GoalHandleNavigateSequence::SharedPtr current_goal_handle_;
  rclcpp_action::Client<NavigateSequence>::SharedPtr action_client_;
  rclcpp::Node::SharedPtr nav2_path_node_;
  rclcpp_action::Client<ComputePathToPose>::SharedPtr nav2_path_client_;
  mutable std::mutex nav2_path_mutex_;
  rclcpp::Client<std_srvs::srv::Trigger>::SharedPtr pause_client_;
  rclcpp::Client<std_srvs::srv::Trigger>::SharedPtr resume_client_;
  rclcpp::Client<std_srvs::srv::Trigger>::SharedPtr cancel_client_;
  rclcpp::Publisher<robot_interfaces_core::msg::RobotState>::SharedPtr state_publisher_;
  rclcpp::Publisher<robot_interfaces_facility::msg::PayloadState>::SharedPtr payload_state_publisher_;
  rclcpp::Publisher<robot_interfaces_navigation::msg::LocalizationHealth>::SharedPtr
      localization_health_publisher_;
  rclcpp::Publisher<robot_interfaces_navigation::msg::SafetyState>::SharedPtr safety_state_publisher_;
  rclcpp::Publisher<geometry_msgs::msg::PoseWithCovarianceStamped>::SharedPtr initial_pose_publisher_;
  rclcpp::Subscription<robot_interfaces_core::msg::ChassisState>::SharedPtr chassis_subscription_;
  rclcpp::Subscription<geometry_msgs::msg::PoseWithCovarianceStamped>::SharedPtr amcl_pose_subscription_;
  rclcpp::Service<std_srvs::srv::Trigger>::SharedPtr start_service_;
  rclcpp::Service<std_srvs::srv::Trigger>::SharedPtr reload_service_;
  rclcpp::Service<std_srvs::srv::Trigger>::SharedPtr pause_service_;
  rclcpp::Service<std_srvs::srv::Trigger>::SharedPtr resume_service_;
  rclcpp::Service<std_srvs::srv::Trigger>::SharedPtr cancel_service_;
  rclcpp::Service<robot_interfaces_mission::srv::GetMissionRecoveryState>::SharedPtr
      recovery_state_service_;
  rclcpp::Service<robot_interfaces_mission::srv::ListMissionEvents>::SharedPtr
      list_mission_events_service_;
  rclcpp::Service<std_srvs::srv::Trigger>::SharedPtr reset_recovery_service_;
  rclcpp::Service<robot_interfaces_mission::srv::EnqueueMission>::SharedPtr enqueue_service_;
  rclcpp::Service<robot_interfaces_mission::srv::CancelQueuedMission>::SharedPtr
      cancel_queued_service_;
  rclcpp::Service<robot_interfaces_business::srv::CancelWorkflow>::SharedPtr
      cancel_workflow_service_;
  rclcpp::Service<robot_interfaces_business::srv::PauseWorkflow>::SharedPtr
      pause_workflow_service_;
  rclcpp::Service<robot_interfaces_business::srv::ResumeWorkflow>::SharedPtr
      resume_workflow_service_;
  rclcpp::Service<robot_interfaces_business::srv::GetWorkflowStatus>::SharedPtr
      get_workflow_status_service_;
  rclcpp::Service<robot_interfaces_mission::srv::ReprioritizeQueuedMission>::SharedPtr
      reprioritize_queued_mission_service_;
  rclcpp::Service<robot_interfaces_mission::srv::ReprioritizeBatch>::SharedPtr
      reprioritize_batch_service_;
  rclcpp::Service<robot_interfaces_mission::srv::PreemptMission>::SharedPtr preempt_service_;
  rclcpp::Service<robot_interfaces_mission::srv::EstimateMissionCost>::SharedPtr
      estimate_mission_service_;
  rclcpp::Service<robot_interfaces_business::srv::SubmitOrder>::SharedPtr
      submit_unified_order_service_;
  rclcpp::Service<robot_interfaces_mission::srv::ListStations>::SharedPtr list_stations_service_;
  rclcpp::Service<robot_interfaces_mission::srv::ListStationRoutes>::SharedPtr
      list_station_routes_service_;
  rclcpp::Service<robot_interfaces_mission::srv::BlockStationRoute>::SharedPtr
      block_station_route_service_;
  rclcpp::Service<robot_interfaces_mission::srv::UnblockStationRoute>::SharedPtr
      unblock_station_route_service_;
  rclcpp::Service<robot_interfaces_mission::srv::ListTrafficReservations>::SharedPtr
      list_traffic_reservations_service_;
  rclcpp::Service<robot_interfaces_mission::srv::ClearTrafficReservation>::SharedPtr
      clear_traffic_reservation_service_;
  rclcpp::Service<robot_interfaces_mission::srv::DetectTrafficDeadlock>::SharedPtr
      detect_traffic_deadlock_service_;
  rclcpp::Service<robot_interfaces_fleet::srv::ListFleetRobots>::SharedPtr
      list_fleet_robots_service_;
  rclcpp::Service<robot_interfaces_fleet::srv::UpdateFleetRobotState>::SharedPtr
      update_fleet_robot_state_service_;
  rclcpp::Service<robot_interfaces_fleet::srv::UpdateRemoteTaskState>::SharedPtr
      update_remote_task_state_service_;
  rclcpp::Service<robot_interfaces_fleet::srv::PreviewFleetStationTask>::SharedPtr
      preview_fleet_station_task_service_;
  rclcpp::Service<robot_interfaces_fleet::srv::SubmitFleetStationTask>::SharedPtr
      submit_fleet_station_task_service_;
  rclcpp::Service<robot_interfaces_business::srv::ListBusinessOrderTypes>::SharedPtr
      list_business_order_types_service_;
  rclcpp::Service<robot_interfaces_business::srv::GetOperatorSnapshot>::SharedPtr
      get_operator_snapshot_service_;
  rclcpp::Service<robot_interfaces_business::srv::ListScenarioTasks>::SharedPtr
      list_scenario_tasks_service_;
  rclcpp::Service<robot_interfaces_business::srv::ListScenarioWorkflows>::SharedPtr
      list_scenario_workflows_service_;
  rclcpp::Service<robot_interfaces_facility::srv::ListFacilityResources>::SharedPtr
      list_facility_resources_service_;
  rclcpp::Service<robot_interfaces_facility::srv::ReserveFacilityResource>::SharedPtr
      reserve_facility_resource_service_;
  rclcpp::Service<robot_interfaces_facility::srv::ReleaseFacilityResource>::SharedPtr
      release_facility_resource_service_;
  rclcpp::Service<robot_interfaces_facility::srv::RequestLiftSession>::SharedPtr
      request_lift_session_service_;
  rclcpp::Service<robot_interfaces_facility::srv::ReleaseLiftSession>::SharedPtr
      release_lift_session_service_;
  rclcpp::Service<robot_interfaces_facility::srv::GetLiftState>::SharedPtr
      get_lift_state_service_;
  rclcpp::Service<robot_interfaces_facility::srv::GetDoorState>::SharedPtr
      get_door_state_service_;
  rclcpp::Service<robot_interfaces_mission::srv::ListDocks>::SharedPtr list_docks_service_;
  rclcpp::Service<robot_interfaces_mission::srv::RequestDocking>::SharedPtr
      request_docking_service_;
  rclcpp::Service<robot_interfaces_mission::srv::RequestUndocking>::SharedPtr
      request_undocking_service_;
  rclcpp::Service<robot_interfaces_mission::srv::GetDockState>::SharedPtr
      get_dock_state_service_;
  rclcpp::Service<robot_interfaces_facility::srv::ConfirmLoad>::SharedPtr
      confirm_load_service_;
  rclcpp::Service<robot_interfaces_facility::srv::ConfirmUnload>::SharedPtr
      confirm_unload_service_;
  rclcpp::Service<robot_interfaces_facility::srv::SetPayloadLoaded>::SharedPtr
      set_payload_loaded_service_;
  rclcpp::Service<robot_interfaces_facility::srv::ExecuteTopModuleAction>::SharedPtr
      execute_top_module_action_service_;
  rclcpp::Service<robot_interfaces_navigation::srv::RequestRelocalization>::SharedPtr
      request_relocalization_service_;
  rclcpp::Service<robot_interfaces_navigation::srv::SetInitialPoseFromStation>::SharedPtr
      set_initial_pose_from_station_service_;
  rclcpp::Service<robot_interfaces_navigation::srv::SetDynamicSpeedLimit>::SharedPtr
      set_dynamic_speed_limit_service_;
  rclcpp::Service<robot_interfaces_navigation::srv::ReportObstacleBlockage>::SharedPtr
      report_obstacle_blockage_service_;
  rclcpp::Service<robot_interfaces_navigation::srv::ClearObstacleBlockage>::SharedPtr
      clear_obstacle_blockage_service_;
  rclcpp::Service<robot_interfaces_mission::srv::RequestCharging>::SharedPtr
      request_charging_service_;
  rclcpp::Service<robot_interfaces_mission::srv::RequestOpportunityCharging>::SharedPtr
      request_opportunity_charging_service_;
  rclcpp::Service<robot_interfaces_facility::srv::ExecuteFacilityAction>::SharedPtr
      execute_facility_action_service_;
  rclcpp::Service<robot_interfaces_facility::srv::RequestStationConfirmation>::SharedPtr
      request_station_confirmation_service_;
  rclcpp::Service<robot_interfaces_facility::srv::GetPendingConfirmations>::SharedPtr
      get_pending_confirmations_service_;
  rclcpp::Service<robot_interfaces_facility::srv::ConfirmStationAction>::SharedPtr
      confirm_station_action_service_;
  rclcpp::Service<robot_interfaces_facility::srv::RejectStationAction>::SharedPtr
      reject_station_action_service_;
  rclcpp::Service<robot_interfaces_business::srv::ConfirmPick>::SharedPtr confirm_pick_service_;
  rclcpp::Service<robot_interfaces_business::srv::ConfirmDropoff>::SharedPtr
      confirm_dropoff_service_;
  rclcpp::Service<robot_interfaces_business::srv::ReportShortPick>::SharedPtr
      report_short_pick_service_;
  rclcpp::Service<robot_interfaces_business::srv::GetOrderFulfillmentStatus>::SharedPtr
      get_order_fulfillment_status_service_;
  rclcpp::Service<robot_interfaces_mission::srv::SubmitStationTransportOrder>::SharedPtr
      submit_station_order_service_;
  rclcpp::Service<robot_interfaces_mission::srv::SubmitStationOrderBatch>::SharedPtr
      submit_station_order_batch_service_;
  rclcpp::Service<robot_interfaces_mission::srv::CancelStationOrderBatch>::SharedPtr
      cancel_station_order_batch_service_;
  rclcpp::Service<robot_interfaces_mission::srv::SubmitStationSequenceTask>::SharedPtr
      submit_station_sequence_service_;
  rclcpp::Service<robot_interfaces_mission::srv::EstimateStationTransport>::SharedPtr
      estimate_station_transport_service_;
  rclcpp::Service<robot_interfaces_mission::srv::EstimateStationSequence>::SharedPtr
      estimate_station_sequence_service_;
  rclcpp::Service<std_srvs::srv::Trigger>::SharedPtr return_to_dock_service_;
  rclcpp::TimerBase::SharedPtr timer_;
