/**
 * @file part_4.inc
 * @brief （实现细节）。
 * @details 所属包：`robot_tasks`；所在层次：实现细节。
 * @note 教学说明见 `docs/教学/` 对应章节。
 */

  template <typename RequestT, typename ResponseT>
  void SubmitStationTransportOrder(
      const std::shared_ptr<RequestT>& request,
      const std::shared_ptr<ResponseT>& response) {
    auto profile = CreateStationTransportOrderProfile(*request, response);
    if (!profile.has_value()) {
      response->queue_size = static_cast<int>(mission_queue_.size());
      return;
    }
    if (MissionIdActiveOrQueued(profile->mission_id)) {
      const auto duplicate_gate = robot_tasks::PlanStationTransportSubmitGate(
          robot_tasks::StationTransportSubmitGateRequest{profile->mission_id, true});
      response->success = false;
      response->mission_id = duplicate_gate.mission_id;
      response->message = duplicate_gate.message;
      response->queue_size = static_cast<int>(mission_queue_.size());
      return;
    }
    const auto catalog = robot_tasks::LoadStationCatalog(station_file_);
    if (!catalog.has_value()) {
      const auto gate = robot_tasks::PlanStationTransportSubmitGate(
          robot_tasks::StationTransportSubmitGateRequest{
              profile->mission_id, false, false});
      response->success = false;
      response->mission_id = profile->mission_id;
      response->message = gate.message;
      response->queue_size = static_cast<int>(mission_queue_.size());
      return;
    }
    const auto preflight = ValidateStationMissionForQueue(
        *profile, *catalog, request->pickup_station, request->dropoff_station);
    const auto route_path =
        robot_tasks::FindRoutePath(*catalog, request->pickup_station, request->dropoff_station);
    const auto gate = robot_tasks::PlanStationTransportSubmitGate(
        robot_tasks::StationTransportSubmitGateRequest{
            profile->mission_id, false, true, preflight,
            route_path, traffic_intersection_locks_enabled_});
    if (!gate.accepted) {
      response->success = false;
      response->mission_id = gate.mission_id;
      response->message = gate.message;
      response->queue_size = static_cast<int>(mission_queue_.size());
      return;
    }
    if (payload_loaded_) {
      response->success = false;
      response->mission_id = profile->mission_id;
      response->message =
          "station transport pickup requires an empty robot payload";
      response->queue_size = static_cast<int>(mission_queue_.size());
      return;
    }

    std::string station_pair_message;
    const auto stations = robot_tasks::ResolveStationPair(
        *catalog, request->pickup_station, request->dropoff_station,
        &station_pair_message);
    if (!stations.has_value()) {
      response->success = false;
      response->mission_id = profile->mission_id;
      response->message = station_pair_message;
      response->queue_size = static_cast<int>(mission_queue_.size());
      return;
    }
    std::string route_lock_message;
    if (!ReserveRouteLocksForMission(
            profile->mission_id, gate.route_lock_ids, &route_lock_message)) {
      response->success = false;
      response->mission_id = profile->mission_id;
      response->message = "station transport route lock rejected: " + route_lock_message;
      response->queue_size = static_cast<int>(mission_queue_.size());
      return;
    }

    StationTransportExecutionRecord execution;
    execution.state.mission_id = profile->mission_id;
    execution.state.pickup_station_id = stations->pickup.id;
    execution.state.dropoff_station_id = stations->dropoff.id;
    execution.pickup_profile =
        robot_tasks::BuildStationWaypointProfile(profile->mission_id, stations->pickup);
    execution.dropoff_profile =
        robot_tasks::BuildStationWaypointProfile(profile->mission_id, stations->dropoff);
    station_transport_executions_[profile->mission_id] = execution;

    const auto admission = AdmitQueuedMission(
        QueuedMission{
            execution.pickup_profile, "station_transport_order:" + profile->mission_id,
            request->priority,
            next_queue_sequence_++},
        request->start_if_idle, request->preempt_current,
        "preempting active mission for station order " + profile->mission_id,
        "queued station transport order " + profile->mission_id + " priority " +
            std::to_string(request->priority));
    response->success = admission.success;
    response->mission_id = profile->mission_id;
    response->queue_size = admission.queue_size;
    response->message = admission.message;
    if (!admission.success) {
      station_transport_executions_.erase(profile->mission_id);
      ReleaseAllResourcesForMission(profile->mission_id);
    }
    ApplyMissionQueueAdmissionState(admission);
  }

  template <typename RequestT, typename ResponseT>
  void SubmitStationOrderBatch(
      const std::shared_ptr<RequestT>& request,
      const std::shared_ptr<ResponseT>& response) {
    std::vector<std::string> sanitized_order_ids;
    sanitized_order_ids.reserve(request->order_ids.size());
    for (const auto& order_id : request->order_ids) {
      sanitized_order_ids.push_back(SanitizeId(order_id));
    }

    robot_tasks::StationOrderBatchSubmitRequest plan_request;
    plan_request.batch_id = SanitizeId(request->batch_id);
    plan_request.original_order_ids = request->order_ids;
    plan_request.sanitized_order_ids = sanitized_order_ids;
    plan_request.pickup_station_ids = request->pickup_station_ids;
    plan_request.dropoff_station_ids = request->dropoff_station_ids;
    plan_request.priority = request->priority;
    plan_request.start_if_idle = request->start_if_idle;
    plan_request.preempt_current = request->preempt_current;
    plan_request.continue_on_error = request->continue_on_error;
    const auto plan = robot_tasks::PlanStationOrderBatchSubmit(plan_request);
    if (!plan.accepted) {
      response->success = false;
      response->message = plan.message;
      response->queue_size = static_cast<int>(mission_queue_.size());
      return;
    }

    for (const auto& item : plan.items) {
      if (!item.accepted) {
        response->rejected_order_ids.push_back(item.reject_order_id);
        response->rejected_messages.push_back(item.reject_message);
        continue;
      }
      auto station_request =
          std::make_shared<robot_interfaces_mission::srv::SubmitStationTransportOrder::Request>();
      station_request->order_id = item.station_order_id;
      station_request->pickup_station = item.pickup_station;
      station_request->dropoff_station = item.dropoff_station;
      station_request->priority = item.priority;
      station_request->start_if_idle = item.start_if_idle;
      station_request->preempt_current = item.preempt_current;

      auto station_response =
          std::make_shared<robot_interfaces_mission::srv::SubmitStationTransportOrder::Response>();
      SubmitStationTransportOrder(station_request, station_response);
      response->queue_size = station_response->queue_size;
      if (station_response->success) {
        response->mission_ids.push_back(station_response->mission_id);
        continue;
      }
      response->rejected_order_ids.push_back(item.order_id);
      response->rejected_messages.push_back(station_response->message);
      if (!request->continue_on_error) {
        break;
      }
    }

    response->accepted_count = static_cast<int>(response->mission_ids.size());
    response->rejected_count = static_cast<int>(response->rejected_order_ids.size());
    response->queue_size = static_cast<int>(mission_queue_.size());
    const auto summary = robot_tasks::PlanStationOrderBatchSubmitSummary(
        plan_request.batch_id, request->continue_on_error, response->accepted_count,
        response->rejected_count);
    response->success = summary.success;
    response->message = summary.message;
  }

  template <typename RequestT, typename ResponseT>
  void CancelStationOrderBatch(
      const std::shared_ptr<RequestT>& request,
      const std::shared_ptr<ResponseT>& response) {
    const auto batch_id = SanitizeId(request->batch_id);
    if (batch_id.empty()) {
      response->success = false;
      response->message = "batch_id is empty";
      response->queue_size = static_cast<int>(mission_queue_.size());
      return;
    }

    robot_tasks::StationOrderBatchCancelRequest cancel_request;
    cancel_request.queue = mission_queue_;
    cancel_request.batch_id = batch_id;
    cancel_request.cancel_active = request->cancel_active;
    cancel_request.mission_active = RunnerBusy();
    cancel_request.active_mission_id = ActiveMissionId();
    const auto decision = robot_tasks::PlanStationOrderBatchCancel(cancel_request);
    mission_queue_ = decision.remaining_queue;
    response->canceled_mission_ids = decision.canceled_mission_ids;
    response->active_cancel_requested = decision.active_cancel_requested;

    for (const auto& mission_id : decision.release_mission_ids) {
      ReleaseAllResourcesForMission(mission_id);
      station_transport_executions_.erase(mission_id);
    }

    if (decision.active_cancel_requested) {
      RequestActiveMissionCancel("station order batch cancellation requested for " + batch_id);
    }

    response->queue_size = decision.queue_size;
    response->success = decision.success;
    response->message = decision.message;
    if (decision.set_state) {
      SetState(decision.state, decision.state_message, decision.state_recoverable);
    }
  }

  template <typename RequestT, typename ResponseT>
  void EstimateStationTransport(
      const std::shared_ptr<RequestT>& request,
      const std::shared_ptr<ResponseT>& response) {
    auto profile = CreateStationEstimateProfile(*request, response);
    if (!profile.has_value()) {
      return;
    }
    const auto catalog = robot_tasks::LoadStationCatalog(station_file_);
    if (!catalog.has_value()) {
      response->success = false;
      response->message = "failed to load station catalog: " + station_file_;
      return;
    }
    FillStationEstimateResponse(
        *profile, *catalog, request->pickup_station, request->dropoff_station,
        request->nominal_speed_mps, request->battery_voltage, response.get());
  }

  template <typename RequestT, typename ResponseT>
  void SubmitStationSequenceTask(
      const std::shared_ptr<RequestT>& request,
      const std::shared_ptr<ResponseT>& response) {
    const auto catalog = robot_tasks::LoadStationCatalog(station_file_);
    if (!catalog.has_value()) {
      response->success = false;
      response->message = "failed to load station catalog: " + station_file_;
      response->queue_size = static_cast<int>(mission_queue_.size());
      return;
    }

    std::string message;
    const auto profiles = BuildStationSequenceProfiles(*request, *catalog, &message);
    if (!profiles.has_value()) {
      response->success = false;
      response->message = message;
      response->queue_size = static_cast<int>(mission_queue_.size());
      return;
    }

    const auto conflict =
        robot_tasks::CheckStationSequenceConflicts(*profiles, ActiveOrQueuedMissionIds());
    if (!conflict.accepted) {
      response->success = false;
      response->message = conflict.message;
      response->queue_size = static_cast<int>(mission_queue_.size());
      return;
    }

    for (const auto& leg : *profiles) {
      const auto preflight =
          ValidateStationMissionForQueue(leg.profile, *catalog, leg.pickup_id, leg.dropoff_id);
      const auto admission =
          robot_tasks::CheckStationSequenceLegPreflight(leg, preflight);
      if (!admission.accepted) {
        response->success = false;
        response->message = admission.message;
        response->queue_size = static_cast<int>(mission_queue_.size());
        return;
      }
    }

    const auto queued_missions = robot_tasks::BuildStationSequenceQueuedMissions(
        *profiles, request->priority, next_queue_sequence_);
    next_queue_sequence_ += queued_missions.size();
    response->mission_ids = robot_tasks::StationSequenceMissionIds(*profiles);
    const auto admission = AdmitQueuedMissions(
        queued_missions, request->start_if_idle, request->preempt_current,
        "preempting active mission for station sequence " + SanitizeId(request->order_id),
        robot_tasks::BuildStationSequenceQueuedMessage(queued_missions.size()));
    response->success = admission.success;
    response->queued_count = static_cast<int>(queued_missions.size());
    response->queue_size = admission.queue_size;
    response->message = admission.message;
    ApplyMissionQueueAdmissionState(admission);
  }

  template <typename RequestT, typename ResponseT>
  void EstimateStationSequence(
      const std::shared_ptr<RequestT>& request,
      const std::shared_ptr<ResponseT>& response) {
    const auto catalog = robot_tasks::LoadStationCatalog(station_file_);
    if (!catalog.has_value()) {
      response->success = false;
      response->message = "failed to load station catalog: " + station_file_;
      return;
    }

    std::string message;
    const auto profile = BuildStationSequenceEstimateProfile(*request, *catalog, &message);
    if (!profile.has_value()) {
      response->success = false;
      response->message = message;
      return;
    }
    FillStationSequenceEstimateResponse(
        *profile, *catalog, request->station_ids, request->nominal_speed_mps,
        request->battery_voltage, response.get());
  }

  std::vector<std::string> ReservedResourceIds() const {
    return robot_tasks::ReservedResourceIds(facility_reservations_);
  }

  const robot_tasks::FacilityResource* SelectChargingResource(
      const robot_tasks::FacilityCatalog& catalog, const std::string& requested_charger_id,
      std::string* message) const {
    if (!requested_charger_id.empty()) {
      const auto* resource = robot_tasks::FindFacilityResource(catalog, requested_charger_id);
      if (resource == nullptr) {
        if (message != nullptr) {
          *message = "unknown charger resource: " + requested_charger_id;
        }
        return nullptr;
      }
      if (resource->type != "charger") {
        if (message != nullptr) {
          *message = "facility resource is not a charger: " + requested_charger_id;
        }
        return nullptr;
      }
      if (!ResourceAvailable(*resource)) {
        if (message != nullptr) {
          *message = "charger unavailable: " + requested_charger_id;
        }
        return nullptr;
      }
      return resource;
    }
    const auto* resource =
        robot_tasks::FindFirstAvailableResource(catalog, "charger", ReservedResourceIds());
    if (resource == nullptr && message != nullptr) {
      *message = "no available charger resource";
    }
    return resource;
  }

  template <typename RequestT, typename ResponseT>
  void RequestCharging(
      const std::shared_ptr<RequestT>& request,
      const std::shared_ptr<ResponseT>& response) {
    std::string message;
    const auto facilities = LoadFacilities(&message);
    if (!facilities.has_value()) {
      response->success = false;
      response->message = message;
      response->queue_size = static_cast<int>(mission_queue_.size());
      return;
    }
    const auto* charger = SelectChargingResource(*facilities, request->charger_id, &message);
    if (charger == nullptr) {
      response->success = false;
      response->message = message;
      response->queue_size = static_cast<int>(mission_queue_.size());
      return;
    }
    const auto stations = robot_tasks::LoadStationCatalog(station_file_);
    if (!stations.has_value()) {
      const auto gate = robot_tasks::PlanChargingRequestGate(
          robot_tasks::ChargingRequestGateRequest{
              "", charger->id, charger->station_id, false,
              "failed to load station catalog: " + station_file_});
      response->success = false;
      response->charger_id = gate.charger_id;
      response->message = gate.message;
      response->queue_size = static_cast<int>(mission_queue_.size());
      return;
    }
    const auto* station = robot_tasks::FindStation(*stations, charger->station_id);
    if (station == nullptr) {
      const auto gate = robot_tasks::PlanChargingRequestGate(
          robot_tasks::ChargingRequestGateRequest{
              "", charger->id, charger->station_id, true, "", false});
      response->success = false;
      response->charger_id = gate.charger_id;
      response->message = gate.message;
      response->queue_size = static_cast<int>(mission_queue_.size());
      return;
    }
    const auto sanitized_request_id = SanitizeId(request->request_id);
    const std::string mission_id =
        sanitized_request_id.empty()
            ? "charging_request_" + std::to_string(next_charging_request_sequence_++)
            : "charging_request_" + sanitized_request_id;
    if (MissionIdActiveOrQueued(mission_id)) {
      const auto gate = robot_tasks::PlanChargingRequestGate(
          robot_tasks::ChargingRequestGateRequest{
              mission_id, charger->id, charger->station_id, true, "", true, true});
      response->success = false;
      response->mission_id = gate.mission_id;
      response->charger_id = gate.charger_id;
      response->message = gate.message;
      response->queue_size = static_cast<int>(mission_queue_.size());
      return;
    }
    auto profile = robot_tasks::BuildStationWaypointProfile(mission_id, *station);
    const auto preflight = ValidateMissionForQueue(profile);
    const auto gate = robot_tasks::PlanChargingRequestGate(
        robot_tasks::ChargingRequestGateRequest{
            mission_id, charger->id, charger->station_id, true, "", true, false, preflight});
    if (!gate.accepted) {
      response->success = false;
      response->mission_id = gate.mission_id;
      response->charger_id = gate.charger_id;
      response->message = gate.message;
      response->queue_size = static_cast<int>(mission_queue_.size());
      return;
    }

    facility_reservations_[charger->id] =
        robot_tasks::ResourceReservation{mission_id, mission_id, "reserved"};
    facility_resource_by_mission_[mission_id] = charger->id;
    const auto admission = AdmitQueuedMission(
        QueuedMission{profile, "charging_request:" + mission_id, request->priority,
                      next_queue_sequence_++},
        request->start_if_idle, request->preempt_current,
        "preempting active mission for charging request " + mission_id,
        "queued charging request " + mission_id + " using charger " + charger->id);
    response->success = admission.success;
    response->mission_id = mission_id;
    response->charger_id = charger->id;
    response->queue_size = admission.queue_size;
    response->message = admission.message;
    ApplyMissionQueueAdmissionState(admission);
  }

  template <typename RequestT, typename ResponseT>
  void RequestOpportunityCharging(
      const std::shared_ptr<RequestT>& request,
      const std::shared_ptr<ResponseT>& response) {
    robot_tasks::OpportunityChargingPlanRequest plan_request;
    plan_request.requested_battery_voltage = request->battery_voltage;
    plan_request.current_battery_voltage = current_battery_voltage_;
    plan_request.requested_opportunity_threshold_voltage =
        request->opportunity_threshold_voltage;
    plan_request.default_opportunity_threshold_voltage = opportunity_charge_threshold_voltage_;
    plan_request.requested_critical_threshold_voltage = request->critical_threshold_voltage;
    plan_request.default_critical_threshold_voltage = critical_charge_threshold_voltage_;
    plan_request.requested_require_idle_queue = request->require_idle_queue;
    plan_request.default_require_idle_queue = opportunity_charge_require_idle_queue_;
    plan_request.mission_active = RunnerBusy();
    plan_request.queue_empty = mission_queue_.empty();
    plan_request.requested_priority = request->priority;
    plan_request.default_priority = opportunity_charge_priority_;
    plan_request.requested_preempt_current = request->preempt_current;
    const auto decision = robot_tasks::PlanOpportunityCharging(plan_request);

    response->evaluated_battery_voltage = decision.evaluated_battery_voltage;
    response->queue_size = static_cast<int>(mission_queue_.size());
    if (!decision.success || !decision.queue_charging) {
      response->success = decision.success;
      response->charging_queued = false;
      response->message = decision.message;
      return;
    }

    auto charging_request =
        std::make_shared<robot_interfaces_mission::srv::RequestCharging::Request>();
    charging_request->request_id = request->request_id;
    charging_request->charger_id = request->charger_id;
    charging_request->priority = decision.priority;
    charging_request->start_if_idle = request->start_if_idle;
    charging_request->preempt_current = decision.preempt_current;

    auto charging_response =
        std::make_shared<robot_interfaces_mission::srv::RequestCharging::Response>();
    RequestCharging(charging_request, charging_response);
    response->success = charging_response->success;
    response->charging_queued = charging_response->success;
    response->message = decision.queued_message_prefix + charging_response->message;
    response->mission_id = charging_response->mission_id;
    response->charger_id = charging_response->charger_id;
    response->queue_size = charging_response->queue_size;
  }

  template <typename ResponseT>
  void FillEstimateResponse(
      const robot_tasks::MissionProfile& profile, const double requested_speed_mps,
      const double requested_battery_voltage, ResponseT* response) const {
    const robot_navigation::ZoneCatalog zone_catalog(preflight_zones_file_);
    const auto path_distance_m = PreferredPathDistanceOverride(profile, std::nullopt);
    const auto preflight = robot_tasks::ValidateMissionPreflight(
        profile, zone_catalog, MakePreflightConfig(requested_speed_mps, requested_battery_voltage),
        path_distance_m);
    FillEstimateFields(
        profile, preflight, response,
        path_distance_m.has_value() ? "estimated mission cost using Nav2 path length"
                                    : "estimated mission cost");
  }

  template <typename ResponseT>
  void FillStationEstimateResponse(
      const robot_tasks::MissionProfile& profile, const robot_tasks::StationCatalog& catalog,
      const std::string& pickup_station, const std::string& dropoff_station,
      const double requested_speed_mps, const double requested_battery_voltage,
      ResponseT* response) const {
    const robot_navigation::ZoneCatalog zone_catalog(preflight_zones_file_);
    const auto path_distance_m = TryComputeNav2PathDistance(profile);
    const auto preflight = robot_tasks::ValidateStationTransportPreflight(
        profile, catalog, pickup_station, dropoff_station, zone_catalog,
        MakePreflightConfig(requested_speed_mps, requested_battery_voltage), path_distance_m);
    FillEstimateFields(
        profile, preflight, response,
        path_distance_m.has_value() ? "estimated station transport Nav2 path cost"
                                    : "estimated station transport route cost");
  }

  template <typename ResponseT>
  void FillStationSequenceEstimateResponse(
      const robot_tasks::MissionProfile& profile, const robot_tasks::StationCatalog& catalog,
      const std::vector<std::string>& station_ids, const double requested_speed_mps,
      const double requested_battery_voltage, ResponseT* response) const {
    const robot_navigation::ZoneCatalog zone_catalog(preflight_zones_file_);
    auto config = MakePreflightConfig(requested_speed_mps, requested_battery_voltage);
    robot_tasks::MissionPreflightResult preflight;
    preflight.allowed = true;
    preflight.message = "station sequence accepted";
    double total_route_distance_m = 0.0;

    for (std::size_t index = 1; index < station_ids.size(); ++index) {
      const auto route_distance =
          robot_tasks::EstimateRouteDistance(catalog, station_ids[index - 1], station_ids[index]);
      if (!route_distance.has_value()) {
        preflight.allowed = false;
        preflight.message =
            "no enabled station route from " + station_ids[index - 1] + " to " + station_ids[index];
        break;
      }
      total_route_distance_m += *route_distance;
    }
    const auto path_distance_m = TryComputeNav2PathDistance(profile);
    const double effective_distance_m =
        path_distance_m.has_value() ? *path_distance_m : total_route_distance_m;
    preflight.cost = robot_tasks::EstimateDistanceCost(
        effective_distance_m, static_cast<int>(profile.waypoints.size()),
        config.nominal_speed_mps, config.battery_voltage, config.battery_drop_per_meter,
        config.minimum_battery_voltage,
        path_distance_m.has_value() ? "planned_path" : "station_route_graph");
    if (preflight.allowed) {
      const auto zone_preflight =
          robot_tasks::ValidateMissionPreflight(profile, zone_catalog, config, effective_distance_m);
      if (!zone_preflight.allowed) {
        preflight.allowed = false;
        preflight.message = zone_preflight.message;
      }
      preflight.cost = zone_preflight.cost;
    }
    FillEstimateFields(
        profile, preflight, response,
        path_distance_m.has_value() ? "estimated station sequence Nav2 path cost"
                                    : "estimated station sequence route cost");
  }

  std::optional<double> PreferredPathDistanceOverride(
      const robot_tasks::MissionProfile& profile,
      const std::optional<double> fallback_distance_m) const {
    const auto nav2_distance_m = TryComputeNav2PathDistance(profile);
    if (nav2_distance_m.has_value()) {
      return nav2_distance_m;
    }
    return fallback_distance_m;
  }

  std::optional<double> TryComputeNav2PathDistance(
      const robot_tasks::MissionProfile& profile) const {
    if (!estimate_use_nav2_path_ || profile.waypoints.size() < 2 || !nav2_path_node_ ||
        !nav2_path_client_) {
      return std::nullopt;
    }

    std::lock_guard<std::mutex> lock(nav2_path_mutex_);
    const auto server_timeout =
        std::chrono::milliseconds(std::max(0, estimate_nav2_server_timeout_ms_));
    if (!nav2_path_client_->wait_for_action_server(server_timeout)) {
      return std::nullopt;
    }

    rclcpp::executors::SingleThreadedExecutor executor;
    executor.add_node(nav2_path_node_);
    const auto result_timeout =
        std::chrono::milliseconds(std::max(1, estimate_nav2_result_timeout_ms_));
    double total_distance_m = 0.0;
    for (std::size_t index = 1; index < profile.waypoints.size(); ++index) {
      const auto stamp = nav2_path_node_->now();
      ComputePathToPose::Goal goal;
      goal.start = PoseStampedFromWaypoint(profile.waypoints[index - 1], profile.frame_id, stamp);
      goal.goal = PoseStampedFromWaypoint(profile.waypoints[index], profile.frame_id, stamp);
      goal.planner_id = estimate_nav2_planner_id_;
      goal.use_start = true;

      auto goal_future = nav2_path_client_->async_send_goal(goal);
      if (executor.spin_until_future_complete(goal_future, result_timeout) !=
          rclcpp::FutureReturnCode::SUCCESS) {
        executor.remove_node(nav2_path_node_);
        return std::nullopt;
      }
      auto goal_handle = goal_future.get();
      if (!goal_handle) {
        executor.remove_node(nav2_path_node_);
        return std::nullopt;
      }

      auto result_future = nav2_path_client_->async_get_result(goal_handle);
      if (executor.spin_until_future_complete(result_future, result_timeout) !=
          rclcpp::FutureReturnCode::SUCCESS) {
        nav2_path_client_->async_cancel_goal(goal_handle);
        executor.remove_node(nav2_path_node_);
        return std::nullopt;
      }

      const auto wrapped = result_future.get();
      if (wrapped.code != rclcpp_action::ResultCode::SUCCEEDED || wrapped.result == nullptr ||
          wrapped.result->error_code != ComputePathToPose::Result::NONE) {
        executor.remove_node(nav2_path_node_);
        return std::nullopt;
      }
      total_distance_m += PathLength(wrapped.result->path);
    }
    executor.remove_node(nav2_path_node_);
    return total_distance_m;
  }

  double RequestedOrDefaultSpeed(const double requested_speed_mps) const {
    return requested_speed_mps > 0.0 ? requested_speed_mps : estimate_nominal_speed_mps_;
  }

  double RequestedOrDefaultBattery(const double requested_battery_voltage) const {
    return requested_battery_voltage > 0.0 ? requested_battery_voltage : dock_low_battery_voltage_ + 2.0;
  }

  template <typename ResponseT>
  void FillEstimateFields(
      const robot_tasks::MissionProfile& profile,
      const robot_tasks::MissionPreflightResult& preflight, ResponseT* response,
      const std::string& accepted_message) const {
    response->success = true;
    response->message =
        preflight.allowed ? accepted_message : accepted_message + "; preflight rejected: " + preflight.message;
    response->mission_id = profile.mission_id;
    response->waypoint_count = preflight.cost.waypoint_count;
    response->distance_m = preflight.cost.distance_m;
    response->eta_sec = preflight.cost.eta_sec;
    response->battery_drop_v = preflight.cost.battery_drop_v;
    response->projected_battery_voltage = preflight.cost.projected_battery_voltage;
    response->battery_sufficient = preflight.cost.battery_sufficient;
  }

  robot_tasks::MissionPreflightConfig MakePreflightConfig() const {
    return MakePreflightConfig(0.0, 0.0);
  }

  robot_tasks::MissionPreflightConfig MakePreflightConfig(
      const double requested_speed_mps, const double requested_battery_voltage) const {
    robot_tasks::MissionPreflightConfig config;
    config.enabled = preflight_enabled_;
    config.map_name = preflight_map_name_;
    config.default_speed_limit_mps = preflight_default_speed_limit_mps_;
    config.nominal_speed_mps = RequestedOrDefaultSpeed(requested_speed_mps);
    config.battery_voltage = requested_battery_voltage > 0.0
                                  ? requested_battery_voltage
                                  : (current_battery_voltage_ > 0.0
                                         ? current_battery_voltage_
                                         : dock_low_battery_voltage_ + 2.0);
    config.battery_drop_per_meter = estimate_battery_drop_per_meter_;
    config.minimum_battery_voltage = estimate_min_battery_voltage_;
    config.require_station_route = preflight_require_station_route_;
    return config;
  }

  robot_tasks::MissionPreflightResult ValidateMissionForQueue(
      const robot_tasks::MissionProfile& profile) const {
    const robot_navigation::ZoneCatalog zone_catalog(preflight_zones_file_);
    return robot_tasks::ValidateMissionPreflight(
        profile, zone_catalog, MakePreflightConfig(),
        PreferredPathDistanceOverride(profile, std::nullopt));
  }

  robot_tasks::MissionPreflightResult ValidateStationMissionForQueue(
      const robot_tasks::MissionProfile& profile, const robot_tasks::StationCatalog& catalog,
      const std::string& pickup_station, const std::string& dropoff_station) const {
    const robot_navigation::ZoneCatalog zone_catalog(preflight_zones_file_);
    return robot_tasks::ValidateStationTransportPreflight(
        profile, catalog, pickup_station, dropoff_station, zone_catalog, MakePreflightConfig(),
        TryComputeNav2PathDistance(profile));
  }

  template <typename ResponseT>
  std::optional<robot_tasks::StationCatalog> LoadStations(
      const std::shared_ptr<ResponseT>& response) {
    const auto catalog = robot_tasks::LoadStationCatalog(station_file_);
    if (!catalog.has_value()) {
      if (response != nullptr) {
        response->success = false;
        response->message = "failed to load station catalog: " + station_file_;
      }
      return std::nullopt;
    }
    return catalog;
  }

  template <typename RequestT, typename ResponseT>
  std::optional<robot_tasks::MissionProfile> CreateStationTransportOrderProfile(
      const RequestT& request,
      const std::shared_ptr<ResponseT>& response) {
    const auto catalog = robot_tasks::LoadStationCatalog(station_file_);
    if (!catalog.has_value()) {
      response->success = false;
      response->message = "failed to load station catalog: " + station_file_;
      return std::nullopt;
    }

    std::string message;
    const auto stations = robot_tasks::ResolveStationPair(
        *catalog, request.pickup_station, request.dropoff_station, &message);
    if (!stations.has_value()) {
      response->success = false;
      response->message = message;
      return std::nullopt;
    }

    const auto sanitized_order_id = SanitizeId(request.order_id);
    std::string mission_id;
    if (sanitized_order_id.empty()) {
      mission_id = "station_order_" + std::to_string(next_station_order_sequence_++);
    } else {
      mission_id = "station_order_" + sanitized_order_id;
    }
    auto profile = robot_tasks::BuildTwoWaypointProfile(
        mission_id, stations->pickup.frame_id, stations->pickup.x, stations->pickup.y,
        stations->pickup.yaw, stations->dropoff.x, stations->dropoff.y, stations->dropoff.yaw);

    if (!robot_tasks::ValidateMissionProfile(profile, &message)) {
      response->success = false;
      response->mission_id = profile.mission_id;
      response->message = "invalid station transport mission: " + message;
      return std::nullopt;
    }
    return profile;
  }

  template <typename RequestT, typename ResponseT>
  std::optional<robot_tasks::MissionProfile> CreateStationEstimateProfile(
      const RequestT& request,
      const std::shared_ptr<ResponseT>& response) {
    const auto catalog = robot_tasks::LoadStationCatalog(station_file_);
    if (!catalog.has_value()) {
      response->success = false;
      response->message = "failed to load station catalog: " + station_file_;
      return std::nullopt;
    }

    std::string message;
    const auto stations = robot_tasks::ResolveStationPair(
        *catalog, request.pickup_station, request.dropoff_station, &message);
    if (!stations.has_value()) {
      response->success = false;
      response->message = message;
      return std::nullopt;
    }

    return robot_tasks::BuildTwoWaypointProfile(
        "estimate_" + SanitizeId(request.pickup_station) + "_to_" +
            SanitizeId(request.dropoff_station),
        stations->pickup.frame_id, stations->pickup.x, stations->pickup.y, stations->pickup.yaw,
        stations->dropoff.x, stations->dropoff.y, stations->dropoff.yaw);
  }

  template <typename RequestT>
  std::optional<std::vector<robot_tasks::StationSequenceLeg>> BuildStationSequenceProfiles(
      const RequestT& request,
      const robot_tasks::StationCatalog& catalog, std::string* message) {
    const auto order_id = SanitizeId(request.order_id);
    const auto base_id = order_id.empty()
                             ? "station_sequence_" +
                                   std::to_string(next_station_sequence_order_sequence_++)
                             : "station_sequence_" + order_id;
    return robot_tasks::BuildStationSequenceProfiles(catalog, request.station_ids, base_id, message);
  }

  template <typename RequestT>
  std::optional<robot_tasks::MissionProfile> BuildStationSequenceEstimateProfile(
      const RequestT& request,
      const robot_tasks::StationCatalog& catalog, std::string* message) const {
    return robot_tasks::BuildStationSequenceEstimateProfile(catalog, request.station_ids, message);
  }

  template <typename RequestT, typename ResponseT>
  std::optional<robot_tasks::MissionProfile> CreateTransportOrderProfile(
      const RequestT& request,
      const std::shared_ptr<ResponseT>& response) {
    if (!std::isfinite(request.pickup_x) || !std::isfinite(request.pickup_y) ||
        !std::isfinite(request.pickup_yaw) || !std::isfinite(request.dropoff_x) ||
        !std::isfinite(request.dropoff_y) || !std::isfinite(request.dropoff_yaw)) {
      response->success = false;
      response->message = "transport order contains non-finite waypoint values";
      return std::nullopt;
    }

    robot_tasks::MissionProfile profile;
    const auto sanitized_order_id = SanitizeId(request.order_id);
    if (sanitized_order_id.empty()) {
      profile.mission_id = "transport_order_" + std::to_string(next_transport_order_sequence_++);
    } else {
      profile.mission_id = "transport_order_" + sanitized_order_id;
    }
    profile = robot_tasks::BuildTwoWaypointProfile(
        profile.mission_id, request.frame_id.empty() ? "map" : request.frame_id, request.pickup_x,
        request.pickup_y, request.pickup_yaw, request.dropoff_x, request.dropoff_y,
        request.dropoff_yaw);

    std::string message;
    if (!robot_tasks::ValidateMissionProfile(profile, &message)) {
      response->success = false;
      response->mission_id = profile.mission_id;
      response->message = "invalid transport order mission: " + message;
      return std::nullopt;
    }
    return profile;
  }

  bool MissionIdActiveOrQueued(const std::string& mission_id) const {
    const auto transport = station_transport_executions_.find(mission_id);
    if (transport != station_transport_executions_.end() &&
        !robot_tasks::IsStationTransportTerminal(transport->second.state.phase)) {
      return true;
    }
    if (RunnerBusy() && ActiveMissionId() == mission_id) {
      return true;
    }
    return robot_tasks::MissionIdQueued(mission_queue_, mission_id);
  }

  std::unordered_set<std::string> ActiveOrQueuedMissionIds() const {
    std::unordered_set<std::string> mission_ids;
    if (RunnerBusy() && !ActiveMissionId().empty()) {
      mission_ids.insert(ActiveMissionId());
    }
    for (const auto& mission : mission_queue_) {
      mission_ids.insert(mission.profile.mission_id);
    }
    for (const auto& item : station_transport_executions_) {
      if (!robot_tasks::IsStationTransportTerminal(item.second.state.phase)) {
        mission_ids.insert(item.first);
      }
    }
    return mission_ids;
  }

  static std::string RouteLockId(const std::string& from, const std::string& to) {
    return robot_tasks::RouteLockId(from, to);
  }

  std::vector<std::string> BuildRouteLockIds(const std::vector<std::string>& station_path) const {
    return robot_tasks::BuildRouteLockIds(station_path, traffic_intersection_locks_enabled_);
  }

  bool ReserveRouteLocksForMission(
      const std::string& mission_id, const std::vector<std::string>& route_lock_ids,
      std::string* message) {
    return robot_tasks::ReserveRouteLocksForMission(
        route_locks_, route_locks_by_mission_, mission_id, route_lock_ids, message);
  }

  void ReleaseRouteLocksForMission(const std::string& mission_id) {
    robot_tasks::ReleaseRouteLocksForMission(route_locks_, route_locks_by_mission_, mission_id);
  }

  void SortMissionQueue() {
    robot_tasks::SortMissionQueue(mission_queue_);
  }

  void ApplyQueuedMissionStartPolicy(
      const bool start_if_idle, const bool preempt_current,
      const std::string& active_cancel_message) {
    robot_tasks::QueuedMissionStartRequest request;
    request.start_if_idle = start_if_idle;
    request.preempt_current = preempt_current;
    request.mission_active = RunnerBusy();
    robot_tasks::QueuedMissionStartBehaviorTreeInput input;
    input.request = request;
    const auto start_result = robot_tasks::TickQueuedMissionStartBehaviorTree(input);
    const auto& decision = start_result.decision;
    if (decision.run_queue) {
      queue_running_ = true;
    }
    if (decision.request_queue_start) {
      queue_start_requested_ = true;
    }
    if (decision.cancel_active) {
      RequestActiveMissionCancel(active_cancel_message);
    }
  }

  robot_tasks::MissionQueueAdmissionDecision AdmitQueuedMissions(
      const std::vector<QueuedMission>& missions, const bool start_if_idle,
      const bool preempt_current, const std::string& active_cancel_message,
      const std::string& response_message, const bool set_state_when_idle = true) {
    for (const auto& mission : missions) {
      mission_queue_.push_back(mission);
    }
    SortMissionQueue();
    ApplyQueuedMissionStartPolicy(start_if_idle, preempt_current, active_cancel_message);

    robot_tasks::MissionQueueAdmissionRequest request;
    request.mission_active = RunnerBusy();
    request.queue_size = mission_queue_.size();
    request.message = response_message;
    request.set_state_when_idle = set_state_when_idle;
    return robot_tasks::PlanMissionQueueAdmission(request);
  }

  robot_tasks::MissionQueueAdmissionDecision AdmitQueuedMission(
      const QueuedMission& mission, const bool start_if_idle, const bool preempt_current,
      const std::string& active_cancel_message, const std::string& response_message,
      const bool set_state_when_idle = true) {
    return AdmitQueuedMissions(
        std::vector<QueuedMission>{mission}, start_if_idle, preempt_current,
        active_cancel_message, response_message, set_state_when_idle);
  }

  void ApplyMissionQueueAdmissionState(
      const robot_tasks::MissionQueueAdmissionDecision& decision) {
    if (decision.set_state) {
      SetState(decision.state, decision.state_message, decision.state_recoverable);
    }
  }

  void StartNextQueuedMission(const std::shared_ptr<std_srvs::srv::Trigger::Response>& response) {
    robot_tasks::MissionQueueBehaviorTreeInput input;
    input.queue = mission_queue_;
    input.paused_order_ids = paused_workflow_orders_;
    const auto queue_dispatch = robot_tasks::TickMissionQueueBehaviorTree(input);
    const auto& dispatch = queue_dispatch.dispatch;

    if (dispatch.queue_empty) {
      queue_running_ = false;
      if (response != nullptr) {
        response->success = false;
        response->message = "mission queue is empty";
      }
      return;
    }
    if (dispatch.all_paused) {
      queue_start_requested_ = false;
      if (response != nullptr) {
        response->success = false;
        response->message = "all queued workflow missions are paused";
      }
      SetState("WORKFLOW_PAUSED", "all queued workflow missions are paused", true);
      return;
    }
    if (!queue_dispatch.success || !dispatch.mission.has_value()) {
      if (response != nullptr) {
        response->success = false;
        response->message = dispatch.message.empty() ? "failed to dispatch queued mission"
                                                     : dispatch.message;
      }
      return;
    }

    const auto transport =
        station_transport_executions_.find(dispatch.mission->profile.mission_id);
    if (transport != station_transport_executions_.end() &&
        !robot_tasks::IsStationTransportTerminal(transport->second.state.phase)) {
      if (payload_loaded_) {
        queue_running_ = true;
        queue_start_requested_ = false;
        if (response != nullptr) {
          response->success = false;
          response->message =
              "station transport pickup requires an empty robot payload";
        }
        SetState(
            "QUEUED", "station transport waiting for the robot payload to be emptied",
            true);
        return;
      }

      mission_queue_ = dispatch.remaining_queue;
      profile_ = transport->second.pickup_profile;
      mission_file_ = dispatch.mission->mission_file;
      active_station_transport_id_ = transport->first;
      std::string dispatch_message;
      const bool sent = DispatchStationTransportLeg(
          transport->first, robot_tasks::StationTransportLeg::kPickup,
          &dispatch_message);
      if (response != nullptr) {
        response->success = sent;
        response->message = dispatch_message;
      }
      return;
    }

    mission_queue_ = dispatch.remaining_queue;
    profile_ = dispatch.mission->profile;
    mission_file_ = dispatch.mission->mission_file;
    DispatchMission(response, "queued mission profile sent");
  }

  std::optional<robot_tasks::MissionProfile> LoadDockProfile(std::string* message) const {
    const auto profile = robot_tasks::LoadMissionProfile(dock_mission_file_);
    if (!profile.has_value()) {
      if (message != nullptr) {
        *message = "failed to load dock mission profile: " + dock_mission_file_;
      }
      return std::nullopt;
    }
    if (message != nullptr) {
      *message = "loaded dock mission " + profile->mission_id;
    }
    return profile;
  }

  bool DockAlreadyRequestedOrRunning(const std::string& dock_mission_id) const {
    if (RunnerBusy() && ActiveMissionId() == dock_mission_id) {
      return true;
    }
    return std::any_of(
        mission_queue_.begin(), mission_queue_.end(), [&](const QueuedMission& mission) {
          return mission.profile.mission_id == dock_mission_id;
        });
  }

  void QueueDockMission(const robot_tasks::MissionProfile& dock_profile, const std::string& reason) {
    mission_queue_.push_back(
        QueuedMission{dock_profile, dock_mission_file_, dock_priority_, next_queue_sequence_++});
    SortMissionQueue();
    queue_running_ = true;
    queue_start_requested_ = true;
    SetState("QUEUED", reason + ": return to dock queued", true);
  }

  void RequestDockReturn(
      const std::string& reason,
      const std::shared_ptr<std_srvs::srv::Trigger::Response>& response) {
    std::string message;
    const auto dock_profile = LoadDockProfile(&message);
    robot_tasks::DockReturnRequest request;
    request.dock_profile_available = dock_profile.has_value();
    request.load_error_message = message;
    request.dock_already_requested_or_running =
        dock_profile.has_value() && DockAlreadyRequestedOrRunning(dock_profile->mission_id);
    request.mission_active = RunnerBusy();
    const auto decision = robot_tasks::PlanDockReturn(request);

    if (decision.set_error_state) {
      SetState("ERROR", decision.error_message, false);
      if (response != nullptr) {
        response->success = decision.response_success;
        response->message = decision.response_message;
      }
      return;
    }
    if (decision.branch == "already_requested") {
      if (response != nullptr) {
        response->success = decision.response_success;
        response->message = decision.response_message;
      }
      return;
    }

    if (decision.queue_dock_mission) {
      QueueDockMission(*dock_profile, reason);
      if (decision.cancel_active_mission) {
        RequestActiveMissionCancel(reason + ": canceling active mission before docking");
      }
      if (response != nullptr) {
        response->success = decision.response_success;
        response->message = decision.response_message;
      }
      return;
    }

    if (decision.dispatch_dock_mission) {
      profile_ = *dock_profile;
      mission_file_ = dock_mission_file_;
      queue_running_ = false;
      queue_start_requested_ = false;
      DispatchMission(response, "return to dock mission sent");
    }
  }

  void HandleChassisState(const robot_interfaces_core::msg::ChassisState& state) {
    current_battery_voltage_ = state.battery_voltage;
    robot_tasks::LowBatteryDockRequest request;
    request.enabled = return_to_dock_on_low_battery_;
    request.battery_voltage = state.battery_voltage;
    request.threshold_voltage = dock_low_battery_voltage_;
    request.request_already_sent = low_battery_dock_request_sent_;
    const auto decision = robot_tasks::PlanLowBatteryDock(request);
    low_battery_dock_request_sent_ = decision.low_battery_request_sent;
    if (decision.request_dock_return) {
      RequestDockReturn(decision.reason, nullptr);
    }
  }

  bool MatchesActiveStationTransportContext(
      const StationTransportDispatchContext& context) const {
    if (context.mission_id.empty() ||
        context.mission_id != active_station_transport_id_) {
      return false;
    }
    const auto it = station_transport_executions_.find(context.mission_id);
    return it != station_transport_executions_.end() &&
           it->second.state.active_leg == context.leg &&
           it->second.state.dispatch_token == context.dispatch_token;
  }

  void ApplyStationTransportPostActions(const bool mission_succeeded) {
    robot_tasks::MissionResultWorkflowDecision result;
    result.mission_succeeded = mission_succeeded;
    ApplyMissionResultPostActions(result, false, LoadDockProfile(nullptr));
  }

  void FinishStationTransportTerminal(
      const std::string& mission_id, const std::string& state,
      const std::string& message, const bool mission_succeeded) {
    active_station_transport_id_.clear();
    cancellation_requested_ = false;
    paused_ = false;
    current_goal_handle_.reset();
    mission_active_ = false;
    SetState(state, message, true);
    station_transport_executions_.erase(mission_id);
    ApplyStationTransportPostActions(mission_succeeded);
  }

  void CompleteStationTransportSuccess(
      const std::string& mission_id, const std::string& message) {
    ReleaseAllResourcesForMission(mission_id);
    FinishStationTransportTerminal(mission_id, "FINISHED", message, true);
  }

  void HandleStationTransportNavigationResult(
      const StationTransportDispatchContext& context, const bool succeeded,
      const bool canceled, const std::string& result_message) {
    if (!MatchesActiveStationTransportContext(context)) {
      RCLCPP_WARN(
          get_logger(), "ignoring stale station transport result for %s leg %s token %llu",
          context.mission_id.c_str(),
          robot_tasks::StationTransportLegName(context.leg),
          static_cast<unsigned long long>(context.dispatch_token));
      return;
    }

    auto it = station_transport_executions_.find(context.mission_id);
    const auto transition = robot_tasks::PlanStationTransportNavigationResult(
        it->second.state, context.leg, context.dispatch_token, succeeded, canceled);
    if (!transition.accepted) {
      RCLCPP_WARN(
          get_logger(), "station transport result rejected: %s",
          transition.message.c_str());
      return;
    }

    it->second.state = transition.next_state;
    mission_active_ = false;
    paused_ = false;
    current_goal_handle_.reset();
    cancellation_requested_ = false;

    const std::string message =
        result_message.empty() ? transition.message : result_message;
    if (transition.release_resources) {
      ReleaseAllResourcesForMission(context.mission_id);
    }

    switch (transition.next_state.phase) {
      case robot_tasks::StationTransportPhase::kWaitLoad:
        SetState(
            "WAIT_LOAD",
            "arrived at pickup station " +
                transition.next_state.pickup_station_id +
                "; waiting for load confirmation",
            true);
        return;
      case robot_tasks::StationTransportPhase::kWaitUnload:
        SetState(
            "WAIT_UNLOAD",
            "arrived at dropoff station " +
                transition.next_state.dropoff_station_id +
                "; waiting for unload confirmation",
            true);
        return;
      case robot_tasks::StationTransportPhase::kNeedsOperator:
        SetState("NEEDS_OPERATOR", message, false);
        return;
      case robot_tasks::StationTransportPhase::kFailed:
        FinishStationTransportTerminal(
            context.mission_id, "FAILED", message, false);
        return;
      case robot_tasks::StationTransportPhase::kCanceled:
        FinishStationTransportTerminal(
            context.mission_id, "CANCELED", message, false);
        return;
      default:
        SetState("ERROR", "unexpected station transport navigation result", false);
        return;
    }
  }

  void SendStationTransportGoal(
      const NavigateSequence::Goal& goal,
      const StationTransportDispatchContext& context) {
    auto options = rclcpp_action::Client<NavigateSequence>::SendGoalOptions();
    options.goal_response_callback =
        [this, context](GoalHandleNavigateSequence::SharedPtr goal_handle) {
          if (!MatchesActiveStationTransportContext(context)) {
            if (goal_handle) {
              action_client_->async_cancel_goal(goal_handle);
            }
            return;
          }
          if (!goal_handle) {
            HandleStationTransportNavigationResult(
                context, false, cancellation_requested_,
                "navigate sequence rejected station transport leg");
            return;
          }
          current_goal_handle_ = goal_handle;
          if (cancellation_requested_) {
            action_client_->async_cancel_goal(goal_handle);
            return;
          }
          const auto* transport = FindActiveStationTransport();
          SetState(
              transport == nullptr
                  ? "ERROR"
                  : robot_tasks::StationTransportPhaseName(
                        transport->state.phase),
              std::string("station transport ") +
                  robot_tasks::StationTransportLegName(context.leg) +
                  " navigation accepted",
              true);
        };
    options.result_callback =
        [this, context](const GoalHandleNavigateSequence::WrappedResult& result) {
          if (!MatchesActiveStationTransportContext(context)) {
            return;
          }
          const bool succeeded =
              result.code == rclcpp_action::ResultCode::SUCCEEDED &&
              result.result && result.result->success;
          const bool canceled =
              result.code == rclcpp_action::ResultCode::CANCELED ||
              cancellation_requested_;
          HandleStationTransportNavigationResult(
              context, succeeded, canceled,
              result.result ? result.result->message : "");
        };
    options.feedback_callback =
        [this, context](
            GoalHandleNavigateSequence::SharedPtr,
            const std::shared_ptr<const NavigateSequence::Feedback> feedback) {
          if (!MatchesActiveStationTransportContext(context)) {
            return;
          }
          current_goal_index_ = feedback->current_goal_index;
          if (feedback->state == "paused") {
            paused_ = true;
            SetState("PAUSED", feedback->message, true);
            return;
          }
          const auto* transport = FindActiveStationTransport();
          if (transport != nullptr && !paused_) {
            SetState(
                robot_tasks::StationTransportPhaseName(transport->state.phase),
                feedback->state + ": " + feedback->message, true);
          }
        };

    mission_active_ = true;
    autostart_sent_ = true;
    action_client_->async_send_goal(goal, options);
  }

  bool DispatchStationTransportLeg(
      const std::string& mission_id,
      const robot_tasks::StationTransportLeg leg, std::string* message) {
    const auto it = station_transport_executions_.find(mission_id);
    if (it == station_transport_executions_.end()) {
      if (message != nullptr) {
        *message = "station transport execution not found: " + mission_id;
      }
      return false;
    }

    const std::uint64_t dispatch_token = it->second.state.dispatch_token + 1U;
    const auto transition = robot_tasks::PlanStationTransportDispatch(
        it->second.state, leg, dispatch_token);
    if (!transition.accepted) {
      if (message != nullptr) {
        *message = transition.message;
      }
      return false;
    }

    it->second.state = transition.next_state;
    active_station_transport_id_ = mission_id;
    profile_ = leg == robot_tasks::StationTransportLeg::kPickup
                   ? it->second.pickup_profile
                   : it->second.dropoff_profile;
    paused_ = false;
    cancellation_requested_ = false;
    current_goal_handle_.reset();

    const StationTransportDispatchContext context{
        mission_id, leg, dispatch_token};
    SetState(
        robot_tasks::StationTransportPhaseName(it->second.state.phase),
        transition.message, true);

    if (!action_client_->wait_for_action_server(
            std::chrono::milliseconds(server_timeout_ms_))) {
      HandleStationTransportNavigationResult(
          context, false, false,
          "navigate sequence action server unavailable");
      if (message != nullptr) {
        *message = "navigate sequence action server unavailable";
      }
      return false;
    }

    NavigateSequence::Goal goal;
    goal.loop = false;
    for (const auto& waypoint : profile_->waypoints) {
      goal.goals.push_back(
          PoseStampedFromWaypoint(waypoint, profile_->frame_id, now()));
    }
    SendStationTransportGoal(goal, context);
    if (message != nullptr) {
      *message = transition.message;
    }
    return true;
  }

  void DispatchMission(const std::shared_ptr<std_srvs::srv::Trigger::Response>& response,
                       const std::string& accepted_message) {
    paused_ = false;
    cancellation_requested_ = false;
    current_goal_handle_.reset();
    if (!profile_.has_value()) {
      LoadProfile(nullptr);
    }
    if (!profile_.has_value()) {
      if (response != nullptr) {
        response->success = false;
        response->message = last_message_;
      }
      return;
    }
    if (active_recovery_mission_id_ != profile_->mission_id) {
      active_recovery_mission_id_ = profile_->mission_id;
      current_failure_retry_count_ = 0;
    }
    if (!action_client_->wait_for_action_server(std::chrono::milliseconds(server_timeout_ms_))) {
      SetState("ERROR", "navigate sequence action server unavailable", true);
      if (response != nullptr) {
        response->success = false;
        response->message = last_message_;
      }
      return;
    }

    NavigateSequence::Goal goal;
    goal.loop = profile_->loop;
    for (const auto& waypoint : profile_->waypoints) {
      geometry_msgs::msg::PoseStamped pose;
      pose.header.stamp = now();
      pose.header.frame_id = profile_->frame_id;
      pose.pose.position.x = waypoint.x;
      pose.pose.position.y = waypoint.y;
      pose.pose.orientation = QuaternionFromYaw(waypoint.yaw);
      goal.goals.push_back(pose);
    }
    SendMissionGoal(goal, response, accepted_message);
  }
