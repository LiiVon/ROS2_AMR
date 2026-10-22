
  bool ValidateDockReferences(
      const robot_tasks::Dock& dock, const robot_tasks::FacilityCatalog& facilities,
      const robot_tasks::StationCatalog& stations, std::string* message) const {
    const auto* charger = robot_tasks::FindFacilityResource(facilities, dock.charger_resource_id);
    if (charger == nullptr || charger->type != "charger") {
      if (message != nullptr) {
        *message = "dock charger resource not found or not charger: " + dock.charger_resource_id;
      }
      return false;
    }
    if (robot_tasks::FindStation(stations, dock.approach_station_id) == nullptr) {
      if (message != nullptr) {
        *message = "dock approach station not found: " + dock.approach_station_id;
      }
      return false;
    }
    return true;
  }

  template <typename RequestT, typename ResponseT>
  void RequestDocking(
      const std::shared_ptr<RequestT>& request,
      const std::shared_ptr<ResponseT>& response) {
    std::string message;
    const auto docks = LoadDocks(&message);
    const auto facilities = LoadFacilities(&message);
    const auto stations = robot_tasks::LoadStationCatalog(station_file_);
    if (!docks.has_value() || !facilities.has_value() || !stations.has_value()) {
      response->success = false;
      response->message = docks.has_value() && facilities.has_value()
                              ? "failed to load station catalog: " + station_file_
                              : message;
      response->queue_size = static_cast<int>(mission_queue_.size());
      return;
    }
    const auto* dock = ResolveDock(*docks, SanitizeId(request->dock_id), &message);
    if (dock == nullptr) {
      response->success = false;
      response->message = message;
      response->queue_size = static_cast<int>(mission_queue_.size());
      return;
    }
    response->dock_id = dock->id;
    const auto dock_projection = robot_tasks::BuildDockStateProjection(
        *dock, dock_states_, facility_reservations_);
    const bool dock_references_valid = ValidateDockReferences(*dock, *facilities, *stations, &message);
    const auto dock_gate = robot_tasks::PlanDockingRequestGate(
        robot_tasks::DockingRequestGateRequest{
            dock->id, dock_projection.enabled, dock_projection.available, dock_projection.state,
            dock_references_valid, message});
    if (!dock_gate.accepted) {
      response->success = false;
      response->state = dock_gate.state;
      response->message = dock_gate.message;
      response->queue_size = static_cast<int>(mission_queue_.size());
      return;
    }

    const auto request_id = SanitizeId(request->request_id);
    const std::string mission_id =
        request_id.empty() ? "docking_request_" + dock->id + "_" +
                                 std::to_string(next_docking_request_sequence_++)
                           : "docking_request_" + request_id;
    if (MissionIdActiveOrQueued(mission_id)) {
      const auto duplicate_gate = robot_tasks::PlanDockingRequestGate(
          robot_tasks::DockingRequestGateRequest{
              dock->id, true, true, dock_projection.state, true, "", mission_id, true});
      response->success = false;
      response->mission_id = duplicate_gate.mission_id;
      response->state = duplicate_gate.state;
      response->message = duplicate_gate.message;
      response->queue_size = static_cast<int>(mission_queue_.size());
      return;
    }
    const auto* approach_station = robot_tasks::FindStation(*stations, dock->approach_station_id);
    auto profile = robot_tasks::BuildStationWaypointProfile(mission_id, *approach_station);
    const auto preflight = ValidateMissionForQueue(profile);
    const auto preflight_gate = robot_tasks::PlanDockingRequestGate(
        robot_tasks::DockingRequestGateRequest{
            dock->id, true, true, dock_projection.state, true, "", mission_id, false, preflight});
    if (!preflight_gate.accepted) {
      response->success = false;
      response->mission_id = preflight_gate.mission_id;
      response->state = preflight_gate.state;
      response->message = preflight_gate.message;
      response->queue_size = static_cast<int>(mission_queue_.size());
      return;
    }

    facility_reservations_[dock->charger_resource_id] =
        robot_tasks::ResourceReservation{mission_id, mission_id, "APPROACHING", true};
    facility_resource_by_mission_[mission_id] = dock->charger_resource_id;
    dock_states_[dock->id] =
        DockRuntimeState{dock->id, mission_id, "APPROACHING", request->simulate_contact_success, 0};
    dock_by_mission_[mission_id] = dock->id;
    const auto admission = AdmitQueuedMission(
        QueuedMission{profile, "docking:" + dock->id, request->priority,
                      next_queue_sequence_++},
        request->start_if_idle, request->preempt_current,
        "preempting active mission for docking " + mission_id,
        "queued docking request " + mission_id + " for dock " + dock->id);
    response->success = admission.success;
    response->mission_id = mission_id;
    response->state = "APPROACHING";
    response->queue_size = admission.queue_size;
    response->message = admission.message;
    ApplyMissionQueueAdmissionState(admission);
  }

  template <typename RequestT, typename ResponseT>
  void RequestUndocking(
      const std::shared_ptr<RequestT>& request,
      const std::shared_ptr<ResponseT>& response) {
    std::string message;
    const auto catalog = LoadDocks(&message);
    if (!catalog.has_value()) {
      response->success = false;
      response->message = message;
      return;
    }
    const auto* dock = ResolveDock(*catalog, SanitizeId(request->dock_id), &message);
    if (dock == nullptr) {
      response->success = false;
      response->message = message;
      return;
    }
    const auto decision = robot_tasks::PlanUndocking(
        robot_tasks::UndockingRequest{
            dock->id, dock->charger_resource_id, SanitizeId(request->request_id),
            request->release_charger},
        dock_states_, facility_reservations_);
    robot_tasks::ApplyUndockingDecision(
        decision, dock->charger_resource_id, dock_states_, facility_reservations_,
        facility_resource_by_mission_);
    RecordMissionEventFor(
        decision.mission_id, decision.state, decision.previous_state, decision.message, true);
    response->success = decision.success;
    response->dock_id = dock->id;
    response->state = decision.state;
    response->message = decision.message;
  }

  robot_tasks::PayloadWorkflowState CurrentPayloadWorkflowState() const {
    robot_tasks::PayloadWorkflowState state;
    state.loaded = payload_loaded_;
    state.payload_id = payload_id_;
    state.state = payload_state_;
    state.last_action = payload_last_action_;
    state.message = payload_message_;
    state.weight_kg = payload_weight_kg_;
    state.capacity_kg = payload_capacity_kg_;
    state.top_module_type = top_module_type_;
    return state;
  }

  void ApplyPayloadWorkflowDecision(
      const robot_tasks::PayloadWorkflowDecision& decision) {
    if (!decision.mutate_state) {
      return;
    }
    payload_loaded_ = decision.loaded;
    payload_id_ = decision.payload_id;
    payload_weight_kg_ = decision.weight_kg;
    payload_state_ = decision.state;
    payload_last_action_ = decision.last_action;
    payload_message_ = decision.message;
  }

  void RecordPayloadWorkflowEvent(
      const robot_tasks::PayloadWorkflowDecision& decision) {
    if (decision.event_mission_id.empty()) {
      return;
    }
    RecordMissionEventFor(
        decision.event_mission_id, decision.event_state,
        decision.event_previous_state, decision.event_message,
        decision.event_recoverable);
  }

  template <typename RequestT, typename ResponseT>
  void ConfirmLoad(
      const std::shared_ptr<RequestT>& request,
      const std::shared_ptr<ResponseT>& response) {
    const auto requested_payload_id = SanitizeId(request->payload_id);
    const auto station_id = SanitizeId(request->station_id);

    auto* transport = FindActiveStationTransport();
    if (transport != nullptr) {
      if (paused_) {
        response->success = false;
        response->message =
            "station transport is paused; resume it before confirming load";
        response->payload_id = payload_id_;
        response->state = payload_state_;
        response->loaded = payload_loaded_;
        response->weight_kg = payload_weight_kg_;
        return;
      }
      const auto transport_decision = robot_tasks::PlanStationTransportConfirmLoad(
          transport->state, station_id, requested_payload_id);
      if (!transport_decision.accepted) {
        PublishPayloadState();
        response->success = false;
        response->message = transport_decision.message;
        response->payload_id = payload_id_;
        response->state = payload_state_;
        response->loaded = payload_loaded_;
        response->weight_kg = payload_weight_kg_;
        return;
      }

      const auto payload_decision = robot_tasks::PlanConfirmLoad(
          CurrentPayloadWorkflowState(), requested_payload_id, station_id,
          request->weight_kg);
      if (!payload_decision.success) {
        PublishPayloadState();
        response->success = false;
        response->message = payload_decision.message;
        response->payload_id = payload_decision.payload_id;
        response->state = payload_decision.state;
        response->loaded = payload_decision.loaded;
        response->weight_kg = payload_decision.weight_kg;
        return;
      }

      ApplyPayloadWorkflowDecision(payload_decision);
      RecordPayloadWorkflowEvent(payload_decision);
      transport->state = transport_decision.next_state;
      PublishPayloadState();

      std::string dispatch_message;
      const bool dispatched = DispatchStationTransportLeg(
          transport->state.mission_id, robot_tasks::StationTransportLeg::kDropoff,
          &dispatch_message);
      response->success = dispatched;
      response->message =
          payload_decision.message + "; " +
          (dispatched ? dispatch_message : "dropoff dispatch failed: " + dispatch_message);
      response->payload_id = payload_id_;
      response->state = payload_state_;
      response->loaded = payload_loaded_;
      response->weight_kg = payload_weight_kg_;
      return;
    }

    const auto payload_id = requested_payload_id.empty() ? "payload" : requested_payload_id;
    const auto decision = robot_tasks::PlanConfirmLoad(
        CurrentPayloadWorkflowState(), payload_id, station_id, request->weight_kg);
    if (!decision.success) {
      PublishPayloadState();
      response->success = false;
      response->message = decision.message;
      response->payload_id = decision.payload_id;
      response->state = decision.state;
      response->loaded = decision.loaded;
      response->weight_kg = decision.weight_kg;
      return;
    }
    ApplyPayloadWorkflowDecision(decision);
    RecordPayloadWorkflowEvent(decision);
    PublishPayloadState();
    response->success = true;
    response->message = decision.message;
    response->payload_id = decision.payload_id;
    response->state = decision.state;
    response->loaded = decision.loaded;
    response->weight_kg = decision.weight_kg;
  }

  template <typename RequestT, typename ResponseT>
  void ConfirmUnload(
      const std::shared_ptr<RequestT>& request,
      const std::shared_ptr<ResponseT>& response) {
    const auto requested_payload_id = SanitizeId(request->payload_id);
    const auto station_id = SanitizeId(request->station_id);

    auto* transport = FindActiveStationTransport();
    if (transport != nullptr) {
      if (paused_) {
        response->success = false;
        response->message =
            "station transport is paused; resume it before confirming unload";
        response->payload_id = payload_id_;
        response->state = payload_state_;
        response->loaded = payload_loaded_;
        return;
      }
      const auto transport_decision = robot_tasks::PlanStationTransportConfirmUnload(
          transport->state, station_id, requested_payload_id);
      if (!transport_decision.accepted) {
        response->success = false;
        response->message = transport_decision.message;
        response->payload_id = payload_id_;
        response->state = payload_state_;
        response->loaded = payload_loaded_;
        return;
      }

      const auto payload_decision = robot_tasks::PlanConfirmUnload(
          CurrentPayloadWorkflowState(), requested_payload_id, station_id);
      if (!payload_decision.success) {
        response->success = false;
        response->message = payload_decision.message;
        response->payload_id = payload_decision.payload_id;
        response->state = payload_decision.state;
        response->loaded = payload_decision.loaded;
        return;
      }

      const std::string completed_mission_id = transport->state.mission_id;
      ApplyPayloadWorkflowDecision(payload_decision);
      RecordPayloadWorkflowEvent(payload_decision);
      transport->state = transport_decision.next_state;
      PublishPayloadState();
      CompleteStationTransportSuccess(completed_mission_id, transport_decision.message);

      response->success = true;
      response->message = payload_decision.message + "; station transport completed";
      response->payload_id = requested_payload_id;
      response->state = payload_state_;
      response->loaded = payload_loaded_;
      return;
    }

    const auto decision = robot_tasks::PlanConfirmUnload(
        CurrentPayloadWorkflowState(), requested_payload_id, station_id);
    if (!decision.success) {
      response->success = false;
      response->message = decision.message;
      response->payload_id = decision.payload_id;
      response->state = decision.state;
      response->loaded = decision.loaded;
      return;
    }
    const auto response_payload_id =
        decision.mutate_state && !decision.event_mission_id.empty()
            ? decision.event_mission_id.substr(std::string("payload_").size())
            : decision.payload_id;
    ApplyPayloadWorkflowDecision(decision);
    RecordPayloadWorkflowEvent(decision);
    PublishPayloadState();
    RequestPendingStationTransportStartAfterPayloadCleared();
    response->success = true;
    response->message = decision.message;
    response->payload_id = response_payload_id;
    response->state = decision.state;
    response->loaded = decision.loaded;
  }

  template <typename RequestT, typename ResponseT>
  void SetPayloadLoaded(
      const std::shared_ptr<RequestT>& request,
      const std::shared_ptr<ResponseT>& response) {
    auto* transport = FindActiveStationTransport();
    if (transport != nullptr &&
        transport->state.phase == robot_tasks::StationTransportPhase::kNeedsOperator &&
        !request->loaded) {
      const auto decision = robot_tasks::PlanSetPayloadLoaded(
          CurrentPayloadWorkflowState(), false, "", 0.0, request->message);
      if (!decision.success) {
        response->success = false;
        response->message = decision.message;
        response->payload_id = decision.payload_id;
        response->state = decision.state;
        response->loaded = decision.loaded;
        response->weight_kg = decision.weight_kg;
        return;
      }

      const std::string mission_id = transport->state.mission_id;
      ApplyPayloadWorkflowDecision(decision);
      RecordPayloadWorkflowEvent(decision);
      transport->state.payload_loaded = false;
      transport->state.payload_id.clear();
      transport->state.phase = robot_tasks::StationTransportPhase::kCanceled;
      PublishPayloadState();
      ReleaseAllResourcesForMission(mission_id);
      FinishStationTransportTerminal(
          mission_id, "CANCELED",
          "operator cleared payload after station transport failure", false);

      response->success = true;
      response->message = "payload cleared and failed station transport canceled";
      response->payload_id = payload_id_;
      response->state = payload_state_;
      response->loaded = payload_loaded_;
      response->weight_kg = payload_weight_kg_;
      return;
    }

    if (transport != nullptr ||
        (request->loaded && HasPendingStationTransport())) {
      response->success = false;
      response->message =
          "set_payload_loaded cannot bypass an active or queued station transport";
      response->payload_id = payload_id_;
      response->state = payload_state_;
      response->loaded = payload_loaded_;
      response->weight_kg = payload_weight_kg_;
      return;
    }
    const auto payload_id = SanitizeId(request->payload_id.empty() ? "payload" : request->payload_id);
    const auto decision = robot_tasks::PlanSetPayloadLoaded(
        CurrentPayloadWorkflowState(), request->loaded, payload_id,
        request->weight_kg, request->message);
    if (!decision.success) {
      response->success = false;
      response->message = decision.message;
      response->payload_id = decision.payload_id;
      response->state = decision.state;
      response->loaded = decision.loaded;
      response->weight_kg = decision.weight_kg;
      return;
    }
    ApplyPayloadWorkflowDecision(decision);
    RecordPayloadWorkflowEvent(decision);
    PublishPayloadState();
    RequestPendingStationTransportStartAfterPayloadCleared();
    response->success = true;
    response->message = decision.message;
    response->payload_id = decision.payload_id;
    response->state = decision.state;
    response->loaded = decision.loaded;
    response->weight_kg = decision.weight_kg;
  }

  template <typename RequestT, typename ResponseT>
  void ExecuteTopModuleAction(
      const std::shared_ptr<RequestT>& request,
      const std::shared_ptr<ResponseT>& response) {
    const auto action = SanitizeId(request->action.empty() ? "inspect" : request->action);
    const auto decision =
        robot_tasks::PlanTopModuleAction(CurrentPayloadWorkflowState(), action);
    if (!decision.success) {
      response->success = false;
      response->action = action;
      response->state = decision.state;
      response->loaded = decision.loaded;
      response->payload_id = decision.payload_id;
      response->message = decision.message;
      return;
    }
    ApplyPayloadWorkflowDecision(decision);
    RecordPayloadWorkflowEvent(decision);
    PublishPayloadState();
    response->success = true;
    response->message = decision.message;
    response->action = action;
    response->state = decision.state;
    response->loaded = decision.loaded;
    response->payload_id = decision.payload_id;
  }

  robot_tasks::MissionLocalizationState CurrentLocalizationState() const {
    robot_tasks::MissionLocalizationState state;
    state.localized = localization_localized_;
    state.state = localization_state_;
    state.station_id = localization_station_id_;
    state.message = localization_message_;
    state.covariance_xy = localization_covariance_xy_;
    state.paused_mission = localization_paused_mission_;
    return state;
  }

  void ApplyLocalizationDecision(
      const robot_tasks::MissionLocalizationDecision& decision) {
    localization_localized_ = decision.localization.localized;
    localization_state_ = decision.localization.state;
    localization_station_id_ = decision.localization.station_id;
    localization_message_ = decision.localization.message;
    localization_covariance_xy_ = decision.localization.covariance_xy;
    localization_paused_mission_ = decision.localization.paused_mission;

    if (decision.event.emit) {
      RecordMissionEventFor(
          decision.event.mission_id, decision.event.state, decision.event.previous_state,
          decision.event.message, decision.event.recoverable);
    }
    if (decision.side_effects.request_pause) {
      paused_ = true;
      SendControlRequest<std_srvs::srv::Trigger>(pause_client_, pause_service_name_);
    }
    if (decision.side_effects.request_resume) {
      paused_ = false;
      SendControlRequest<std_srvs::srv::Trigger>(resume_client_, resume_service_name_);
    }
    if (decision.side_effects.set_runner_state) {
      SetState(
          decision.side_effects.runner_state, decision.side_effects.runner_message,
          decision.side_effects.runner_recoverable);
    }
  }

  void HandleLocalizationPose(const geometry_msgs::msg::PoseWithCovarianceStamped& pose) {
    const auto decision = robot_tasks::PlanLocalizationPoseUpdate(
        robot_tasks::MissionLocalizationPoseUpdateRequest{
            CurrentLocalizationState(), pose.pose.covariance[0], pose.pose.covariance[7],
            localization_covariance_threshold_, localization_pause_on_lost_, mission_active_,
            paused_});
    ApplyLocalizationDecision(decision);
    PublishLocalizationHealth();
  }

  template <typename RequestT, typename ResponseT>
  void RequestRelocalization(
      const std::shared_ptr<RequestT>& request, const std::shared_ptr<ResponseT>& response) {
    const auto decision = robot_tasks::PlanRelocalizationRequest(
        robot_tasks::MissionRelocalizationRequest{CurrentLocalizationState(), request->reason});
    ApplyLocalizationDecision(decision);
    PublishLocalizationHealth();
    response->success = true;
    response->message = localization_message_;
    response->state = localization_state_;
    response->localized = localization_localized_;
  }

  template <typename RequestT, typename ResponseT>
  void SetInitialPoseFromStation(
      const std::shared_ptr<RequestT>& request, const std::shared_ptr<ResponseT>& response) {
    const auto station_id = SanitizeId(request->station_id);
    const auto stations = robot_tasks::LoadStationCatalog(station_file_);
    if (!stations.has_value()) {
      response->success = false;
      response->message = "failed to load station catalog: " + station_file_;
      response->station_id = station_id;
      response->state = localization_state_;
      response->localized = localization_localized_;
      return;
    }
    const auto* station = robot_tasks::FindStation(*stations, station_id);
    if (station == nullptr) {
      response->success = false;
      response->message = "unknown station: " + station_id;
      response->station_id = station_id;
      response->state = localization_state_;
      response->localized = localization_localized_;
      return;
    }
    geometry_msgs::msg::PoseWithCovarianceStamped pose;
    pose.header.stamp = now();
    pose.header.frame_id = station->frame_id.empty() ? "map" : station->frame_id;
    pose.pose.pose.position.x = station->x;
    pose.pose.pose.position.y = station->y;
    pose.pose.pose.orientation = QuaternionFromYaw(station->yaw);
    pose.pose.covariance[0] = 0.05;
    pose.pose.covariance[7] = 0.05;
    pose.pose.covariance[35] = 0.05;
    initial_pose_publisher_->publish(pose);
    const auto decision = robot_tasks::PlanInitialPoseRecovery(
        robot_tasks::MissionInitialPoseRecoveryRequest{
            CurrentLocalizationState(), station->id, mission_active_, paused_, 0.05});
    ApplyLocalizationDecision(decision);
    PublishLocalizationHealth();
    response->success = true;
    response->message = localization_message_;
    response->station_id = station->id;
    response->state = localization_state_;
    response->localized = localization_localized_;
  }

  robot_tasks::MissionSafetyState CurrentMissionSafetyState() const {
    robot_tasks::MissionSafetyState state;
    state.obstacle_blocked = safety_obstacle_blocked_;
    state.blockage_paused_mission = safety_blockage_paused_mission_;
    state.runtime_speed_limit_mps = safety_runtime_speed_limit_mps_;
    state.zone_speed_limit_mps = safety_zone_speed_limit_mps_;
    state.stop_reason = safety_stop_reason_;
    state.blockage_id = safety_blockage_id_;
    state.mission_id = safety_mission_id_;
    state.message = safety_message_;
    return state;
  }

  double EffectiveSafetySpeedLimit() const {
    return robot_tasks::EffectiveSafetySpeedLimit(CurrentMissionSafetyState());
  }

  std::string SafetyStateName() const {
    return robot_tasks::MissionSafetyStateName(CurrentMissionSafetyState());
  }

  void ApplyMissionSafetyDecision(
      const robot_tasks::MissionSafetyDecision& decision) {
    if (decision.mutate_state) {
      safety_obstacle_blocked_ = decision.safety.obstacle_blocked;
      safety_blockage_paused_mission_ = decision.safety.blockage_paused_mission;
      safety_runtime_speed_limit_mps_ = decision.safety.runtime_speed_limit_mps;
      safety_zone_speed_limit_mps_ = decision.safety.zone_speed_limit_mps;
      safety_stop_reason_ = decision.safety.stop_reason;
      safety_blockage_id_ = decision.safety.blockage_id;
      safety_mission_id_ = decision.safety.mission_id;
      safety_message_ = decision.safety.message;
    }
    if (decision.event.emit) {
      RecordMissionEventFor(
          decision.event.mission_id, decision.event.state, decision.event.previous_state,
          decision.event.message, decision.event.recoverable);
    }
    if (decision.side_effects.request_pause) {
      paused_ = true;
      SendControlRequest<std_srvs::srv::Trigger>(pause_client_, pause_service_name_);
    }
    if (decision.side_effects.request_resume) {
      paused_ = false;
      SendControlRequest<std_srvs::srv::Trigger>(resume_client_, resume_service_name_);
    }
    if (decision.side_effects.set_runner_state) {
      SetState(
          decision.side_effects.runner_state, decision.side_effects.runner_message,
          decision.side_effects.runner_recoverable);
    }
  }

  template <typename RequestT, typename ResponseT>
  void SetDynamicSpeedLimit(
      const std::shared_ptr<RequestT>& request, const std::shared_ptr<ResponseT>& response) {
    const auto decision = robot_tasks::PlanSetDynamicSpeedLimit(
        robot_tasks::MissionSafetySpeedLimitRequest{
            CurrentMissionSafetyState(), request->speed_limit_mps, request->reason});
    ApplyMissionSafetyDecision(decision);
    if (decision.mutate_state) {
      PublishSafetyState();
    }

    response->success = decision.success;
    response->message = decision.response_message;
    response->runtime_speed_limit_mps = decision.response_runtime_speed_limit_mps;
    response->effective_speed_limit_mps = decision.response_effective_speed_limit_mps;
  }

  template <typename RequestT, typename ResponseT>
  void ReportObstacleBlockage(
      const std::shared_ptr<RequestT>& request, const std::shared_ptr<ResponseT>& response) {
    const auto blockage_id = SanitizeId(request->blockage_id.empty() ? "obstacle" : request->blockage_id);
    const auto decision = robot_tasks::PlanReportObstacleBlockage(
        robot_tasks::MissionSafetyReportBlockageRequest{
            CurrentMissionSafetyState(), blockage_id, request->reason,
            profile_.has_value() ? profile_->mission_id : "", previous_state_,
            request->pause_active, safety_obstacle_pause_on_blockage_, mission_active_, paused_});
    ApplyMissionSafetyDecision(decision);
    if (decision.mutate_state) {
      PublishSafetyState();
    }

    response->success = decision.success;
    response->message = decision.response_message;
    response->state = decision.response_state;
    response->blockage_id = decision.response_blockage_id;
  }

  template <typename RequestT, typename ResponseT>
  void ClearObstacleBlockage(
      const std::shared_ptr<RequestT>& request, const std::shared_ptr<ResponseT>& response) {
    const auto decision = robot_tasks::PlanClearObstacleBlockage(
        robot_tasks::MissionSafetyClearBlockageRequest{
            CurrentMissionSafetyState(), SanitizeId(request->blockage_id),
            request->resolution, current_state_, request->resume_active, mission_active_,
            paused_});
    ApplyMissionSafetyDecision(decision);
    if (decision.mutate_state) {
      PublishSafetyState();
    }

    response->success = decision.success;
    response->message = decision.response_message;
    response->state = decision.response_state;
  }

  template <typename RequestT, typename ResponseT>
  void ExecuteFacilityAction(
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
    const auto requested_resource_id = SanitizeId(request->resource_id);
    const auto requested_resource_type = SanitizeId(request->resource_type);
    const auto action = SanitizeId(request->action.empty() ? "use" : request->action);
    const auto selection = robot_tasks::SelectFacilityActionResource(
        *facilities, facility_reservations_, requested_resource_id, requested_resource_type);
    const auto* resource = selection.resource;
    if (resource == nullptr) {
      response->success = false;
      response->message = selection.message;
      response->queue_size = static_cast<int>(mission_queue_.size());
      return;
    }
    response->resource_id = resource->id;
    response->action = action;
    if (!robot_tasks::FacilityActionSupported(resource->type, action)) {
      response->success = false;
      response->status = "rejected";
      response->message = "unsupported facility action " + action + " for " + resource->type;
      response->queue_size = static_cast<int>(mission_queue_.size());
      return;
    }
    const auto stations = robot_tasks::LoadStationCatalog(station_file_);
    if (!stations.has_value()) {
      response->success = false;
      response->message = "failed to load station catalog: " + station_file_;
      response->queue_size = static_cast<int>(mission_queue_.size());
      return;
    }
    const auto* station = robot_tasks::FindStation(*stations, resource->station_id);
    if (station == nullptr) {
      response->success = false;
      response->message = "facility station not found: " + resource->station_id;
      response->queue_size = static_cast<int>(mission_queue_.size());
      return;
    }
    const auto request_id = SanitizeId(request->request_id);
    const auto mission_id =
        robot_tasks::BuildFacilityActionMissionId(request_id, resource->id, action);
    const auto duplicate_gate = robot_tasks::PlanMissionSubmitGate(
        robot_tasks::MissionSubmitGateRequest{
            mission_id, "facility action", "facility action",
            MissionIdActiveOrQueued(mission_id)});
    if (!duplicate_gate.accepted) {
      response->success = false;
      response->mission_id = duplicate_gate.mission_id;
      response->status = "duplicate";
      response->message = duplicate_gate.message;
      response->queue_size = static_cast<int>(mission_queue_.size());
      return;
    }
    auto profile = robot_tasks::BuildStationWaypointProfile(mission_id, *station);
    const auto preflight = ValidateMissionForQueue(profile);
    const auto gate = robot_tasks::PlanMissionSubmitGate(
        robot_tasks::MissionSubmitGateRequest{
            mission_id, "facility action", "facility action", false, preflight});
    if (!gate.accepted) {
      response->success = false;
      response->mission_id = gate.mission_id;
      response->status = "rejected";
      response->message = gate.message;
      response->queue_size = static_cast<int>(mission_queue_.size());
      return;
    }

    robot_tasks::ReserveFacilityActionResource(
        facility_reservations_, facility_resource_by_mission_, *resource, mission_id,
        request->hold_after_action);
    const auto admission = AdmitQueuedMission(
        QueuedMission{
            profile, "facility_action:" + resource->id + ":" + action, request->priority,
            next_queue_sequence_++},
        request->start_if_idle, request->preempt_current,
        "preempting active mission for facility action " + mission_id,
        "queued facility action " + action + " for " + resource->id + " at " + station->id);
    response->success = admission.success;
    response->mission_id = mission_id;
    response->status = "reserved";
    response->queue_size = admission.queue_size;
    response->message = admission.message;
    ApplyMissionQueueAdmissionState(admission);
  }

  template <typename RequestT, typename ResponseT>
  void RequestStationConfirmation(
      const std::shared_ptr<RequestT>& request,
      const std::shared_ptr<ResponseT>& response) {
    const auto station_id = SanitizeId(request->station_id);
    const auto action = SanitizeId(request->action.empty() ? "confirm" : request->action);
    auto confirmation_id = SanitizeId(request->confirmation_id);
    if (confirmation_id.empty()) {
      confirmation_id = "station_confirm_" + station_id + "_" + action;
    }
    const auto stations = robot_tasks::LoadStationCatalog(station_file_);
    const auto station_known =
        stations.has_value() && robot_tasks::FindStation(*stations, station_id) != nullptr;
    robot_tasks::StationConfirmationRequest workflow_request;
    workflow_request.confirmation_id = confirmation_id;
    workflow_request.station_id = station_id;
    workflow_request.action = action;
    workflow_request.mission_id = SanitizeId(request->mission_id);
    workflow_request.operator_hint = request->operator_hint;
    workflow_request.timeout_sec = request->timeout_sec;
    workflow_request.timeout_policy =
        SanitizeId(request->timeout_policy.empty() ? "manual" : request->timeout_policy);
    workflow_request.created_at_sec = now().seconds();
    workflow_request.station_known = station_known;
    const auto decision = robot_tasks::PlanStationConfirmationRequest(
        workflow_request, station_confirmations_);
    if (decision.mutate_state) {
      station_confirmations_[decision.confirmation_id] = decision.confirmation;
      RecordMissionEventFor(
          decision.event_mission_id, decision.event_state, decision.event_previous_state,
          decision.event_message, decision.event_recoverable);
    }
    response->success = decision.success;
    response->confirmation_id = decision.confirmation_id;
    response->state = decision.state;
    response->message = decision.message;
  }

  template <typename RequestT, typename ResponseT>
  void GetPendingConfirmations(
      const std::shared_ptr<RequestT>& request,
      const std::shared_ptr<ResponseT>& response)
      const {
    const auto station_filter = SanitizeId(request->station_id_filter);
    const auto projection = robot_tasks::BuildPendingStationConfirmationsProjection(
        station_confirmations_, station_filter);
    response->confirmation_ids = projection.confirmation_ids;
    response->station_ids = projection.station_ids;
    response->actions = projection.actions;
    response->mission_ids = projection.mission_ids;
    response->operator_hints = projection.operator_hints;
    response->states = projection.states;
    response->success = projection.success;
    response->message = projection.message;
  }

  template <typename RequestT, typename ResponseT>
  void ConfirmStationAction(
      const std::shared_ptr<RequestT>& request,
      const std::shared_ptr<ResponseT>& response) {
    ResolveStationConfirmation(
        SanitizeId(request->confirmation_id), "CONFIRMED",
        SanitizeId(request->operator_id.empty() ? "operator" : request->operator_id),
        request->note.empty() ? "station action confirmed" : request->note, response);
  }

  template <typename RequestT, typename ResponseT>
  void RejectStationAction(
      const std::shared_ptr<RequestT>& request,
      const std::shared_ptr<ResponseT>& response) {
    ResolveStationConfirmation(
        SanitizeId(request->confirmation_id), "REJECTED",
        SanitizeId(request->operator_id.empty() ? "operator" : request->operator_id),
        request->reason.empty() ? "station action rejected" : request->reason, response);
  }

  template <typename ResponseT>
  void ResolveStationConfirmation(
      const std::string& confirmation_id, const std::string& state,
      const std::string& operator_id, const std::string& note,
      const std::shared_ptr<ResponseT>& response) {
    const auto decision = ResolveStationConfirmationForRunner(
        confirmation_id, state, operator_id, note);
    response->success = decision.success;
    response->confirmation_id = decision.confirmation_id;
    response->state = decision.state;
    response->message = decision.message;
  }

  void ExpireStationConfirmations() {
    const auto current_time = now().seconds();
    for (auto& [confirmation_id, confirmation] : station_confirmations_) {
      if (confirmation.state != "PENDING" || confirmation.timeout_sec <= 0) {
        continue;
      }
      if (current_time - confirmation.created_at_sec < confirmation.timeout_sec) {
        continue;
      }
      ApplyStationConfirmationTimeout(confirmation_id, confirmation);
    }
  }

  void ApplyStationConfirmationTimeout(
      const std::string& confirmation_id, StationConfirmation& confirmation) {
    confirmation.confirmation_id = confirmation_id;
    const auto decision =
        robot_tasks::PlanStationConfirmationTimeout(confirmation, now().seconds());
    if (!decision.timed_out) {
      return;
    }
    if (decision.update_created_at) {
      confirmation.created_at_sec = decision.new_created_at_sec;
    }
    if (decision.update_state) {
      confirmation.state = decision.state;
    }
    RecordMissionEventFor(
        decision.event_mission_id, decision.event_state, decision.event_previous_state,
        decision.event_message, decision.event_recoverable);
    if (decision.request_dock_return) {
      RequestDockReturn("station confirmation timeout " + confirmation_id, nullptr);
    }
    if (decision.set_runner_state) {
      SetState(
          decision.runner_state, decision.runner_message, decision.runner_recoverable);
    }
  }

  void AddStationConfirmation(
      const std::string& confirmation_id, const std::string& station_id,
      const std::string& action, const std::string& mission_id,
      const std::string& operator_hint) {
    robot_tasks::StationConfirmationRequest workflow_request;
    workflow_request.confirmation_id = confirmation_id;
    workflow_request.station_id = station_id;
    workflow_request.action = action;
    workflow_request.mission_id = mission_id;
    workflow_request.operator_hint = operator_hint;
    workflow_request.created_at_sec = now().seconds();
    workflow_request.station_known = true;
    const auto decision = robot_tasks::PlanStationConfirmationRequest(
        workflow_request, station_confirmations_);
    if (!decision.mutate_state) {
      return;
    }
    station_confirmations_[decision.confirmation_id] = decision.confirmation;
    RecordMissionEventFor(
        decision.event_mission_id, decision.event_state, decision.event_previous_state,
        decision.event_message, decision.event_recoverable);
  }

  bool ResolveFulfillmentConfirmation(
      const std::string& confirmation_id, const std::string& state,
      const std::string& operator_id, const std::string& note) {
    const auto decision = ResolveStationConfirmationForRunner(
        confirmation_id, state, operator_id, note);
    return decision.success;
  }

  robot_tasks::StationConfirmationResolveDecision ResolveStationConfirmationForRunner(
      const std::string& confirmation_id, const std::string& state,
      const std::string& operator_id, const std::string& note) {
    robot_tasks::StationConfirmationResolveRequest request;
    request.confirmation_id = confirmation_id;
    request.state = state;
    request.operator_id = operator_id;
    request.note = note;
    const auto decision =
        robot_tasks::PlanStationConfirmationResolve(request, station_confirmations_);
    if (decision.mutate_state) {
      station_confirmations_[decision.confirmation_id].state = decision.state;
      RecordMissionEventFor(
          decision.event_mission_id, decision.event_state, decision.event_previous_state,
          decision.event_message, decision.event_recoverable);
    }
    return decision;
  }

  std::optional<std::vector<std::string>> QueueFulfillmentStationSequence(
      const std::string& fulfillment_id, const std::vector<std::string>& station_ids,
      const int priority, const bool start_if_idle, std::string* message,
      int* queue_size) {
    if (station_ids.size() < 2U) {
      *message = "fulfillment station sequence requires at least two stations";
      *queue_size = static_cast<int>(mission_queue_.size());
      return std::nullopt;
    }
    auto sequence_request =
        std::make_shared<robot_interfaces_mission::srv::SubmitStationSequenceTask::Request>();
    sequence_request->order_id = fulfillment_id;
    sequence_request->station_ids = station_ids;
    sequence_request->priority = priority;
    sequence_request->start_if_idle = start_if_idle;
    sequence_request->preempt_current = false;
    auto sequence_response =
        std::make_shared<robot_interfaces_mission::srv::SubmitStationSequenceTask::Response>();
    SubmitStationSequenceTask(sequence_request, sequence_response);
    *message = sequence_response->message;
    *queue_size = sequence_response->queue_size;
    if (!sequence_response->success) {
      return std::nullopt;
    }
    return sequence_response->mission_ids;
  }

  bool FulfillmentExists(const std::string& fulfillment_id) const {
    return fulfillment_orders_.find(fulfillment_id) != fulfillment_orders_.end();
  }

  robot_tasks::FulfillmentSubmitDecision PlanFulfillmentSubmitForRunner(
      const std::string& fulfillment_id, const std::string& order_type,
      const std::size_t station_count, const bool require_two_stations) const {
    robot_tasks::FulfillmentSubmitRequest request;
    request.fulfillment_id = fulfillment_id;
    request.order_type = order_type;
    request.station_count = station_count;
    request.duplicate = FulfillmentExists(fulfillment_id);
    request.require_two_stations = require_two_stations;
    return robot_tasks::PlanFulfillmentSubmit(request);
  }

  template <typename ResponseT>
  bool ApplyFulfillmentSubmitDecision(
      const robot_tasks::FulfillmentSubmitDecision& decision, ResponseT* response) const {
    if (decision.success) {
      return true;
    }
    response->success = false;
    response->fulfillment_id = decision.fulfillment_id;
    response->state = decision.state;
    response->queue_size = static_cast<int>(mission_queue_.size());
    response->message = decision.message;
    return false;
  }

  FulfillmentOrder* FindFulfillment(
      const std::string& fulfillment_id, std::string* message) {
    const auto it = fulfillment_orders_.find(fulfillment_id);
    if (it == fulfillment_orders_.end()) {
      *message = "unknown fulfillment order: " + fulfillment_id;
      return nullptr;
    }
    return &it->second;
  }

  template <typename RequestT, typename ResponseT>
  void SubmitPickingWave(
      const std::shared_ptr<RequestT>& request,
      const std::shared_ptr<ResponseT>& response) {
    const auto wave_id = SanitizeId(request->wave_id);
    const auto fulfillment_id = "picking_wave_" + wave_id;
    SubmitFulfillmentOrder(
        fulfillment_id, "picking_wave", SanitizeId(request->sku_id), SanitizeId(request->tote_id),
        SanitizeId(request->pick_station), SanitizeId(request->dropoff_station),
        std::max(1, request->quantity), request->priority, request->start_if_idle,
        response.get());
  }

  template <typename RequestT, typename ResponseT>
  void SubmitReplenishmentOrder(
      const std::shared_ptr<RequestT>& request,
      const std::shared_ptr<ResponseT>& response) {
    const auto order_id = SanitizeId(request->order_id);
    const auto fulfillment_id = "replenishment_" + order_id;
    SubmitFulfillmentOrder(
        fulfillment_id, "replenishment", SanitizeId(request->sku_id),
        SanitizeId(request->tote_id), SanitizeId(request->source_station),
        SanitizeId(request->line_station), std::max(1, request->quantity), request->priority,
        request->start_if_idle, response.get());
  }

  template <typename RequestT, typename ResponseT>
  void SubmitMilkRun(
      const std::shared_ptr<RequestT>& request,
      const std::shared_ptr<ResponseT>& response) {
    const auto run_id = SanitizeId(request->run_id);
    const auto fulfillment_id = "milk_run_" + run_id;
    std::vector<std::string> station_ids;
    station_ids.reserve(request->station_ids.size());
    for (const auto& station_id : request->station_ids) {
      station_ids.push_back(SanitizeId(station_id));
    }
    const auto submit_decision = PlanFulfillmentSubmitForRunner(
        fulfillment_id, "milk_run", station_ids.size(), true);
    if (!ApplyFulfillmentSubmitDecision(submit_decision, response.get())) {
      return;
    }
    std::string message;
    int queue_size = static_cast<int>(mission_queue_.size());
    const auto mission_ids = QueueFulfillmentStationSequence(
        fulfillment_id, station_ids, request->priority, request->start_if_idle, &message,
        &queue_size);
    response->queue_size = queue_size;
    response->fulfillment_id = fulfillment_id;
    if (!mission_ids.has_value()) {
      response->success = false;
      response->state = "REJECTED";
      response->message = message;
      return;
    }
    FulfillmentOrder order;
    order.fulfillment_id = fulfillment_id;
    order.order_type = "milk_run";
    order.tote_id = SanitizeId(request->tote_id);
    order.state = "ROUTE_QUEUED";
    order.pick_station = station_ids.front();
    order.dropoff_station = station_ids.back();
    order.mission_ids = *mission_ids;
    for (const auto& station_id : station_ids) {
      const auto confirmation_id = fulfillment_id + "_" + station_id + "_stop";
      order.confirmation_ids.push_back(confirmation_id);
      AddStationConfirmation(
          confirmation_id, station_id, "milk_run_stop", fulfillment_id,
          "confirm milk run stop for " + order.tote_id);
    }
    fulfillment_orders_[fulfillment_id] = order;
    RecordMissionEventFor(
        fulfillment_id, "FULFILLMENT_ROUTE_QUEUED", "READY",
        "milk run queued with " + std::to_string(station_ids.size()) + " station(s)", true);
    response->success = true;
    response->state = order.state;
    response->mission_ids = order.mission_ids;
    response->message = "milk run queued: " + fulfillment_id;
  }

  template <typename ResponseT>
  void SubmitFulfillmentOrder(
      const std::string& fulfillment_id, const std::string& order_type,
      const std::string& sku_id, const std::string& tote_id,
      const std::string& pick_station, const std::string& dropoff_station,
      const int quantity, const int priority, const bool start_if_idle, ResponseT* response) {
    const auto submit_decision = PlanFulfillmentSubmitForRunner(
        fulfillment_id, order_type, 2, false);
    if (!ApplyFulfillmentSubmitDecision(submit_decision, response)) {
      return;
    }
    if (pick_station.empty() || dropoff_station.empty()) {
      response->success = false;
      response->fulfillment_id = fulfillment_id;
      response->state = "REJECTED";
      response->queue_size = static_cast<int>(mission_queue_.size());
      response->message = "pick station and dropoff station are required";
      return;
    }
    std::string message;
    int queue_size = static_cast<int>(mission_queue_.size());
    const auto mission_ids = QueueFulfillmentStationSequence(
        fulfillment_id, {pick_station, dropoff_station}, priority, start_if_idle, &message,
        &queue_size);
    response->queue_size = queue_size;
    response->fulfillment_id = fulfillment_id;
    if (!mission_ids.has_value()) {
      response->success = false;
      response->state = "REJECTED";
      response->message = message;
      return;
    }
    FulfillmentOrder order;
    order.fulfillment_id = fulfillment_id;
    order.order_type = order_type;
    order.sku_id = sku_id;
    order.tote_id = tote_id;
    order.state = "AWAITING_PICK";
    order.pick_station = pick_station;
    order.dropoff_station = dropoff_station;
    order.requested_quantity = quantity;
    order.mission_ids = *mission_ids;
    order.confirmation_ids = {fulfillment_id + "_pick", fulfillment_id + "_dropoff"};
    AddStationConfirmation(
        order.confirmation_ids[0], pick_station, "pick_" + sku_id, fulfillment_id,
        "pick " + std::to_string(quantity) + " " + sku_id + " into " + tote_id);
    AddStationConfirmation(
        order.confirmation_ids[1], dropoff_station, "dropoff_" + sku_id, fulfillment_id,
        "drop off " + tote_id + " at " + dropoff_station);
    fulfillment_orders_[fulfillment_id] = order;
    RecordMissionEventFor(
        fulfillment_id, "FULFILLMENT_ROUTE_QUEUED", "READY",
        order_type + " queued from " + pick_station + " to " + dropoff_station, true);
    response->success = true;
    response->state = order.state;
    response->mission_ids = order.mission_ids;
    response->message = order_type + " queued: " + fulfillment_id;
  }

  template <typename RequestT, typename ResponseT>
  void ConfirmPick(
      const std::shared_ptr<RequestT>& request,
      const std::shared_ptr<ResponseT>& response) {
    const auto fulfillment_id = SanitizeId(request->fulfillment_id);
    std::string message;
    auto* order = FindFulfillment(fulfillment_id, &message);
    response->fulfillment_id = fulfillment_id;
    if (order == nullptr) {
      response->success = false;
      response->state = "UNKNOWN";
      response->message = message;
      return;
    }
    const auto decision = robot_tasks::PlanFulfillmentPick(
        robot_tasks::FulfillmentPickRequest{
            fulfillment_id, order->order_type, order->state, order->requested_quantity,
            request->picked_quantity});
    if (!decision.success) {
      response->success = false;
      response->state = decision.state;
      response->message = decision.message;
      return;
    }
    order->picked_quantity = decision.picked_quantity;
    order->short_quantity = decision.short_quantity;
    order->state = decision.state;
    if (decision.resolve_pick_confirmation && !order->confirmation_ids.empty()) {
      ResolveFulfillmentConfirmation(
          order->confirmation_ids[0], "CONFIRMED",
          SanitizeId(request->operator_id.empty() ? "operator" : request->operator_id),
          request->note.empty() ? "pick confirmed" : request->note);
    }
    RecordMissionEventFor(
        fulfillment_id, decision.event_state, decision.event_previous_state,
        decision.event_message, decision.event_recoverable);
    response->success = true;
    response->state = decision.state;
    response->picked_quantity = order->picked_quantity;
    response->short_quantity = order->short_quantity;
    response->message = decision.message;
  }

  template <typename RequestT, typename ResponseT>
  void ConfirmDropoff(
      const std::shared_ptr<RequestT>& request,
      const std::shared_ptr<ResponseT>& response) {
    const auto fulfillment_id = SanitizeId(request->fulfillment_id);
    std::string message;
    auto* order = FindFulfillment(fulfillment_id, &message);
    response->fulfillment_id = fulfillment_id;
    if (order == nullptr) {
      response->success = false;
      response->state = "UNKNOWN";
      response->message = message;
      return;
    }
    const auto decision = robot_tasks::PlanFulfillmentDropoff(
        robot_tasks::FulfillmentDropoffRequest{
            fulfillment_id, order->order_type, order->state, order->short_quantity,
            order->confirmation_ids.size()});
    if (!decision.success) {
      response->success = false;
      response->state = decision.state;
      response->message = decision.message;
      return;
    }
    const auto operator_id = SanitizeId(request->operator_id.empty() ? "operator" : request->operator_id);
    const auto note = request->note.empty() ? "dropoff confirmed" : request->note;
    if (decision.resolve_dropoff_confirmation) {
      ResolveFulfillmentConfirmation(order->confirmation_ids[1], "CONFIRMED", operator_id, note);
    } else if (decision.resolve_all_confirmations) {
      for (const auto& confirmation_id : order->confirmation_ids) {
        ResolveFulfillmentConfirmation(confirmation_id, "CONFIRMED", operator_id, note);
      }
    }
    order->state = decision.state;
    RecordMissionEventFor(
        fulfillment_id, decision.event_state, decision.event_previous_state, note,
        decision.event_recoverable);
    response->success = true;
    response->state = decision.state;
    response->message = decision.message;
  }

  template <typename RequestT, typename ResponseT>
  void ReportShortPick(
      const std::shared_ptr<RequestT>& request,
      const std::shared_ptr<ResponseT>& response) {
    const auto fulfillment_id = SanitizeId(request->fulfillment_id);
    std::string message;
    auto* order = FindFulfillment(fulfillment_id, &message);
    response->fulfillment_id = fulfillment_id;
    if (order == nullptr) {
      response->success = false;
      response->state = "UNKNOWN";
      response->message = message;
      return;
    }
    const auto decision = robot_tasks::PlanFulfillmentShortPick(
        robot_tasks::FulfillmentShortPickRequest{
            fulfillment_id, order->order_type, order->state, order->requested_quantity,
            request->short_quantity});
    if (!decision.success) {
      response->success = false;
      response->state = decision.state;
      response->message = decision.message;
      return;
    }
    order->short_quantity = decision.short_quantity;
    order->picked_quantity = decision.picked_quantity;
    order->state = decision.state;
    const auto operator_id = SanitizeId(request->operator_id.empty() ? "operator" : request->operator_id);
    RecordMissionEventFor(
        fulfillment_id, decision.event_state, decision.event_previous_state,
        "short pick " + std::to_string(order->short_quantity) + " reported by " +
            operator_id + ": " + request->reason,
        decision.event_recoverable);
    response->success = true;
    response->state = decision.state;
    response->short_quantity = order->short_quantity;
    response->message = decision.message;
  }

  template <typename RequestT, typename ResponseT>
  void GetOrderFulfillmentStatus(
      const std::shared_ptr<RequestT>& request,
      const std::shared_ptr<ResponseT>& response) {
    const auto fulfillment_id = SanitizeId(request->fulfillment_id);
    std::string message;
    const auto* order = FindFulfillment(fulfillment_id, &message);
    response->fulfillment_id = fulfillment_id;
    if (order == nullptr) {
      response->success = false;
      response->message = message;
      return;
    }
    const auto projection = robot_tasks::BuildFulfillmentStatusProjection(*order);
    response->success = projection.success;
    response->message = projection.message;
    response->order_type = projection.order_type;
    response->sku_id = projection.sku_id;
    response->tote_id = projection.tote_id;
    response->state = projection.state;
    response->pick_station = projection.pick_station;
    response->dropoff_station = projection.dropoff_station;
    response->requested_quantity = projection.requested_quantity;
    response->picked_quantity = projection.picked_quantity;
    response->short_quantity = projection.short_quantity;
    response->mission_ids = projection.mission_ids;
    response->confirmation_ids = projection.confirmation_ids;
  }
