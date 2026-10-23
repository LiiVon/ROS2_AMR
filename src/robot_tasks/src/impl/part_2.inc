/**
 * @file part_2.inc
 * @brief （实现细节）。
 * @details 所属包：`robot_tasks`；所在层次：实现细节。
 * @note 教学说明见 `docs/教学/` 对应章节。
 */

  template <typename RequestT, typename ResponseT>
  void SubmitOrder(
      const std::shared_ptr<RequestT>& request,
      const std::shared_ptr<ResponseT>& response) {
    const auto route_result = robot_tasks::TickSubmitOrderBehaviorTree(
        robot_tasks::SubmitOrderBehaviorTreeInput{robot_tasks::SubmitOrderInput{
            request->order_id, request->order_type, request->priority, request->payload_json,
            request->tags}});
    if (!route_result.success || !route_result.route.has_value()) {
      RejectSubmitOrder(
          response,
          route_result.message.empty() ? "unhandled order_type route" : route_result.message);
      return;
    }
    const auto& route = *route_result.route;

    if (route.kind == robot_tasks::SubmitOrderKind::kTransport) {
      auto typed_request = std::make_shared<robot_interfaces_business::srv::SubmitTransportOrder::Request>();
      typed_request->order_id = route.order_id;
      typed_request->frame_id = route.frame_id;
      typed_request->pickup_x = route.pickup_x;
      typed_request->pickup_y = route.pickup_y;
      typed_request->pickup_yaw = route.pickup_yaw;
      typed_request->dropoff_x = route.dropoff_x;
      typed_request->dropoff_y = route.dropoff_y;
      typed_request->dropoff_yaw = route.dropoff_yaw;
      typed_request->priority = route.priority;
      typed_request->start_if_idle = route.start_if_idle;
      auto typed_response =
          std::make_shared<robot_interfaces_business::srv::SubmitTransportOrder::Response>();
      SubmitTransportOrder(typed_request, typed_response);
      FillSubmitOrderResponse(
          response, typed_response->success, typed_response->message,
          typed_response->mission_id, {}, typed_response->queue_size);
      return;
    }

    if (route.kind == robot_tasks::SubmitOrderKind::kStationTransport) {
      auto typed_request =
          std::make_shared<robot_interfaces_mission::srv::SubmitStationTransportOrder::Request>();
      typed_request->order_id = route.order_id;
      typed_request->pickup_station = route.pickup_station;
      typed_request->dropoff_station = route.dropoff_station;
      typed_request->priority = route.priority;
      typed_request->start_if_idle = route.start_if_idle;
      typed_request->preempt_current = route.preempt_current;
      auto typed_response =
          std::make_shared<robot_interfaces_mission::srv::SubmitStationTransportOrder::Response>();
      SubmitStationTransportOrder(typed_request, typed_response);
      FillSubmitOrderResponse(
          response, typed_response->success, typed_response->message,
          typed_response->mission_id, {}, typed_response->queue_size);
      return;
    }

    if (route.kind == robot_tasks::SubmitOrderKind::kStationOrderBatch) {
      auto typed_request =
          std::make_shared<robot_interfaces_mission::srv::SubmitStationOrderBatch::Request>();
      typed_request->batch_id = route.batch_id;
      typed_request->order_ids = route.order_ids;
      typed_request->pickup_station_ids = route.pickup_station_ids;
      typed_request->dropoff_station_ids = route.dropoff_station_ids;
      typed_request->priority = route.priority;
      typed_request->start_if_idle = route.start_if_idle;
      typed_request->preempt_current = route.preempt_current;
      typed_request->continue_on_error = route.continue_on_error;
      auto typed_response =
          std::make_shared<robot_interfaces_mission::srv::SubmitStationOrderBatch::Response>();
      SubmitStationOrderBatch(typed_request, typed_response);
      FillSubmitOrderResponse(
          response, typed_response->success, typed_response->message, "",
          typed_response->mission_ids, typed_response->queue_size);
      return;
    }

    if (route.kind == robot_tasks::SubmitOrderKind::kStationSequence) {
      auto typed_request =
          std::make_shared<robot_interfaces_mission::srv::SubmitStationSequenceTask::Request>();
      typed_request->order_id = route.order_id;
      typed_request->station_ids = route.station_ids;
      typed_request->priority = route.priority;
      typed_request->start_if_idle = route.start_if_idle;
      typed_request->preempt_current = route.preempt_current;
      auto typed_response =
          std::make_shared<robot_interfaces_mission::srv::SubmitStationSequenceTask::Response>();
      SubmitStationSequenceTask(typed_request, typed_response);
      FillSubmitOrderResponse(
          response, typed_response->success, typed_response->message, "",
          typed_response->mission_ids, typed_response->queue_size);
      return;
    }

    if (route.kind == robot_tasks::SubmitOrderKind::kFleetStation) {
      auto typed_request =
          std::make_shared<robot_interfaces_fleet::srv::SubmitFleetStationTask::Request>();
      typed_request->task_id = route.task_id;
      typed_request->required_capability = route.required_capability;
      typed_request->pickup_station = route.pickup_station;
      typed_request->dropoff_station = route.dropoff_station;
      typed_request->priority = route.priority;
      typed_request->start_if_idle = route.start_if_idle;
      typed_request->preempt_current = route.preempt_current;
      auto typed_response =
          std::make_shared<robot_interfaces_fleet::srv::SubmitFleetStationTask::Response>();
      SubmitFleetStationTask(typed_request, typed_response);
      FillSubmitOrderResponse(
          response, typed_response->success, typed_response->message,
          typed_response->mission_id, {}, typed_response->queue_size);
      return;
    }

    if (route.kind == robot_tasks::SubmitOrderKind::kBusiness) {
      auto typed_request =
          std::make_shared<robot_interfaces_business::srv::SubmitBusinessOrder::Request>();
      typed_request->business_order_id = route.business_order_id;
      typed_request->business_type = route.business_type;
      typed_request->priority = route.priority;
      typed_request->start_if_idle = route.start_if_idle;
      typed_request->preempt_current = route.preempt_current;
      auto typed_response =
          std::make_shared<robot_interfaces_business::srv::SubmitBusinessOrder::Response>();
      SubmitBusinessOrder(typed_request, typed_response);
      FillSubmitOrderResponse(
          response, typed_response->success, typed_response->message, "",
          typed_response->mission_ids, typed_response->queue_size);
      return;
    }

    if (route.kind == robot_tasks::SubmitOrderKind::kScenarioTask) {
      auto typed_request =
          std::make_shared<robot_interfaces_business::srv::SubmitScenarioTask::Request>();
      typed_request->scenario_id = route.scenario_id;
      typed_request->task_id = route.task_id;
      typed_request->order_id = route.order_id;
      typed_request->priority = route.priority;
      typed_request->start_if_idle = route.start_if_idle;
      typed_request->preempt_current = route.preempt_current;
      auto typed_response =
          std::make_shared<robot_interfaces_business::srv::SubmitScenarioTask::Response>();
      SubmitScenarioTask(typed_request, typed_response);
      FillSubmitOrderResponse(
          response, typed_response->success, typed_response->message,
          typed_response->mission_id, {}, typed_response->queue_size);
      return;
    }

    if (route.kind == robot_tasks::SubmitOrderKind::kScenarioWorkflow) {
      auto typed_request =
          std::make_shared<robot_interfaces_business::srv::SubmitScenarioWorkflow::Request>();
      typed_request->scenario_id = route.scenario_id;
      typed_request->workflow_id = route.workflow_id;
      typed_request->order_id = route.order_id;
      typed_request->priority = route.priority;
      typed_request->start_if_idle = route.start_if_idle;
      typed_request->preempt_current = route.preempt_current;
      auto typed_response =
          std::make_shared<robot_interfaces_business::srv::SubmitScenarioWorkflow::Response>();
      SubmitScenarioWorkflow(typed_request, typed_response);
      FillSubmitOrderResponse(
          response, typed_response->success, typed_response->message, "",
          typed_response->mission_ids, typed_response->queue_size);
      return;
    }

    if (route.kind == robot_tasks::SubmitOrderKind::kVda5050) {
      auto typed_request =
          std::make_shared<robot_interfaces_business::srv::SubmitVda5050Order::Request>();
      typed_request->order_id = route.order_id;
      typed_request->serial_number = route.serial_number;
      typed_request->pickup_node_id = route.pickup_node_id;
      typed_request->dropoff_node_id = route.dropoff_node_id;
      typed_request->priority = route.priority;
      typed_request->start_if_idle = route.start_if_idle;
      typed_request->preempt_current = route.preempt_current;
      auto typed_response =
          std::make_shared<robot_interfaces_business::srv::SubmitVda5050Order::Response>();
      SubmitVda5050Order(typed_request, typed_response);
      FillSubmitOrderResponse(
          response, typed_response->success, typed_response->message,
          typed_response->mission_id, {}, typed_response->queue_size);
      return;
    }

    if (route.kind == robot_tasks::SubmitOrderKind::kPickingWave) {
      auto typed_request =
          std::make_shared<robot_interfaces_business::srv::SubmitPickingWave::Request>();
      typed_request->wave_id = route.wave_id;
      typed_request->sku_id = route.sku_id;
      typed_request->tote_id = route.tote_id;
      typed_request->pick_station = route.pick_station;
      typed_request->dropoff_station = route.dropoff_station;
      typed_request->quantity = route.quantity;
      typed_request->priority = route.priority;
      typed_request->start_if_idle = route.start_if_idle;
      auto typed_response =
          std::make_shared<robot_interfaces_business::srv::SubmitPickingWave::Response>();
      SubmitPickingWave(typed_request, typed_response);
      FillSubmitOrderResponse(
          response, typed_response->success, typed_response->message, "",
          typed_response->mission_ids, typed_response->queue_size);
      return;
    }

    if (route.kind == robot_tasks::SubmitOrderKind::kReplenishment) {
      auto typed_request =
          std::make_shared<robot_interfaces_business::srv::SubmitReplenishmentOrder::Request>();
      typed_request->order_id = route.order_id;
      typed_request->sku_id = route.sku_id;
      typed_request->tote_id = route.tote_id;
      typed_request->source_station = route.source_station;
      typed_request->line_station = route.line_station;
      typed_request->quantity = route.quantity;
      typed_request->priority = route.priority;
      typed_request->start_if_idle = route.start_if_idle;
      auto typed_response =
          std::make_shared<robot_interfaces_business::srv::SubmitReplenishmentOrder::Response>();
      SubmitReplenishmentOrder(typed_request, typed_response);
      FillSubmitOrderResponse(
          response, typed_response->success, typed_response->message, "",
          typed_response->mission_ids, typed_response->queue_size);
      return;
    }

    if (route.kind == robot_tasks::SubmitOrderKind::kMilkRun) {
      auto typed_request = std::make_shared<robot_interfaces_business::srv::SubmitMilkRun::Request>();
      typed_request->run_id = route.run_id;
      typed_request->tote_id = route.tote_id;
      typed_request->station_ids = route.station_ids;
      typed_request->priority = route.priority;
      typed_request->start_if_idle = route.start_if_idle;
      auto typed_response =
          std::make_shared<robot_interfaces_business::srv::SubmitMilkRun::Response>();
      SubmitMilkRun(typed_request, typed_response);
      FillSubmitOrderResponse(
          response, typed_response->success, typed_response->message, "",
          typed_response->mission_ids, typed_response->queue_size);
      return;
    }

    RejectSubmitOrder(response, "unhandled order_type route");
  }

  template <typename RequestT, typename ResponseT>
  void SubmitTransportOrder(
      const std::shared_ptr<RequestT>& request,
      const std::shared_ptr<ResponseT>& response) {
    auto profile = CreateTransportOrderProfile(*request, response);
    if (!profile.has_value()) {
      response->queue_size = static_cast<int>(mission_queue_.size());
      return;
    }
    const auto duplicate_gate = robot_tasks::PlanMissionSubmitGate(
        robot_tasks::MissionSubmitGateRequest{
            profile->mission_id, "transport order", "transport order",
            MissionIdActiveOrQueued(profile->mission_id)});
    if (!duplicate_gate.accepted) {
      response->success = false;
      response->mission_id = duplicate_gate.mission_id;
      response->message = duplicate_gate.message;
      response->queue_size = static_cast<int>(mission_queue_.size());
      return;
    }
    const auto preflight = ValidateMissionForQueue(*profile);
    const auto gate = robot_tasks::PlanMissionSubmitGate(
        robot_tasks::MissionSubmitGateRequest{
            profile->mission_id, "transport order", "transport order", false, preflight});
    if (!gate.accepted) {
      response->success = false;
      response->mission_id = gate.mission_id;
      response->message = gate.message;
      response->queue_size = static_cast<int>(mission_queue_.size());
      return;
    }

    const auto admission = AdmitQueuedMission(
        QueuedMission{*profile, "transport_order:" + profile->mission_id, request->priority,
                      next_queue_sequence_++},
        request->start_if_idle, false, "",
        "queued transport order " + profile->mission_id + " priority " +
            std::to_string(request->priority));
    response->success = admission.success;
    response->mission_id = profile->mission_id;
    response->queue_size = admission.queue_size;
    response->message = admission.message;
    ApplyMissionQueueAdmissionState(admission);
  }

  template <typename ResponseT>
  void ListStations(const std::shared_ptr<ResponseT>& response) {
    const auto catalog = LoadStations(response);
    if (!catalog.has_value()) {
      return;
    }
    const auto projection = robot_tasks::BuildStationListProjection(*catalog);
    response->success = true;
    response->message = projection.message;
    response->station_ids = projection.station_ids;
    response->frame_ids = projection.frame_ids;
    response->x = projection.x;
    response->y = projection.y;
    response->yaw = projection.yaw;
  }

  template <typename ResponseT>
  void ListStationRoutes(const std::shared_ptr<ResponseT>& response) {
    const auto catalog = robot_tasks::LoadStationCatalog(station_file_);
    if (!catalog.has_value()) {
      response->success = false;
      response->message = "failed to load station catalog: " + station_file_;
      return;
    }
    const auto projection = robot_tasks::BuildStationRouteListProjection(*catalog);
    response->success = true;
    response->message = projection.message;
    response->from_station = projection.from_station;
    response->to_station = projection.to_station;
    response->distance_m = projection.distance_m;
    response->bidirectional = projection.bidirectional;
    response->enabled = projection.enabled;
  }

  template <typename RequestT, typename ResponseT>
  void BlockStationRoute(
      const std::shared_ptr<RequestT>& request,
      const std::shared_ptr<ResponseT>& response) {
    const auto from = SanitizeId(request->from_station);
    const auto to = SanitizeId(request->to_station);
    const auto lock_id = RouteLockId(from, to);
    const auto catalog = robot_tasks::LoadStationCatalog(station_file_);
    if (!catalog.has_value()) {
      response->success = false;
      response->message = "failed to load station catalog: " + station_file_;
      response->resource_id = lock_id;
      return;
    }
    const auto reason = SanitizeId(request->reason.empty() ? "manual" : request->reason);
    const auto decision = robot_tasks::PlanTrafficBlock(
        robot_tasks::TrafficBlockRequest{
            from, to, reason,
            robot_tasks::FindStation(*catalog, from) != nullptr &&
                robot_tasks::FindStation(*catalog, to) != nullptr,
            robot_tasks::FindRoutePath(*catalog, from, to).has_value()},
        route_locks_);
    if (decision.mutate_state) {
      route_locks_[decision.resource_id] = decision.owner_id;
    }
    response->success = decision.success;
    response->message = decision.message;
    response->resource_id = decision.resource_id;
  }

  template <typename RequestT, typename ResponseT>
  void UnblockStationRoute(
      const std::shared_ptr<RequestT>& request,
      const std::shared_ptr<ResponseT>& response) {
    const auto from = SanitizeId(request->from_station);
    const auto to = SanitizeId(request->to_station);
    const auto decision = robot_tasks::PlanTrafficUnblock(from, to, route_locks_);
    if (decision.mutate_state) {
      route_locks_.erase(decision.resource_id);
    }
    response->success = decision.success;
    response->message = decision.message;
    response->resource_id = decision.resource_id;
  }

  template <typename ResponseT>
  void ListTrafficReservations(const std::shared_ptr<ResponseT>& response) const {
    response->success = true;
    response->message = "loaded " + std::to_string(route_locks_.size()) + " traffic reservation(s)";
    for (const auto& [resource_id, owner_id] : route_locks_) {
      response->resource_ids.push_back(resource_id);
      response->owner_ids.push_back(owner_id);
      response->reservation_types.push_back(
          robot_tasks::IsTrafficBlockOwner(owner_id) ? "blocked" : "mission");
    }
  }

  template <typename RequestT, typename ResponseT>
  void ClearTrafficReservation(
      const std::shared_ptr<RequestT>& request,
      const std::shared_ptr<ResponseT>& response) {
    const auto decision =
        robot_tasks::PlanTrafficReservationClear(request->resource_id, route_locks_);
    if (decision.erase_resource) {
      route_locks_.erase(decision.resource_id);
    }
    if (!decision.release_mission_id.empty()) {
      ReleaseRouteLocksForMission(decision.release_mission_id);
    }
    response->success = decision.success;
    response->message = decision.message;
    response->owner_id = decision.owner_id;
  }

  template <typename ResponseT>
  void DetectTrafficDeadlock(const std::shared_ptr<ResponseT>& response) const {
    response->success = true;
    const auto decision = robot_tasks::PlanTrafficDeadlockDetection(route_locks_);
    response->deadlocked = decision.deadlocked;
    response->mission_ids = decision.mission_ids;
    response->conflict_resource_ids = decision.conflict_resource_ids;
    response->conflict_station_ids = decision.conflict_station_ids;
    response->descriptions = decision.descriptions;
    response->message = decision.message;
  }

  std::optional<robot_tasks::FleetCatalog> LoadFleet(std::string* message) const {
    auto catalog = robot_tasks::LoadFleetCatalog(fleet_file_);
    if (!catalog.has_value()) {
      if (message != nullptr) {
        *message = "failed to load fleet catalog: " + fleet_file_;
      }
      return std::nullopt;
    }
    ApplyRuntimeFleetState(*catalog);
    return catalog;
  }

  void ApplyRuntimeFleetState(robot_tasks::FleetCatalog& catalog) const {
    robot_tasks::ApplyRuntimeFleetState(catalog, runtime_fleet_state_);
  }

  std::string JoinIds(const std::vector<std::string>& ids) const {
    std::string result;
    for (const auto& id : ids) {
      if (!result.empty()) {
        result += ",";
      }
      result += id;
    }
    return result;
  }

  template <typename RequestT, typename ResponseT>
  void ListFleetRobots(
      const std::shared_ptr<RequestT>& request,
      const std::shared_ptr<ResponseT>& response) {
    std::string message;
    const auto catalog = LoadFleet(&message);
    if (!catalog.has_value()) {
      response->success = false;
      response->message = message;
      return;
    }
    const auto projection = robot_tasks::BuildFleetRobotListProjection(
        *catalog,
        robot_tasks::FleetRobotListRequest{request->include_disabled, request->capability});
    response->success = projection.success;
    response->robot_ids = projection.robot_ids;
    response->enabled = projection.enabled;
    response->states = projection.states;
    response->battery_voltage = projection.battery_voltage;
    response->current_station_ids = projection.current_station_ids;
    response->capabilities = projection.capabilities;
    response->message = projection.message;
  }

  template <typename RequestT, typename ResponseT>
  void UpdateFleetRobotState(
      const std::shared_ptr<RequestT>& request,
      const std::shared_ptr<ResponseT>& response) {
    const auto robot_id = SanitizeId(request->robot_id);
    response->robot_id = robot_id;
    auto base_catalog = robot_tasks::LoadFleetCatalog(fleet_file_);
    if (!base_catalog.has_value()) {
      response->success = false;
      response->message = "failed to load fleet catalog: " + fleet_file_;
      return;
    }
    std::optional<robot_tasks::StationCatalog> stations;
    if (request->update_current_station_id) {
      stations = robot_tasks::LoadStationCatalog(station_file_);
      if (!stations.has_value()) {
        response->success = false;
        response->message = "failed to load station catalog: " + station_file_;
        return;
      }
    }

    const auto decision = robot_tasks::PlanFleetRuntimeStateUpdate(
        robot_tasks::FleetRuntimeUpdateRequest{
            robot_id, request->enabled, SanitizeId(request->state),
            request->battery_voltage, SanitizeId(request->current_station_id),
            request->update_enabled, request->update_state,
            request->update_battery_voltage, request->update_current_station_id},
        *base_catalog, stations.has_value() ? &*stations : nullptr, runtime_fleet_state_);
    response->success = decision.success;
    response->message = decision.message;
    if (!decision.success) {
      return;
    }
    runtime_fleet_state_[robot_id] = decision.runtime_state;
    response->enabled = decision.effective_robot.enabled;
    response->state = decision.effective_robot.state;
    response->battery_voltage = decision.effective_robot.battery_voltage;
    response->current_station_id = decision.effective_robot.current_station_id;
  }

  template <typename RequestT, typename ResponseT>
  void UpdateRemoteTaskState(
      const std::shared_ptr<RequestT>& request,
      const std::shared_ptr<ResponseT>& response) {
    const auto task_id = SanitizeId(request->task_id);
    const auto state = SanitizeId(request->state);
    const auto decision = robot_tasks::PlanRemoteFleetTaskStateUpdate(
        robot_tasks::RemoteFleetTaskUpdateRequest{task_id, state, request->message},
        remote_tasks_);
    if (decision.mutate_state) {
      remote_tasks_[task_id] = decision.task;
      RecordMissionEventFor(
          decision.event_mission_id, decision.event_state, decision.event_previous_state,
          decision.event_message, decision.event_recoverable);
    }
    response->success = decision.success;
    response->task_id = task_id;
    response->assigned_robot_id = decision.task.assigned_robot_id;
    response->state = decision.task.state;
    response->message = decision.response_message;
  }

  std::optional<robot_tasks::FleetRobotTaskScore> SelectFleetStationTaskCandidate(
      const std::string& required_capability, const std::string& pickup_station,
      const std::string& dropoff_station, std::string* selected_robot_id,
      std::string* message) const {
    const auto catalog = LoadFleet(message);
    if (!catalog.has_value()) {
      return std::nullopt;
    }
    const auto stations = robot_tasks::LoadStationCatalog(station_file_);
    if (!stations.has_value()) {
      if (message != nullptr) {
        *message = "failed to load station catalog: " + station_file_;
      }
      return std::nullopt;
    }
    robot_tasks::FleetRobotTaskScore selected_score;
    const auto* selected = robot_tasks::SelectFleetRobotForStationTask(
        *catalog, *stations, required_capability, pickup_station, dropoff_station, {},
        &selected_score);
    if (selected == nullptr) {
      if (message != nullptr) {
        *message = "no available fleet robot with capability: " + required_capability;
      }
      return std::nullopt;
    }
    if (selected_robot_id != nullptr) {
      *selected_robot_id = selected->id;
    }
    if (selected_score.robot_id.empty()) {
      selected_score.robot_id = selected->id;
      selected_score.battery_voltage = selected->battery_voltage;
    }
    return selected_score;
  }

  template <typename RequestT, typename ResponseT>
  void PreviewFleetStationTask(
      const std::shared_ptr<RequestT>& request,
      const std::shared_ptr<ResponseT>& response) {
    const auto required_capability =
        request->required_capability.empty() ? "station_transport" : request->required_capability;
    std::string message;
    std::string selected_robot_id;
    const auto score = SelectFleetStationTaskCandidate(
        required_capability, request->pickup_station, request->dropoff_station,
        &selected_robot_id, &message);
    if (!score.has_value()) {
      response->success = false;
      response->message = message;
      return;
    }
    response->success = true;
    response->assigned_robot_id = selected_robot_id;
    response->local_robot = selected_robot_id == robot_id_;
    response->approach_distance_m = score->approach_distance_m;
    response->task_distance_m = score->task_distance_m;
    response->estimated_total_distance_m = score->total_distance_m;
    response->battery_voltage = score->battery_voltage;
    response->message = "preview selected " + selected_robot_id +
                        " estimated_total_distance_m=" +
                        std::to_string(score->total_distance_m);
  }

  template <typename RequestT, typename ResponseT>
  void SubmitFleetStationTask(
      const std::shared_ptr<RequestT>& request,
      const std::shared_ptr<ResponseT>& response) {
    const auto required_capability =
        request->required_capability.empty() ? "station_transport" : request->required_capability;
    std::string message;
    std::string selected_robot_id;
    const auto selected_score = SelectFleetStationTaskCandidate(
        required_capability, request->pickup_station, request->dropoff_station,
        &selected_robot_id, &message);
    if (!selected_score.has_value()) {
      response->success = false;
      response->message = message;
      response->queue_size = static_cast<int>(mission_queue_.size());
      return;
    }
    response->assigned_robot_id = selected_robot_id;
    const auto sanitized_task_id = SanitizeId(request->task_id);
    const auto task_id = sanitized_task_id.empty()
                             ? "fleet_task_" + std::to_string(next_fleet_task_sequence_++)
                             : sanitized_task_id;
    const auto dispatch_decision = robot_tasks::PlanFleetStationTaskDispatch(
        robot_tasks::FleetStationTaskDispatchRequest{
            task_id, selected_robot_id, robot_id_, request->pickup_station,
            request->dropoff_station, current_state_, selected_score->total_distance_m},
        remote_tasks_);
    if (!dispatch_decision.success) {
      response->success = false;
      response->task_id = task_id;
      response->queue_size = static_cast<int>(mission_queue_.size());
      response->message = dispatch_decision.response_message;
      return;
    }
    if (dispatch_decision.dispatch_remote) {
      remote_tasks_[task_id] = dispatch_decision.remote_task;
      RecordMissionEventFor(
          dispatch_decision.event_mission_id, dispatch_decision.event_state,
          dispatch_decision.event_previous_state, dispatch_decision.event_message,
          dispatch_decision.event_recoverable);
      response->success = true;
      response->task_id = task_id;
      response->mission_id = dispatch_decision.mission_id;
      response->queue_size = static_cast<int>(mission_queue_.size());
      response->estimated_total_distance_m = selected_score->total_distance_m;
      response->message = dispatch_decision.response_message;
      return;
    }

    auto station_request =
        std::make_shared<robot_interfaces_mission::srv::SubmitStationTransportOrder::Request>();
    station_request->order_id = dispatch_decision.station_order_id;
    station_request->pickup_station = request->pickup_station;
    station_request->dropoff_station = request->dropoff_station;
    station_request->priority = request->priority;
    station_request->start_if_idle = request->start_if_idle;
    station_request->preempt_current = request->preempt_current;
    auto station_response =
        std::make_shared<robot_interfaces_mission::srv::SubmitStationTransportOrder::Response>();
    SubmitStationTransportOrder(station_request, station_response);

    response->success = station_response->success;
    response->task_id = task_id;
    response->mission_id = station_response->mission_id;
    response->queue_size = station_response->queue_size;
    response->estimated_total_distance_m = selected_score->total_distance_m;
    response->message = station_response->success
                            ? "assigned fleet task " + task_id + " to " + selected_robot_id +
                                  " estimated_total_distance_m=" +
                                  std::to_string(selected_score->total_distance_m) +
                                  ": " + station_response->message
                            : "fleet task rejected by station order chain: " +
                                  station_response->message;
  }

  template <typename RequestT, typename ResponseT>
  void SubmitVda5050Order(
      const std::shared_ptr<RequestT>& request,
      const std::shared_ptr<ResponseT>& response) {
    const auto decision = robot_tasks::PlanVda5050StationOrder(
        robot_tasks::Vda5050StationOrderRequest{
            SanitizeId(request->order_id), SanitizeId(request->serial_number),
            SanitizeId(request->pickup_node_id), SanitizeId(request->dropoff_node_id),
            request->priority, request->start_if_idle, request->preempt_current});
    response->order_id = decision.order_id;
    response->serial_number = decision.serial_number;
    response->queue_size = static_cast<int>(mission_queue_.size());
    if (!decision.success) {
      response->success = false;
      response->message = decision.message;
      return;
    }

    auto station_request =
        std::make_shared<robot_interfaces_mission::srv::SubmitStationTransportOrder::Request>();
    station_request->order_id = decision.station_order_id;
    station_request->pickup_station = decision.pickup_station;
    station_request->dropoff_station = decision.dropoff_station;
    station_request->priority = decision.priority;
    station_request->start_if_idle = decision.start_if_idle;
    station_request->preempt_current = decision.preempt_current;
    auto station_response =
        std::make_shared<robot_interfaces_mission::srv::SubmitStationTransportOrder::Response>();
    SubmitStationTransportOrder(station_request, station_response);

    response->success = station_response->success;
    response->mission_id = station_response->mission_id;
    response->queue_size = station_response->queue_size;
    response->message = station_response->success
                            ? decision.message + ": " +
                                  station_response->message
                            : "vda5050 order rejected by station order chain: " +
                                  station_response->message;
  }

  template <typename RequestT, typename ResponseT>
  void SubmitBusinessOrder(
      const std::shared_ptr<RequestT>& request,
      const std::shared_ptr<ResponseT>& response) {
    const auto business_order_id = SanitizeId(request->business_order_id);
    const auto business_type = SanitizeId(request->business_type);
    response->queue_size = static_cast<int>(mission_queue_.size());

    std::string message;
    const auto business_catalog = LoadBusinessOrders(&message);
    const auto decision = robot_tasks::PlanBusinessOrderSubmit(
        robot_tasks::BusinessOrderSubmitRequest{
            business_order_id, business_type, request->priority, request->start_if_idle,
            request->preempt_current},
        business_catalog.has_value() ? &*business_catalog : nullptr,
        message);
    response->business_order_id = decision.business_order_id;
    response->business_type = decision.business_type;
    response->routed_service = decision.routed_service;
    response->routed_scenario_id = decision.routed_scenario_id;
    response->routed_workflow_id = decision.routed_workflow_id;
    if (!decision.success) {
      response->success = false;
      response->message = decision.message;
      return;
    }

    auto workflow_request =
        std::make_shared<robot_interfaces_business::srv::SubmitScenarioWorkflow::Request>();
    workflow_request->scenario_id = decision.routed_scenario_id;
    workflow_request->workflow_id = decision.routed_workflow_id;
    workflow_request->order_id = decision.workflow_order_id;
    workflow_request->priority = decision.priority;
    workflow_request->start_if_idle = decision.start_if_idle;
    workflow_request->preempt_current = decision.preempt_current;
    auto workflow_response =
        std::make_shared<robot_interfaces_business::srv::SubmitScenarioWorkflow::Response>();
    SubmitScenarioWorkflow(workflow_request, workflow_response);

    response->success = workflow_response->success;
    response->mission_ids = workflow_response->mission_ids;
    response->queued_count = workflow_response->queued_count;
    response->queue_size = workflow_response->queue_size;
    response->message = workflow_response->success
                            ? decision.message + ": " + workflow_response->message
                            : "business order rejected by scenario workflow chain: " +
                                  workflow_response->message;
  }

  std::optional<robot_tasks::BusinessOrderCatalog> LoadBusinessOrders(
      std::string* message) const {
    const auto catalog = robot_tasks::LoadBusinessOrderCatalog(business_order_file_);
    if (!catalog.has_value()) {
      if (message != nullptr) {
        *message = "failed to load business order catalog: " + business_order_file_;
      }
      return std::nullopt;
    }
    return catalog;
  }

  template <typename ResponseT>
  void ListBusinessOrderTypes(const std::shared_ptr<ResponseT>& response) const {
    std::string message;
    const auto catalog = LoadBusinessOrders(&message);
    if (!catalog.has_value()) {
      response->success = false;
      response->message = message;
      return;
    }
    for (const auto& order_type : catalog->order_types) {
      response->business_types.push_back(order_type.business_type);
      response->scenario_ids.push_back(order_type.scenario_id);
      response->workflow_ids.push_back(order_type.workflow_id);
      response->labels.push_back(order_type.label);
    }
    response->success = true;
    response->message =
        "loaded " + std::to_string(response->business_types.size()) +
        " business order type(s)";
  }

  template <typename RequestT, typename ResponseT>
  void GetOperatorSnapshot(
      const std::shared_ptr<RequestT>& request,
      const std::shared_ptr<ResponseT>& response) const {
    std::string message;
    const auto fleet = LoadFleet(&message);
    const auto facilities = LoadFacilities(&message);
    const auto business_orders = LoadBusinessOrders(&message);
    const auto projection = robot_tasks::BuildOperatorSnapshotProjection(
        robot_tasks::OperatorSnapshotRequest{
            current_state_,
            previous_state_,
            ActiveMissionId(),
            last_message_,
            recoverable_,
            RunnerBusy(),
            paused_,
            mission_queue_,
            paused_workflow_orders_,
            fleet.has_value() ? &*fleet : nullptr,
            &remote_tasks_,
            facilities.has_value() ? &*facilities : nullptr,
            &facility_reservations_,
            business_orders.has_value() ? &*business_orders : nullptr,
            mission_events_,
            request->event_limit});

    response->success = projection.success;
    response->mission_state = projection.mission_state;
    response->previous_mission_state = projection.previous_mission_state;
    response->active_mission_id = projection.active_mission_id;
    response->mission_message = projection.mission_message;
    response->recoverable = projection.recoverable;
    response->mission_active = projection.mission_active;
    response->paused = projection.paused;
    response->queue_size = projection.queue_size;
    response->queued_mission_ids = projection.queued_mission_ids;
    response->queued_priorities = projection.queued_priorities;
    response->paused_workflow_order_ids = projection.paused_workflow_order_ids;
    response->fleet_robot_ids = projection.fleet_robot_ids;
    response->fleet_enabled = projection.fleet_enabled;
    response->fleet_states = projection.fleet_states;
    response->fleet_battery_voltage = projection.fleet_battery_voltage;
    response->fleet_current_station_ids = projection.fleet_current_station_ids;
    response->fleet_capabilities = projection.fleet_capabilities;
    response->remote_task_ids = projection.remote_task_ids;
    response->remote_task_robot_ids = projection.remote_task_robot_ids;
    response->remote_task_states = projection.remote_task_states;
    response->facility_resource_ids = projection.facility_resource_ids;
    response->facility_resource_types = projection.facility_resource_types;
    response->facility_station_ids = projection.facility_station_ids;
    response->facility_available = projection.facility_available;
    response->facility_status = projection.facility_status;
    response->business_types = projection.business_types;
    response->business_scenario_ids = projection.business_scenario_ids;
    response->business_workflow_ids = projection.business_workflow_ids;
    response->recent_event_stamp = projection.recent_event_stamp;
    response->recent_event_mission_ids = projection.recent_event_mission_ids;
    response->recent_event_states = projection.recent_event_states;
    response->recent_event_messages = projection.recent_event_messages;
    response->message = projection.message;
  }

  std::optional<robot_tasks::ScenarioCatalog> LoadScenarios(std::string* message) const {
    const auto catalog = robot_tasks::LoadScenarioCatalog(scenario_file_);
    if (!catalog.has_value()) {
      if (message != nullptr) {
        *message = "failed to load scenario catalog: " + scenario_file_;
      }
      return std::nullopt;
    }
    return catalog;
  }

  template <typename RequestT, typename ResponseT>
  void ListScenarioTasks(
      const std::shared_ptr<RequestT>& request,
      const std::shared_ptr<ResponseT>& response) {
    std::string message;
    const auto catalog = LoadScenarios(&message);
    if (!catalog.has_value()) {
      response->success = false;
      response->message = message;
      return;
    }
    const auto scenario_filter = SanitizeId(request->scenario_id);
    for (const auto& task : catalog->tasks) {
      if (!scenario_filter.empty() && task.scenario_id != scenario_filter) {
        continue;
      }
      response->scenario_ids.push_back(task.scenario_id);
      response->task_ids.push_back(task.task_id);
      response->labels.push_back(task.label);
      response->task_types.push_back(task.task_type.empty() ? "station_transport" : task.task_type);
      response->pickup_station_ids.push_back(task.pickup_station_id);
      response->dropoff_station_ids.push_back(task.dropoff_station_id);
      response->station_sequences_csv.push_back(JoinIds(task.station_sequence_ids));
      response->required_capabilities.push_back(task.required_capability);
      response->resource_ids.push_back(task.resource_id);
      response->resource_types.push_back(task.resource_type);
      response->actions.push_back(task.action);
      response->hold_after_actions.push_back(task.hold_after_action);
      response->default_priorities.push_back(task.default_priority);
    }
    response->success = true;
    response->message =
        "loaded " + std::to_string(response->task_ids.size()) + " scenario task(s)";
  }

  template <typename RequestT, typename ResponseT>
  void SubmitScenarioTask(
      const std::shared_ptr<RequestT>& request,
      const std::shared_ptr<ResponseT>& response) {
    std::string message;
    const auto catalog = LoadScenarios(&message);
    const auto scenario_id = SanitizeId(request->scenario_id);
    const auto task_id = SanitizeId(request->task_id);
    const auto decision = robot_tasks::PlanScenarioTaskSubmit(
        robot_tasks::ScenarioTaskSubmitRequest{
            scenario_id, task_id, SanitizeId(request->order_id), request->priority,
            request->start_if_idle, request->preempt_current},
        catalog.has_value() ? &*catalog : nullptr,
        message);
    response->scenario_id = decision.scenario_id;
    response->task_id = decision.task_id;
    response->queue_size = static_cast<int>(mission_queue_.size());
    if (!decision.success) {
      response->success = false;
      response->message = decision.message;
      return;
    }

    if (decision.dispatch_kind == robot_tasks::ScenarioTaskDispatchKind::kFacilityAction) {
      auto facility_request =
          std::make_shared<robot_interfaces_facility::srv::ExecuteFacilityAction::Request>();
      facility_request->request_id = decision.order_id;
      facility_request->resource_id = decision.resource_id;
      facility_request->resource_type = decision.resource_type;
      facility_request->action = decision.action;
      facility_request->priority = decision.priority;
      facility_request->start_if_idle = decision.start_if_idle;
      facility_request->preempt_current = decision.preempt_current;
      facility_request->hold_after_action = decision.hold_after_action;
      auto facility_response =
          std::make_shared<robot_interfaces_facility::srv::ExecuteFacilityAction::Response>();
      ExecuteFacilityAction(facility_request, facility_response);

      response->success = facility_response->success;
      response->mission_id = facility_response->mission_id;
      response->queue_size = facility_response->queue_size;
      response->message = facility_response->success
                              ? "queued scenario facility task " + decision.scenario_id + "/" +
                                    decision.task_id + ": " + facility_response->message
                              : "scenario facility task rejected by facility action chain: " +
                                    facility_response->message;
      return;
    }

    if (decision.dispatch_kind == robot_tasks::ScenarioTaskDispatchKind::kStationSequence) {
      auto sequence_request =
          std::make_shared<robot_interfaces_mission::srv::SubmitStationSequenceTask::Request>();
      sequence_request->order_id = decision.order_id;
      sequence_request->station_ids = decision.station_ids;
      sequence_request->priority = decision.priority;
      sequence_request->start_if_idle = decision.start_if_idle;
      sequence_request->preempt_current = decision.preempt_current;
      auto sequence_response =
          std::make_shared<robot_interfaces_mission::srv::SubmitStationSequenceTask::Response>();
      SubmitStationSequenceTask(sequence_request, sequence_response);

      response->success = sequence_response->success;
      response->mission_id =
          sequence_response->mission_ids.empty() ? "" : sequence_response->mission_ids.front();
      response->queue_size = sequence_response->queue_size;
      response->message = sequence_response->success
                              ? "queued scenario station sequence task " + decision.scenario_id +
                                    "/" + decision.task_id + ": " + sequence_response->message
                              : "scenario station sequence task rejected by sequence chain: " +
                                    sequence_response->message;
      return;
    }

    auto fleet_request =
        std::make_shared<robot_interfaces_fleet::srv::SubmitFleetStationTask::Request>();
    fleet_request->task_id = decision.order_id;
    fleet_request->required_capability = decision.required_capability;
    fleet_request->pickup_station = decision.pickup_station;
    fleet_request->dropoff_station = decision.dropoff_station;
    fleet_request->priority = decision.priority;
    fleet_request->start_if_idle = decision.start_if_idle;
    fleet_request->preempt_current = decision.preempt_current;
    auto fleet_response =
        std::make_shared<robot_interfaces_fleet::srv::SubmitFleetStationTask::Response>();
    SubmitFleetStationTask(fleet_request, fleet_response);

    response->success = fleet_response->success;
    response->mission_id = fleet_response->mission_id;
    response->assigned_robot_id = fleet_response->assigned_robot_id;
    response->queue_size = fleet_response->queue_size;
    response->message = fleet_response->success
                            ? "queued scenario task " + decision.scenario_id + "/" +
                                  decision.task_id + ": " + fleet_response->message
                            : "scenario task rejected by fleet task chain: " +
                                  fleet_response->message;
  }

  template <typename RequestT, typename ResponseT>
  void ListScenarioWorkflows(
      const std::shared_ptr<RequestT>& request,
      const std::shared_ptr<ResponseT>& response) {
    std::string message;
    const auto catalog = LoadScenarios(&message);
    if (!catalog.has_value()) {
      response->success = false;
      response->message = message;
      return;
    }
    const auto scenario_filter = SanitizeId(request->scenario_id);
    for (const auto& workflow : catalog->workflows) {
      if (!scenario_filter.empty() && workflow.scenario_id != scenario_filter) {
        continue;
      }
      response->scenario_ids.push_back(workflow.scenario_id);
      response->workflow_ids.push_back(workflow.workflow_id);
      response->labels.push_back(workflow.label);
      response->task_ids_csv.push_back(JoinIds(workflow.task_ids));
      response->default_priorities.push_back(workflow.default_priority);
    }
    response->success = true;
    response->message =
        "loaded " + std::to_string(response->workflow_ids.size()) + " scenario workflow(s)";
  }

  template <typename RequestT, typename ResponseT>
  void SubmitScenarioWorkflow(
      const std::shared_ptr<RequestT>& request,
      const std::shared_ptr<ResponseT>& response) {
    std::string message;
    const auto catalog = LoadScenarios(&message);
    const auto scenario_id = SanitizeId(request->scenario_id);
    const auto workflow_id = SanitizeId(request->workflow_id);
    const auto decision = robot_tasks::PlanScenarioWorkflowSubmit(
        robot_tasks::ScenarioWorkflowSubmitRequest{
            scenario_id, workflow_id, SanitizeId(request->order_id), request->priority,
            request->start_if_idle, request->preempt_current},
        catalog.has_value() ? &*catalog : nullptr,
        message);
    response->scenario_id = decision.scenario_id;
    response->workflow_id = decision.workflow_id;
    if (!decision.success) {
      response->success = false;
      response->message = decision.message;
      response->queue_size = static_cast<int>(mission_queue_.size());
      return;
    }

    for (std::size_t index = 0; index < decision.steps.size(); ++index) {
      const auto& step = decision.steps[index];
      auto task_request = std::make_shared<robot_interfaces_business::srv::SubmitScenarioTask::Request>();
      task_request->scenario_id = step.scenario_id;
      task_request->task_id = step.task_id;
      task_request->order_id = step.order_id;
      task_request->priority = step.priority;
      task_request->start_if_idle = step.start_if_idle;
      task_request->preempt_current = step.preempt_current;
      auto task_response =
          std::make_shared<robot_interfaces_business::srv::SubmitScenarioTask::Response>();
      SubmitScenarioTask(task_request, task_response);
      if (!task_response->success) {
        response->success = false;
        response->queued_count = static_cast<int>(response->mission_ids.size());
        response->queue_size = task_response->queue_size;
        response->message = "scenario workflow rejected at step " +
                            std::to_string(static_cast<int>(index) + 1) + ": " +
                            task_response->message;
        return;
      }
      response->mission_ids.push_back(task_response->mission_id);
      response->queue_size = task_response->queue_size;
    }
    response->success = true;
    response->queued_count = static_cast<int>(response->mission_ids.size());
    response->queue_size = static_cast<int>(mission_queue_.size());
    response->message = "queued scenario workflow " + decision.scenario_id + "/" +
                        decision.workflow_id +
                        " with " + std::to_string(response->queued_count) + " task(s)";
  }

  std::optional<robot_tasks::FacilityCatalog> LoadFacilities(std::string* message) const {
    const auto catalog = robot_tasks::LoadFacilityCatalog(facility_file_);
    if (!catalog.has_value()) {
      if (message != nullptr) {
        *message = "failed to load facility catalog: " + facility_file_;
      }
      return std::nullopt;
    }
    return catalog;
  }

  bool ResourceAvailable(const robot_tasks::FacilityResource& resource) const {
    return robot_tasks::FacilityResourceAvailable(resource, facility_reservations_);
  }

  std::string ResourceStatus(const robot_tasks::FacilityResource& resource) const {
    return robot_tasks::FacilityResourceStatus(resource, facility_reservations_);
  }

  template <typename RequestT, typename ResponseT>
  void ListFacilityResources(
      const std::shared_ptr<RequestT>& request,
      const std::shared_ptr<ResponseT>& response) {
    std::string message;
    const auto catalog = LoadFacilities(&message);
    if (!catalog.has_value()) {
      response->success = false;
      response->message = message;
      return;
    }
    const auto projection = robot_tasks::BuildFacilityResourceListProjection(
        *catalog, facility_reservations_,
        robot_tasks::FacilityResourceListRequest{
            request->include_disabled, request->resource_type});
    response->success = projection.success;
    response->resource_ids = projection.resource_ids;
    response->resource_types = projection.resource_types;
    response->station_ids = projection.station_ids;
    response->enabled = projection.enabled;
    response->available = projection.available;
    response->holder_ids = projection.holder_ids;
    response->status = projection.status;
    response->message = projection.message;
  }

  void ReleaseResourcesForHolder(const std::string& holder_id) {
    robot_tasks::ReleaseResourcesForHolder(facility_reservations_, holder_id);
  }

  template <typename RequestT, typename ResponseT>
  void ReserveFacilityResource(
      const std::shared_ptr<RequestT>& request,
      const std::shared_ptr<ResponseT>& response) {
    std::string message;
    const auto catalog = LoadFacilities(&message);
    if (!catalog.has_value()) {
      response->success = false;
      response->message = message;
      return;
    }
    const auto* resource = robot_tasks::FindFacilityResource(*catalog, request->resource_id);
    if (resource == nullptr) {
      response->success = false;
      response->message = "unknown facility resource: " + request->resource_id;
      return;
    }
    const auto holder_id =
        SanitizeId(request->holder_id.empty() ? request->resource_id : request->holder_id);
    const auto result = robot_tasks::ReserveFacilityResource(
        facility_reservations_, *resource, holder_id, SanitizeId(request->mission_id),
        request->release_existing_for_holder);
    response->success = result.success;
    response->resource_id = result.resource_id;
    response->holder_id = result.holder_id;
    response->status = result.status;
    response->message = result.message;
  }

  template <typename RequestT, typename ResponseT>
  void ReleaseFacilityResource(
      const std::shared_ptr<RequestT>& request,
      const std::shared_ptr<ResponseT>& response) {
    const auto result = robot_tasks::ReleaseFacilityResource(
        facility_reservations_, request->resource_id, SanitizeId(request->holder_id));
    response->success = result.success;
    response->resource_id = result.resource_id;
    response->status = result.status;
    response->message = result.message;
  }

  const robot_tasks::FacilityResource* FindFacilityByType(
      const robot_tasks::FacilityCatalog& catalog, const std::string& resource_id,
      const std::string& resource_type, std::string* message) const {
    const auto* resource = robot_tasks::FindFacilityResource(catalog, resource_id);
    if (resource == nullptr) {
      if (message != nullptr) {
        *message = "unknown " + resource_type + " resource: " + resource_id;
      }
      return nullptr;
    }
    if (resource->type != resource_type) {
      if (message != nullptr) {
        *message = resource_id + " is not a " + resource_type;
      }
      return nullptr;
    }
    return resource;
  }

  template <typename ResponseT>
  void FillLiftStateResponse(
      const robot_tasks::FacilityResource& resource,
      const std::shared_ptr<ResponseT>& response) const {
    const auto projection = robot_tasks::BuildFacilityResourceStateProjection(
        resource, facility_reservations_, "lift");
    response->success = true;
    response->lift_id = projection.resource_id;
    response->station_id = projection.station_id;
    response->available = projection.available;
    response->holder_id = projection.holder_id;
    response->status = projection.status;
    response->message = projection.message;
  }

  template <typename ResponseT>
  void FillDoorStateResponse(
      const robot_tasks::FacilityResource& resource,
      const std::shared_ptr<ResponseT>& response) const {
    const auto projection = robot_tasks::BuildFacilityResourceStateProjection(
        resource, facility_reservations_, "door");
    response->success = true;
    response->door_id = projection.resource_id;
    response->station_id = projection.station_id;
    response->available = projection.available;
    response->holder_id = projection.holder_id;
    response->status = projection.status;
    response->message = projection.message;
  }

  template <typename RequestT, typename ResponseT>
  void RequestLiftSession(
      const std::shared_ptr<RequestT>& request,
      const std::shared_ptr<ResponseT>& response) {
    std::string message;
    const auto catalog = LoadFacilities(&message);
    const auto lift_id = SanitizeId(request->lift_id);
    if (!catalog.has_value()) {
      response->success = false;
      response->message = message;
      response->lift_id = lift_id;
      return;
    }
    const auto* lift = FindFacilityByType(*catalog, lift_id, "elevator", &message);
    if (lift == nullptr) {
      response->success = false;
      response->message = message;
      response->lift_id = lift_id;
      return;
    }
    const auto session_id =
        robot_tasks::BuildLiftSessionId(SanitizeId(request->session_id), lift->id);
    const auto holder_id =
        robot_tasks::BuildLiftSessionHolderId(SanitizeId(request->requester_id), session_id);
    const auto result = robot_tasks::ReserveLiftSessionResource(
        facility_reservations_, *lift, session_id, holder_id,
        request->release_existing_for_requester);
    response->success = result.success;
    response->session_id = result.session_id;
    response->lift_id = result.lift_id;
    response->status = result.status;
    response->message = result.message;
  }

  template <typename RequestT, typename ResponseT>
  void ReleaseLiftSession(
      const std::shared_ptr<RequestT>& request,
      const std::shared_ptr<ResponseT>& response) {
    const auto lift_id = SanitizeId(request->lift_id);
    const auto session_id = SanitizeId(request->session_id);
    response->lift_id = lift_id;
    response->session_id = session_id;
    const auto result =
        robot_tasks::ReleaseLiftSessionResource(facility_reservations_, lift_id, session_id);
    response->success = result.success;
    response->status = result.status;
    response->message = result.message;
  }

  template <typename RequestT, typename ResponseT>
  void GetLiftState(
      const std::shared_ptr<RequestT>& request,
      const std::shared_ptr<ResponseT>& response) {
    std::string message;
    const auto catalog = LoadFacilities(&message);
    const auto lift_id = SanitizeId(request->lift_id);
    if (!catalog.has_value()) {
      response->success = false;
      response->message = message;
      response->lift_id = lift_id;
      return;
    }
    const auto* lift = FindFacilityByType(*catalog, lift_id, "elevator", &message);
    if (lift == nullptr) {
      response->success = false;
      response->message = message;
      response->lift_id = lift_id;
      return;
    }
    FillLiftStateResponse(*lift, response);
  }

  template <typename RequestT, typename ResponseT>
  void GetDoorState(
      const std::shared_ptr<RequestT>& request,
      const std::shared_ptr<ResponseT>& response) {
    std::string message;
    const auto catalog = LoadFacilities(&message);
    const auto door_id = SanitizeId(request->door_id);
    if (!catalog.has_value()) {
      response->success = false;
      response->message = message;
      response->door_id = door_id;
      return;
    }
    const auto* door = FindFacilityByType(*catalog, door_id, "door", &message);
    if (door == nullptr) {
      response->success = false;
      response->message = message;
      response->door_id = door_id;
      return;
    }
    FillDoorStateResponse(*door, response);
  }

  std::optional<robot_tasks::DockCatalog> LoadDocks(std::string* message) const {
    const auto catalog = robot_tasks::LoadDockCatalog(dock_file_);
    if (!catalog.has_value()) {
      if (message != nullptr) {
        *message = "failed to load dock catalog: " + dock_file_;
      }
      return std::nullopt;
    }
    return catalog;
  }

  const robot_tasks::Dock* ResolveDock(
      const robot_tasks::DockCatalog& catalog, const std::string& requested_dock_id,
      std::string* message) const {
    if (!requested_dock_id.empty()) {
      const auto* dock = robot_tasks::FindDock(catalog, requested_dock_id);
      if (dock == nullptr && message != nullptr) {
        *message = "unknown dock: " + requested_dock_id;
      }
      return dock;
    }
    const auto* dock = robot_tasks::FindFirstEnabledDock(catalog);
    if (dock == nullptr && message != nullptr) {
      *message = "no enabled dock";
    }
    return dock;
  }

  template <typename RequestT, typename ResponseT>
  void ListDocks(
      const std::shared_ptr<RequestT>& request,
      const std::shared_ptr<ResponseT>& response) {
    std::string message;
    const auto catalog = LoadDocks(&message);
    if (!catalog.has_value()) {
      response->success = false;
      response->message = message;
      return;
    }
    const auto projection = robot_tasks::BuildDockListProjection(
        *catalog, dock_states_, facility_reservations_, request->include_disabled);
    response->dock_ids = projection.dock_ids;
    response->station_ids = projection.station_ids;
    response->approach_station_ids = projection.approach_station_ids;
    response->charger_resource_ids = projection.charger_resource_ids;
    response->enabled = projection.enabled;
    response->available = projection.available;
    response->states = projection.states;
    response->mission_ids = projection.mission_ids;
    response->success = true;
    response->message = projection.message;
  }

  template <typename RequestT, typename ResponseT>
  void GetDockState(
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
    const auto projection = robot_tasks::BuildDockStateProjection(
        *dock, dock_states_, facility_reservations_);
    response->dock_id = projection.dock_id;
    response->station_id = projection.station_id;
    response->approach_station_id = projection.approach_station_id;
    response->charger_resource_id = projection.charger_resource_id;
    response->enabled = projection.enabled;
    response->available = projection.available;
    response->state = projection.state;
    response->mission_id = projection.mission_id;
    response->success = true;
    response->message = "dock " + dock->id + " state=" + response->state;
  }
