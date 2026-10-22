  MissionRunnerNode() : Node("mission_runner_node") {
    const auto default_mission_file =
        std::filesystem::path(ament_index_cpp::get_package_share_directory("robot_tasks")) /
        "config" / "missions" / "demo_patrol.yaml";
    const auto default_dock_mission_file =
        std::filesystem::path(ament_index_cpp::get_package_share_directory("robot_tasks")) /
        "config" / "missions" / "dock_return.yaml";
    const auto default_station_file =
        std::filesystem::path(ament_index_cpp::get_package_share_directory("robot_tasks")) /
        "config" / "stations" / "warehouse_stations.yaml";
    const auto default_facility_file =
        std::filesystem::path(ament_index_cpp::get_package_share_directory("robot_tasks")) /
        "config" / "facilities" / "warehouse_facilities.yaml";
    const auto default_dock_file =
        std::filesystem::path(ament_index_cpp::get_package_share_directory("robot_tasks")) /
        "config" / "docks" / "docks.yaml";
    const auto default_fleet_file =
        std::filesystem::path(ament_index_cpp::get_package_share_directory("robot_tasks")) /
        "config" / "fleet" / "warehouse_fleet.yaml";
    const auto default_scenario_file =
        std::filesystem::path(ament_index_cpp::get_package_share_directory("robot_tasks")) /
        "config" / "scenarios" / "industry_scenarios.yaml";
    const auto default_business_order_file =
        std::filesystem::path(ament_index_cpp::get_package_share_directory("robot_tasks")) /
        "config" / "business_orders" / "business_orders.yaml";
    const auto default_zones_file =
        std::filesystem::path(ament_index_cpp::get_package_share_directory("robot_navigation")) /
        "config" / "map_zones.yaml";
    mission_file_ = declare_parameter<std::string>("mission_file", default_mission_file.string());
    dock_mission_file_ =
        declare_parameter<std::string>("dock_mission_file", default_dock_mission_file.string());
    station_file_ = declare_parameter<std::string>("station_file", default_station_file.string());
    facility_file_ =
        declare_parameter<std::string>("facility_file", default_facility_file.string());
    dock_file_ = declare_parameter<std::string>("dock_file", default_dock_file.string());
    fleet_file_ = declare_parameter<std::string>("fleet_file", default_fleet_file.string());
    scenario_file_ =
        declare_parameter<std::string>("scenario_file", default_scenario_file.string());
    business_order_file_ = declare_parameter<std::string>(
        "business_order_file", default_business_order_file.string());
    robot_id_ = declare_parameter<std::string>("robot_id", "robot_1");
    preflight_zones_file_ =
        declare_parameter<std::string>("preflight_zones_file", default_zones_file.string());
    preflight_map_name_ = declare_parameter<std::string>("preflight_map_name", "indoor_room");
    action_name_ = declare_parameter<std::string>("action_name", "/navigate_sequence");
    pause_service_name_ = declare_parameter<std::string>("pause_service_name", "/mission/pause");
    resume_service_name_ = declare_parameter<std::string>("resume_service_name", "/mission/resume");
    cancel_service_name_ = declare_parameter<std::string>("cancel_service_name", "/mission/cancel");
    auto_start_queue_ = declare_parameter<bool>("auto_start_queue", true);
    return_to_dock_after_mission_ = declare_parameter<bool>("return_to_dock_after_mission", false);
    return_to_dock_on_low_battery_ = declare_parameter<bool>("return_to_dock_on_low_battery", true);
    autostart_ = declare_parameter<bool>("autostart", false);
    dock_priority_ = declare_parameter<int>("dock_priority", 100);
    server_timeout_ms_ = declare_parameter<int>("server_timeout_ms", 5000);
    dock_low_battery_voltage_ = declare_parameter<double>("dock_low_battery_voltage", 22.0);
    opportunity_charge_threshold_voltage_ =
        declare_parameter<double>("opportunity_charge_threshold_voltage", 23.2);
    critical_charge_threshold_voltage_ =
        declare_parameter<double>("critical_charge_threshold_voltage", dock_low_battery_voltage_);
    opportunity_charge_priority_ = declare_parameter<int>("opportunity_charge_priority", 75);
    opportunity_charge_require_idle_queue_ =
        declare_parameter<bool>("opportunity_charge_require_idle_queue", true);
    estimate_nominal_speed_mps_ = declare_parameter<double>("estimate_nominal_speed_mps", 0.35);
    estimate_battery_drop_per_meter_ =
        declare_parameter<double>("estimate_battery_drop_per_meter", 0.015);
    estimate_min_battery_voltage_ =
        declare_parameter<double>("estimate_min_battery_voltage", dock_low_battery_voltage_);
    estimate_use_nav2_path_ = declare_parameter<bool>("estimate_use_nav2_path", true);
    estimate_nav2_compute_path_action_ =
        declare_parameter<std::string>("estimate_nav2_compute_path_action", "/compute_path_to_pose");
    estimate_nav2_planner_id_ = declare_parameter<std::string>("estimate_nav2_planner_id", "");
    estimate_nav2_server_timeout_ms_ =
        declare_parameter<int>("estimate_nav2_server_timeout_ms", 50);
    estimate_nav2_result_timeout_ms_ =
        declare_parameter<int>("estimate_nav2_result_timeout_ms", 1500);
    failure_recovery_enabled_ = declare_parameter<bool>("failure_recovery_enabled", true);
    failure_recovery_policy_ =
        declare_parameter<std::string>("failure_recovery_policy", "retry_then_dock");
    failure_retry_limit_ = declare_parameter<int>("failure_retry_limit", 1);
    failure_recovery_priority_ = declare_parameter<int>("failure_recovery_priority", 90);
    preflight_enabled_ = declare_parameter<bool>("preflight_enabled", true);
    preflight_require_station_route_ =
        declare_parameter<bool>("preflight_require_station_route", true);
    preflight_default_speed_limit_mps_ =
        declare_parameter<double>("preflight_default_speed_limit_mps", 0.6);
    traffic_intersection_locks_enabled_ =
        declare_parameter<bool>("traffic_intersection_locks_enabled", true);
    payload_capacity_kg_ = declare_parameter<double>("payload_capacity_kg", 30.0);
    top_module_type_ = declare_parameter<std::string>("top_module_type", "manual_load");
    localization_covariance_threshold_ =
        declare_parameter<double>("localization_covariance_threshold", 0.5);
    localization_pause_on_lost_ = declare_parameter<bool>("localization_pause_on_lost", true);
    safety_zone_speed_limit_mps_ =
        declare_parameter<double>("safety_zone_speed_limit_mps", preflight_default_speed_limit_mps_);
    safety_obstacle_pause_on_blockage_ =
        declare_parameter<bool>("safety_obstacle_pause_on_blockage", true);

    action_client_ = rclcpp_action::create_client<NavigateSequence>(this, action_name_);
    nav2_path_node_ = std::make_shared<rclcpp::Node>(
        "mission_runner_nav2_path_estimator",
        rclcpp::NodeOptions().use_global_arguments(false));
    nav2_path_client_ =
        rclcpp_action::create_client<ComputePathToPose>(
            nav2_path_node_, estimate_nav2_compute_path_action_);
    pause_client_ = create_client<std_srvs::srv::Trigger>(pause_service_name_);
    resume_client_ = create_client<std_srvs::srv::Trigger>(resume_service_name_);
    cancel_client_ = create_client<std_srvs::srv::Trigger>(cancel_service_name_);
    state_publisher_ = create_publisher<robot_interfaces_core::msg::RobotState>("/mission/state", 10);
    payload_state_publisher_ =
        create_publisher<robot_interfaces_facility::msg::PayloadState>("/facility/payload_state", 10);
    localization_health_publisher_ =
        create_publisher<robot_interfaces_navigation::msg::LocalizationHealth>("/navigation/localization_health", 10);
    safety_state_publisher_ =
        create_publisher<robot_interfaces_navigation::msg::SafetyState>("/navigation/safety_state", 10);
    initial_pose_publisher_ =
        create_publisher<geometry_msgs::msg::PoseWithCovarianceStamped>("/initialpose", 10);
    chassis_subscription_ = create_subscription<robot_interfaces_core::msg::ChassisState>(
        "/chassis/state", 10,
        [this](robot_interfaces_core::msg::ChassisState::SharedPtr msg) { HandleChassisState(*msg); });
    amcl_pose_subscription_ = create_subscription<geometry_msgs::msg::PoseWithCovarianceStamped>(
        "/amcl_pose", 10,
        [this](geometry_msgs::msg::PoseWithCovarianceStamped::SharedPtr msg) {
          HandleLocalizationPose(*msg);
        });
    start_service_ = create_service<std_srvs::srv::Trigger>(
        "/mission/start_profile",
        [this](const std::shared_ptr<std_srvs::srv::Trigger::Request>,
               std::shared_ptr<std_srvs::srv::Trigger::Response> response) {
          StartMission(response);
        });
    reload_service_ = create_service<std_srvs::srv::Trigger>(
        "/mission/reload_profile",
        [this](const std::shared_ptr<std_srvs::srv::Trigger::Request>,
               std::shared_ptr<std_srvs::srv::Trigger::Response> response) {
          LoadProfile(response);
        });
    pause_service_ = create_service<std_srvs::srv::Trigger>(
        "/mission/pause_profile",
        [this](const std::shared_ptr<std_srvs::srv::Trigger::Request>,
               std::shared_ptr<std_srvs::srv::Trigger::Response> response) {
          PauseMission(response);
        });
    resume_service_ = create_service<std_srvs::srv::Trigger>(
        "/mission/resume_profile",
        [this](const std::shared_ptr<std_srvs::srv::Trigger::Request>,
               std::shared_ptr<std_srvs::srv::Trigger::Response> response) {
          ResumeMission(response);
        });
    cancel_service_ = create_service<std_srvs::srv::Trigger>(
        "/mission/cancel_profile",
        [this](const std::shared_ptr<std_srvs::srv::Trigger::Request>,
               std::shared_ptr<std_srvs::srv::Trigger::Response> response) {
          CancelMission(response);
        });
    recovery_state_service_ =
        create_service<robot_interfaces_mission::srv::GetMissionRecoveryState>(
            "/mission/get_recovery_state",
            [this](
                const std::shared_ptr<
                    robot_interfaces_mission::srv::GetMissionRecoveryState::Request>,
                std::shared_ptr<
                    robot_interfaces_mission::srv::GetMissionRecoveryState::Response>
                    response) { GetMissionRecoveryState(response); });
    list_mission_events_service_ =
        create_service<robot_interfaces_mission::srv::ListMissionEvents>(
            "/mission/list_events",
            [this](
                const std::shared_ptr<robot_interfaces_mission::srv::ListMissionEvents::Request>
                    request,
                std::shared_ptr<robot_interfaces_mission::srv::ListMissionEvents::Response>
                    response) { ListMissionEvents(request, response); });
    reset_recovery_service_ = create_service<std_srvs::srv::Trigger>(
        "/mission/reset_recovery_state",
        [this](const std::shared_ptr<std_srvs::srv::Trigger::Request>,
               std::shared_ptr<std_srvs::srv::Trigger::Response> response) {
          ResetMissionRecoveryState(response);
        });
    enqueue_service_ = create_service<robot_interfaces_mission::srv::EnqueueMission>(
        "/mission/enqueue",
        [this](const std::shared_ptr<robot_interfaces_mission::srv::EnqueueMission::Request>
                   request,
               std::shared_ptr<robot_interfaces_mission::srv::EnqueueMission::Response>
                   response) { EnqueueMission(request, response); });
    cancel_queued_service_ =
        create_service<robot_interfaces_mission::srv::CancelQueuedMission>(
            "/mission/cancel_queued",
            [this](
                const std::shared_ptr<robot_interfaces_mission::srv::CancelQueuedMission::Request>
                    request,
                std::shared_ptr<robot_interfaces_mission::srv::CancelQueuedMission::Response>
                    response) { CancelQueuedMission(request, response); });
    cancel_workflow_service_ = create_service<robot_interfaces_business::srv::CancelWorkflow>(
        "/business/cancel_workflow",
        [this](const std::shared_ptr<robot_interfaces_business::srv::CancelWorkflow::Request>
                   request,
               std::shared_ptr<robot_interfaces_business::srv::CancelWorkflow::Response>
                   response) { CancelWorkflow(request, response); });
    pause_workflow_service_ = create_service<robot_interfaces_business::srv::PauseWorkflow>(
        "/business/pause_workflow",
        [this](const std::shared_ptr<robot_interfaces_business::srv::PauseWorkflow::Request>
                   request,
               std::shared_ptr<robot_interfaces_business::srv::PauseWorkflow::Response>
                   response) { PauseWorkflow(request, response); });
    resume_workflow_service_ = create_service<robot_interfaces_business::srv::ResumeWorkflow>(
        "/business/resume_workflow",
        [this](const std::shared_ptr<robot_interfaces_business::srv::ResumeWorkflow::Request>
                   request,
               std::shared_ptr<robot_interfaces_business::srv::ResumeWorkflow::Response>
                   response) { ResumeWorkflow(request, response); });
    get_workflow_status_service_ =
        create_service<robot_interfaces_business::srv::GetWorkflowStatus>(
            "/business/get_workflow_status",
            [this](
                const std::shared_ptr<robot_interfaces_business::srv::GetWorkflowStatus::Request>
                    request,
                std::shared_ptr<robot_interfaces_business::srv::GetWorkflowStatus::Response>
                    response) { GetWorkflowStatus(request, response); });
    reprioritize_queued_mission_service_ =
        create_service<robot_interfaces_mission::srv::ReprioritizeQueuedMission>(
            "/mission/reprioritize",
            [this](
                const std::shared_ptr<
                    robot_interfaces_mission::srv::ReprioritizeQueuedMission::Request>
                    request,
                std::shared_ptr<
                    robot_interfaces_mission::srv::ReprioritizeQueuedMission::Response>
                    response) { ReprioritizeQueuedMission(request, response); });
    reprioritize_batch_service_ =
        create_service<robot_interfaces_mission::srv::ReprioritizeBatch>(
            "/mission/reprioritize_batch",
            [this](
                const std::shared_ptr<robot_interfaces_mission::srv::ReprioritizeBatch::Request>
                    request,
                std::shared_ptr<robot_interfaces_mission::srv::ReprioritizeBatch::Response>
                    response) { ReprioritizeBatch(request, response); });
    preempt_service_ = create_service<robot_interfaces_mission::srv::PreemptMission>(
        "/mission/preempt",
        [this](const std::shared_ptr<robot_interfaces_mission::srv::PreemptMission::Request>
                   request,
               std::shared_ptr<robot_interfaces_mission::srv::PreemptMission::Response>
                   response) { PreemptMission(request, response); });
    estimate_mission_service_ =
        create_service<robot_interfaces_mission::srv::EstimateMissionCost>(
            "/mission/estimate_cost",
            [this](
                const std::shared_ptr<robot_interfaces_mission::srv::EstimateMissionCost::Request>
                    request,
                std::shared_ptr<robot_interfaces_mission::srv::EstimateMissionCost::Response>
                    response) { EstimateMission(request, response); });
    submit_unified_order_service_ =
        create_service<robot_interfaces_business::srv::SubmitOrder>(
            "/business/submit_order",
            [this](
                const std::shared_ptr<robot_interfaces_business::srv::SubmitOrder::Request>
                    request,
                std::shared_ptr<robot_interfaces_business::srv::SubmitOrder::Response>
                    response) { SubmitOrder(request, response); });
    list_stations_service_ = create_service<robot_interfaces_mission::srv::ListStations>(
        "/mission/list_stations",
        [this](const std::shared_ptr<robot_interfaces_mission::srv::ListStations::Request>,
               std::shared_ptr<robot_interfaces_mission::srv::ListStations::Response>
                   response) { ListStations(response); });
    list_station_routes_service_ =
        create_service<robot_interfaces_mission::srv::ListStationRoutes>(
            "/mission/list_station_routes",
            [this](
                const std::shared_ptr<robot_interfaces_mission::srv::ListStationRoutes::Request>,
                std::shared_ptr<robot_interfaces_mission::srv::ListStationRoutes::Response>
                    response) { ListStationRoutes(response); });
    block_station_route_service_ =
        create_service<robot_interfaces_mission::srv::BlockStationRoute>(
            "/mission/block_route",
            [this](
                const std::shared_ptr<robot_interfaces_mission::srv::BlockStationRoute::Request>
                    request,
                std::shared_ptr<robot_interfaces_mission::srv::BlockStationRoute::Response>
                    response) { BlockStationRoute(request, response); });
    unblock_station_route_service_ =
        create_service<robot_interfaces_mission::srv::UnblockStationRoute>(
            "/mission/unblock_route",
            [this](
                const std::shared_ptr<robot_interfaces_mission::srv::UnblockStationRoute::Request>
                    request,
                std::shared_ptr<robot_interfaces_mission::srv::UnblockStationRoute::Response>
                    response) { UnblockStationRoute(request, response); });
    list_traffic_reservations_service_ =
        create_service<robot_interfaces_mission::srv::ListTrafficReservations>(
            "/mission/list_reservations",
            [this](
                const std::shared_ptr<
                    robot_interfaces_mission::srv::ListTrafficReservations::Request>,
                std::shared_ptr<
                    robot_interfaces_mission::srv::ListTrafficReservations::Response>
                    response) { ListTrafficReservations(response); });
    clear_traffic_reservation_service_ =
        create_service<robot_interfaces_mission::srv::ClearTrafficReservation>(
            "/mission/clear_reservation",
            [this](
                const std::shared_ptr<
                    robot_interfaces_mission::srv::ClearTrafficReservation::Request>
                    request,
                std::shared_ptr<
                    robot_interfaces_mission::srv::ClearTrafficReservation::Response>
                    response) { ClearTrafficReservation(request, response); });
    detect_traffic_deadlock_service_ =
        create_service<robot_interfaces_mission::srv::DetectTrafficDeadlock>(
            "/mission/detect_deadlock",
            [this](
                const std::shared_ptr<
                    robot_interfaces_mission::srv::DetectTrafficDeadlock::Request>,
                std::shared_ptr<robot_interfaces_mission::srv::DetectTrafficDeadlock::Response>
                    response) { DetectTrafficDeadlock(response); });
    list_fleet_robots_service_ =
        create_service<robot_interfaces_fleet::srv::ListFleetRobots>(
            "/fleet/list_robots",
            [this](
                const std::shared_ptr<robot_interfaces_fleet::srv::ListFleetRobots::Request>
                    request,
                std::shared_ptr<robot_interfaces_fleet::srv::ListFleetRobots::Response>
                    response) { ListFleetRobots(request, response); });
    update_fleet_robot_state_service_ =
        create_service<robot_interfaces_fleet::srv::UpdateFleetRobotState>(
            "/fleet/update_robot_state",
            [this](
                const std::shared_ptr<robot_interfaces_fleet::srv::UpdateFleetRobotState::Request>
                    request,
                std::shared_ptr<robot_interfaces_fleet::srv::UpdateFleetRobotState::Response>
                    response) { UpdateFleetRobotState(request, response); });
    update_remote_task_state_service_ =
        create_service<robot_interfaces_fleet::srv::UpdateRemoteTaskState>(
            "/fleet/update_remote_task",
            [this](
                const std::shared_ptr<robot_interfaces_fleet::srv::UpdateRemoteTaskState::Request>
                    request,
                std::shared_ptr<robot_interfaces_fleet::srv::UpdateRemoteTaskState::Response>
                    response) { UpdateRemoteTaskState(request, response); });
    preview_fleet_station_task_service_ =
        create_service<robot_interfaces_fleet::srv::PreviewFleetStationTask>(
            "/fleet/preview_station_task",
            [this](
                const std::shared_ptr<
                    robot_interfaces_fleet::srv::PreviewFleetStationTask::Request>
                    request,
                std::shared_ptr<robot_interfaces_fleet::srv::PreviewFleetStationTask::Response>
                    response) { PreviewFleetStationTask(request, response); });
    submit_fleet_station_task_service_ =
        create_service<robot_interfaces_fleet::srv::SubmitFleetStationTask>(
            "/fleet/submit_station_task",
            [this](
                const std::shared_ptr<robot_interfaces_fleet::srv::SubmitFleetStationTask::Request>
                    request,
                std::shared_ptr<robot_interfaces_fleet::srv::SubmitFleetStationTask::Response>
                    response) { SubmitFleetStationTask(request, response); });
    list_business_order_types_service_ =
        create_service<robot_interfaces_business::srv::ListBusinessOrderTypes>(
            "/business/list_order_types",
            [this](
                const std::shared_ptr<
                    robot_interfaces_business::srv::ListBusinessOrderTypes::Request>,
                std::shared_ptr<robot_interfaces_business::srv::ListBusinessOrderTypes::Response>
                    response) { ListBusinessOrderTypes(response); });
    get_operator_snapshot_service_ =
        create_service<robot_interfaces_business::srv::GetOperatorSnapshot>(
            "/business/get_operator_snapshot",
            [this](
                const std::shared_ptr<robot_interfaces_business::srv::GetOperatorSnapshot::Request>
                    request,
                std::shared_ptr<robot_interfaces_business::srv::GetOperatorSnapshot::Response>
                    response) { GetOperatorSnapshot(request, response); });
    list_scenario_tasks_service_ =
        create_service<robot_interfaces_business::srv::ListScenarioTasks>(
            "/business/list_scenario_tasks",
            [this](
                const std::shared_ptr<robot_interfaces_business::srv::ListScenarioTasks::Request>
                    request,
                std::shared_ptr<robot_interfaces_business::srv::ListScenarioTasks::Response>
                    response) { ListScenarioTasks(request, response); });
    list_scenario_workflows_service_ =
        create_service<robot_interfaces_business::srv::ListScenarioWorkflows>(
            "/business/list_scenario_workflows",
            [this](
                const std::shared_ptr<
                    robot_interfaces_business::srv::ListScenarioWorkflows::Request> request,
                std::shared_ptr<robot_interfaces_business::srv::ListScenarioWorkflows::Response>
                    response) { ListScenarioWorkflows(request, response); });
    list_facility_resources_service_ =
        create_service<robot_interfaces_facility::srv::ListFacilityResources>(
            "/facility/list_resources",
            [this](
                const std::shared_ptr<
                    robot_interfaces_facility::srv::ListFacilityResources::Request>
                    request,
                std::shared_ptr<robot_interfaces_facility::srv::ListFacilityResources::Response>
                    response) { ListFacilityResources(request, response); });
    reserve_facility_resource_service_ =
        create_service<robot_interfaces_facility::srv::ReserveFacilityResource>(
            "/facility/reserve_resource",
            [this](
                const std::shared_ptr<
                    robot_interfaces_facility::srv::ReserveFacilityResource::Request>
                    request,
                std::shared_ptr<
                    robot_interfaces_facility::srv::ReserveFacilityResource::Response>
                    response) { ReserveFacilityResource(request, response); });
    release_facility_resource_service_ =
        create_service<robot_interfaces_facility::srv::ReleaseFacilityResource>(
            "/facility/release_resource",
            [this](
                const std::shared_ptr<
                    robot_interfaces_facility::srv::ReleaseFacilityResource::Request>
                    request,
                std::shared_ptr<
                    robot_interfaces_facility::srv::ReleaseFacilityResource::Response>
                    response) { ReleaseFacilityResource(request, response); });
    request_lift_session_service_ =
        create_service<robot_interfaces_facility::srv::RequestLiftSession>(
            "/facility/request_lift_session",
            [this](
                const std::shared_ptr<robot_interfaces_facility::srv::RequestLiftSession::Request>
                    request,
                std::shared_ptr<robot_interfaces_facility::srv::RequestLiftSession::Response>
                    response) { RequestLiftSession(request, response); });
    release_lift_session_service_ =
        create_service<robot_interfaces_facility::srv::ReleaseLiftSession>(
            "/facility/release_lift_session",
            [this](
                const std::shared_ptr<robot_interfaces_facility::srv::ReleaseLiftSession::Request>
                    request,
                std::shared_ptr<robot_interfaces_facility::srv::ReleaseLiftSession::Response>
                    response) { ReleaseLiftSession(request, response); });
    get_lift_state_service_ = create_service<robot_interfaces_facility::srv::GetLiftState>(
        "/facility/get_lift_state",
        [this](const std::shared_ptr<robot_interfaces_facility::srv::GetLiftState::Request>
                   request,
               std::shared_ptr<robot_interfaces_facility::srv::GetLiftState::Response>
                   response) { GetLiftState(request, response); });
    get_door_state_service_ = create_service<robot_interfaces_facility::srv::GetDoorState>(
        "/facility/get_door_state",
        [this](const std::shared_ptr<robot_interfaces_facility::srv::GetDoorState::Request>
                   request,
               std::shared_ptr<robot_interfaces_facility::srv::GetDoorState::Response>
                   response) { GetDoorState(request, response); });
    list_docks_service_ = create_service<robot_interfaces_mission::srv::ListDocks>(
        "/mission/list_docks",
        [this](const std::shared_ptr<robot_interfaces_mission::srv::ListDocks::Request>
                   request,
               std::shared_ptr<robot_interfaces_mission::srv::ListDocks::Response>
                   response) { ListDocks(request, response); });
    request_docking_service_ = create_service<robot_interfaces_mission::srv::RequestDocking>(
        "/mission/request_docking",
        [this](const std::shared_ptr<robot_interfaces_mission::srv::RequestDocking::Request>
                   request,
               std::shared_ptr<robot_interfaces_mission::srv::RequestDocking::Response>
                   response) { RequestDocking(request, response); });
    request_undocking_service_ =
        create_service<robot_interfaces_mission::srv::RequestUndocking>(
            "/mission/request_undocking",
            [this](
                const std::shared_ptr<robot_interfaces_mission::srv::RequestUndocking::Request>
                    request,
                std::shared_ptr<robot_interfaces_mission::srv::RequestUndocking::Response>
                    response) { RequestUndocking(request, response); });
    get_dock_state_service_ = create_service<robot_interfaces_mission::srv::GetDockState>(
        "/mission/get_dock_state",
        [this](const std::shared_ptr<robot_interfaces_mission::srv::GetDockState::Request>
                   request,
               std::shared_ptr<robot_interfaces_mission::srv::GetDockState::Response>
                   response) { GetDockState(request, response); });
    confirm_load_service_ = create_service<robot_interfaces_facility::srv::ConfirmLoad>(
        "/facility/confirm_load",
        [this](const std::shared_ptr<robot_interfaces_facility::srv::ConfirmLoad::Request>
                   request,
               std::shared_ptr<robot_interfaces_facility::srv::ConfirmLoad::Response>
                   response) { ConfirmLoad(request, response); });
    confirm_unload_service_ = create_service<robot_interfaces_facility::srv::ConfirmUnload>(
        "/facility/confirm_unload",
        [this](const std::shared_ptr<robot_interfaces_facility::srv::ConfirmUnload::Request>
                   request,
               std::shared_ptr<robot_interfaces_facility::srv::ConfirmUnload::Response>
                   response) { ConfirmUnload(request, response); });
    set_payload_loaded_service_ =
        create_service<robot_interfaces_facility::srv::SetPayloadLoaded>(
            "/facility/set_payload_loaded",
            [this](
                const std::shared_ptr<robot_interfaces_facility::srv::SetPayloadLoaded::Request>
                    request,
                std::shared_ptr<robot_interfaces_facility::srv::SetPayloadLoaded::Response>
                    response) { SetPayloadLoaded(request, response); });
    execute_top_module_action_service_ =
        create_service<robot_interfaces_facility::srv::ExecuteTopModuleAction>(
            "/facility/execute_top_module_action",
            [this](
                const std::shared_ptr<
                    robot_interfaces_facility::srv::ExecuteTopModuleAction::Request>
                    request,
                std::shared_ptr<robot_interfaces_facility::srv::ExecuteTopModuleAction::Response>
                    response) { ExecuteTopModuleAction(request, response); });
    request_relocalization_service_ =
        create_service<robot_interfaces_navigation::srv::RequestRelocalization>(
            "/navigation/request_relocalization",
            [this](
                const std::shared_ptr<robot_interfaces_navigation::srv::RequestRelocalization::Request>
                    request,
                std::shared_ptr<robot_interfaces_navigation::srv::RequestRelocalization::Response>
                    response) { RequestRelocalization(request, response); });
    set_initial_pose_from_station_service_ =
        create_service<robot_interfaces_navigation::srv::SetInitialPoseFromStation>(
            "/navigation/set_initial_pose",
            [this](
                const std::shared_ptr<robot_interfaces_navigation::srv::SetInitialPoseFromStation::Request>
                    request,
                std::shared_ptr<robot_interfaces_navigation::srv::SetInitialPoseFromStation::Response>
                    response) { SetInitialPoseFromStation(request, response); });
    set_dynamic_speed_limit_service_ =
        create_service<robot_interfaces_navigation::srv::SetDynamicSpeedLimit>(
            "/navigation/set_speed_limit",
            [this](const std::shared_ptr<robot_interfaces_navigation::srv::SetDynamicSpeedLimit::Request>
                       request,
                   std::shared_ptr<robot_interfaces_navigation::srv::SetDynamicSpeedLimit::Response>
                       response) { SetDynamicSpeedLimit(request, response); });
    report_obstacle_blockage_service_ =
        create_service<robot_interfaces_navigation::srv::ReportObstacleBlockage>(
            "/navigation/report_obstacle",
            [this](const std::shared_ptr<robot_interfaces_navigation::srv::ReportObstacleBlockage::Request>
                       request,
                   std::shared_ptr<robot_interfaces_navigation::srv::ReportObstacleBlockage::Response>
                       response) { ReportObstacleBlockage(request, response); });
    clear_obstacle_blockage_service_ =
        create_service<robot_interfaces_navigation::srv::ClearObstacleBlockage>(
            "/navigation/clear_obstacle",
            [this](const std::shared_ptr<robot_interfaces_navigation::srv::ClearObstacleBlockage::Request>
                       request,
                   std::shared_ptr<robot_interfaces_navigation::srv::ClearObstacleBlockage::Response>
                       response) { ClearObstacleBlockage(request, response); });
    request_charging_service_ =
        create_service<robot_interfaces_mission::srv::RequestCharging>(
            "/mission/request_charging",
            [this](
                const std::shared_ptr<robot_interfaces_mission::srv::RequestCharging::Request>
                    request,
                std::shared_ptr<robot_interfaces_mission::srv::RequestCharging::Response>
                    response) { RequestCharging(request, response); });
    request_opportunity_charging_service_ =
        create_service<robot_interfaces_mission::srv::RequestOpportunityCharging>(
            "/mission/request_opportunity_charging",
            [this](
                const std::shared_ptr<
                    robot_interfaces_mission::srv::RequestOpportunityCharging::Request>
                    request,
                std::shared_ptr<
                    robot_interfaces_mission::srv::RequestOpportunityCharging::Response>
                    response) { RequestOpportunityCharging(request, response); });
    execute_facility_action_service_ =
        create_service<robot_interfaces_facility::srv::ExecuteFacilityAction>(
            "/facility/execute_action",
            [this](
                const std::shared_ptr<
                    robot_interfaces_facility::srv::ExecuteFacilityAction::Request>
                    request,
                std::shared_ptr<robot_interfaces_facility::srv::ExecuteFacilityAction::Response>
                    response) { ExecuteFacilityAction(request, response); });
    request_station_confirmation_service_ =
        create_service<robot_interfaces_facility::srv::RequestStationConfirmation>(
            "/facility/request_station_confirmation",
            [this](
                const std::shared_ptr<
                    robot_interfaces_facility::srv::RequestStationConfirmation::Request>
                    request,
                std::shared_ptr<
                    robot_interfaces_facility::srv::RequestStationConfirmation::Response>
                    response) { RequestStationConfirmation(request, response); });
    get_pending_confirmations_service_ =
        create_service<robot_interfaces_facility::srv::GetPendingConfirmations>(
            "/facility/get_pending_confirmations",
            [this](
                const std::shared_ptr<
                    robot_interfaces_facility::srv::GetPendingConfirmations::Request>
                    request,
                std::shared_ptr<robot_interfaces_facility::srv::GetPendingConfirmations::Response>
                    response) { GetPendingConfirmations(request, response); });
    confirm_station_action_service_ =
        create_service<robot_interfaces_facility::srv::ConfirmStationAction>(
            "/facility/confirm_station_action",
            [this](
                const std::shared_ptr<
                    robot_interfaces_facility::srv::ConfirmStationAction::Request>
                    request,
                std::shared_ptr<robot_interfaces_facility::srv::ConfirmStationAction::Response>
                    response) { ConfirmStationAction(request, response); });
    reject_station_action_service_ =
        create_service<robot_interfaces_facility::srv::RejectStationAction>(
            "/facility/reject_station_action",
            [this](
                const std::shared_ptr<
                    robot_interfaces_facility::srv::RejectStationAction::Request>
                    request,
                std::shared_ptr<robot_interfaces_facility::srv::RejectStationAction::Response>
                    response) { RejectStationAction(request, response); });
    confirm_pick_service_ =
        create_service<robot_interfaces_business::srv::ConfirmPick>(
            "/business/confirm_pick",
            [this](
                const std::shared_ptr<robot_interfaces_business::srv::ConfirmPick::Request>
                    request,
                std::shared_ptr<robot_interfaces_business::srv::ConfirmPick::Response>
                    response) { ConfirmPick(request, response); });
    confirm_dropoff_service_ =
        create_service<robot_interfaces_business::srv::ConfirmDropoff>(
            "/business/confirm_dropoff",
            [this](
                const std::shared_ptr<robot_interfaces_business::srv::ConfirmDropoff::Request>
                    request,
                std::shared_ptr<robot_interfaces_business::srv::ConfirmDropoff::Response>
                    response) { ConfirmDropoff(request, response); });
    report_short_pick_service_ =
        create_service<robot_interfaces_business::srv::ReportShortPick>(
            "/business/report_short_pick",
            [this](
                const std::shared_ptr<robot_interfaces_business::srv::ReportShortPick::Request>
                    request,
                std::shared_ptr<robot_interfaces_business::srv::ReportShortPick::Response>
                    response) { ReportShortPick(request, response); });
    get_order_fulfillment_status_service_ =
        create_service<robot_interfaces_business::srv::GetOrderFulfillmentStatus>(
            "/business/get_fulfillment_status",
            [this](
                const std::shared_ptr<
                    robot_interfaces_business::srv::GetOrderFulfillmentStatus::Request> request,
                std::shared_ptr<
                    robot_interfaces_business::srv::GetOrderFulfillmentStatus::Response>
                    response) { GetOrderFulfillmentStatus(request, response); });
    submit_station_order_service_ =
        create_service<robot_interfaces_mission::srv::SubmitStationTransportOrder>(
            "/mission/submit_station_transport",
            [this](
                const std::shared_ptr<
                    robot_interfaces_mission::srv::SubmitStationTransportOrder::Request>
                    request,
                std::shared_ptr<
                    robot_interfaces_mission::srv::SubmitStationTransportOrder::Response>
                    response) { SubmitStationTransportOrder(request, response); });
    submit_station_order_batch_service_ =
        create_service<robot_interfaces_mission::srv::SubmitStationOrderBatch>(
            "/mission/submit_station_batch",
            [this](
                const std::shared_ptr<
                    robot_interfaces_mission::srv::SubmitStationOrderBatch::Request>
                    request,
                std::shared_ptr<robot_interfaces_mission::srv::SubmitStationOrderBatch::Response>
                    response) { SubmitStationOrderBatch(request, response); });
    cancel_station_order_batch_service_ =
        create_service<robot_interfaces_mission::srv::CancelStationOrderBatch>(
            "/mission/cancel_station_batch",
            [this](
                const std::shared_ptr<
                    robot_interfaces_mission::srv::CancelStationOrderBatch::Request>
                    request,
                std::shared_ptr<robot_interfaces_mission::srv::CancelStationOrderBatch::Response>
                    response) { CancelStationOrderBatch(request, response); });
    submit_station_sequence_service_ =
        create_service<robot_interfaces_mission::srv::SubmitStationSequenceTask>(
            "/mission/submit_station_sequence",
            [this](
                const std::shared_ptr<
                    robot_interfaces_mission::srv::SubmitStationSequenceTask::Request>
                    request,
                std::shared_ptr<robot_interfaces_mission::srv::SubmitStationSequenceTask::Response>
                    response) { SubmitStationSequenceTask(request, response); });
    estimate_station_transport_service_ =
        create_service<robot_interfaces_mission::srv::EstimateStationTransport>(
            "/mission/estimate_station_transport",
            [this](
                const std::shared_ptr<
                    robot_interfaces_mission::srv::EstimateStationTransport::Request>
                    request,
                std::shared_ptr<robot_interfaces_mission::srv::EstimateStationTransport::Response>
                    response) { EstimateStationTransport(request, response); });
    estimate_station_sequence_service_ =
        create_service<robot_interfaces_mission::srv::EstimateStationSequence>(
            "/mission/estimate_station_sequence",
            [this](
                const std::shared_ptr<
                    robot_interfaces_mission::srv::EstimateStationSequence::Request>
                    request,
                std::shared_ptr<robot_interfaces_mission::srv::EstimateStationSequence::Response>
                    response) { EstimateStationSequence(request, response); });
    return_to_dock_service_ = create_service<std_srvs::srv::Trigger>(
        "/mission/return_to_dock",
        [this](const std::shared_ptr<std_srvs::srv::Trigger::Request>,
               std::shared_ptr<std_srvs::srv::Trigger::Response> response) {
          RequestDockReturn("manual return-to-dock request", response);
        });
    LoadProfile(nullptr);
    LoadStations(std::shared_ptr<robot_interfaces_mission::srv::ListStations::Response>{});
    timer_ = create_wall_timer(200ms, [this]() { Tick(); });
  }

 private:
  using QueuedMission = robot_tasks::QueuedMission;
  using DockRuntimeState = robot_tasks::DockingRuntimeState;
  using StationConfirmation = robot_tasks::StationConfirmationRuntimeState;
  using FulfillmentOrder = robot_tasks::FulfillmentOrderRuntimeState;

  using RemoteTaskState = robot_tasks::RemoteFleetTaskState;

  struct StationTransportExecutionRecord {
    robot_tasks::StationTransportRuntimeState state;
    robot_tasks::MissionProfile pickup_profile;
    robot_tasks::MissionProfile dropoff_profile;
  };

  struct StationTransportDispatchContext {
    std::string mission_id;
    robot_tasks::StationTransportLeg leg = robot_tasks::StationTransportLeg::kNone;
    std::uint64_t dispatch_token = 0;
  };

  bool RunnerBusy() const {
    if (mission_active_) {
      return true;
    }
    const auto it = station_transport_executions_.find(active_station_transport_id_);
    return it != station_transport_executions_.end() &&
           robot_tasks::IsStationTransportBusy(it->second.state.phase);
  }

  std::string ActiveMissionId() const {
    if (!active_station_transport_id_.empty()) {
      return active_station_transport_id_;
    }
    return profile_.has_value() ? profile_->mission_id : "";
  }

  bool HasPendingStationTransport() const {
    return std::any_of(
        station_transport_executions_.begin(), station_transport_executions_.end(),
        [](const auto& item) {
          return !robot_tasks::IsStationTransportTerminal(item.second.state.phase);
        });
  }

  void RequestPendingStationTransportStartAfterPayloadCleared() {
    if (!payload_loaded_ && queue_running_ && HasPendingStationTransport()) {
      queue_start_requested_ = true;
    }
  }

  StationTransportExecutionRecord* FindActiveStationTransport() {
    const auto it = station_transport_executions_.find(active_station_transport_id_);
    return it == station_transport_executions_.end() ? nullptr : &it->second;
  }

  const StationTransportExecutionRecord* FindActiveStationTransport() const {
    const auto it = station_transport_executions_.find(active_station_transport_id_);
    return it == station_transport_executions_.end() ? nullptr : &it->second;
  }

  void LoadProfile(const std::shared_ptr<std_srvs::srv::Trigger::Response>& response) {
    const auto profile = robot_tasks::LoadMissionProfile(mission_file_);
    if (!profile.has_value()) {
      profile_.reset();
      SetState("ERROR", "failed to load mission profile: " + mission_file_, false);
      if (response != nullptr) {
        response->success = false;
        response->message = last_message_;
      }
      return;
    }
    profile_ = *profile;
    SetState(
        "READY",
        "loaded mission " + profile_->mission_id + " with " +
            std::to_string(profile_->waypoints.size()) + " waypoint(s)",
        true);
    if (response != nullptr) {
      response->success = true;
      response->message = last_message_;
    }
  }

  void StartMission(const std::shared_ptr<std_srvs::srv::Trigger::Response>& response) {
    if (RunnerBusy()) {
      if (response != nullptr) {
        response->success = false;
        response->message = "mission already active";
      }
      return;
    }
    if (!mission_queue_.empty()) {
      queue_running_ = true;
      StartNextQueuedMission(response);
      return;
    }
    DispatchMission(response, "mission profile sent");
  }

  template <typename RequestT, typename ResponseT>
  void EnqueueMission(
      const std::shared_ptr<RequestT>& request,
      const std::shared_ptr<ResponseT>& response) {
    const auto profile = robot_tasks::LoadMissionProfile(request->mission_file);
    if (!profile.has_value()) {
      response->success = false;
      response->message = "failed to load mission profile: " + request->mission_file;
      response->queue_size = static_cast<int>(mission_queue_.size());
      return;
    }
    const auto duplicate_gate = robot_tasks::PlanMissionSubmitGate(
        robot_tasks::MissionSubmitGateRequest{
            profile->mission_id, "mission", "mission",
            MissionIdActiveOrQueued(profile->mission_id)});
    if (!duplicate_gate.accepted) {
      response->success = false;
      response->message = duplicate_gate.message;
      response->queue_size = static_cast<int>(mission_queue_.size());
      return;
    }
    const auto preflight = ValidateMissionForQueue(*profile);
    const auto gate = robot_tasks::PlanMissionSubmitGate(
        robot_tasks::MissionSubmitGateRequest{
            profile->mission_id, "mission", "mission", false, preflight});
    if (!gate.accepted) {
      response->success = false;
      response->message = gate.message;
      response->queue_size = static_cast<int>(mission_queue_.size());
      return;
    }

    const auto admission = AdmitQueuedMission(
        QueuedMission{*profile, request->mission_file, request->priority,
                      next_queue_sequence_++},
        request->start_if_idle, false, "",
        "queued mission " + profile->mission_id + " priority " +
            std::to_string(request->priority));
    response->success = admission.success;
    response->queue_size = admission.queue_size;
    response->message = admission.message;
    ApplyMissionQueueAdmissionState(admission);
  }

  template <typename RequestT, typename ResponseT>
  void CancelQueuedMission(
      const std::shared_ptr<RequestT>& request,
      const std::shared_ptr<ResponseT>& response) {
    const auto mission_id = SanitizeId(request->mission_id);
    if (mission_id.empty()) {
      response->success = false;
      response->message = "mission_id is empty";
      response->queue_size = static_cast<int>(mission_queue_.size());
      return;
    }

    robot_tasks::QueuedMissionCancelRequest cancel_request;
    cancel_request.queue = mission_queue_;
    cancel_request.mission_id = mission_id;
    cancel_request.cancel_active = request->cancel_active;
    cancel_request.mission_active = RunnerBusy();
    cancel_request.active_mission_id = ActiveMissionId();
    const auto decision = robot_tasks::PlanQueuedMissionCancel(cancel_request);
    mission_queue_ = decision.remaining_queue;
    for (const auto& released_mission_id : decision.release_mission_ids) {
      ReleaseAllResourcesForMission(released_mission_id);
      station_transport_executions_.erase(released_mission_id);
    }
    if (decision.active_cancel_requested) {
      RequestActiveMissionCancel("active mission cancellation requested by id");
    }

    response->success = decision.success;
    response->message = decision.message;
    response->queue_size = decision.queue_size;
    if (decision.set_state) {
      SetState(decision.state, decision.state_message, decision.state_recoverable);
    }
  }

  bool MissionBelongsToWorkflowOrder(
      const std::string& mission_id, const std::string& order_id) const {
    return robot_tasks::MissionBelongsToWorkflowOrder(mission_id, order_id);
  }

  bool MissionBelongsToStationOrderBatch(
      const std::string& mission_id, const std::string& batch_id) const {
    return robot_tasks::MissionBelongsToStationOrderBatch(mission_id, batch_id);
  }

  template <typename RequestT, typename ResponseT>
  void CancelWorkflow(
      const std::shared_ptr<RequestT>& request,
      const std::shared_ptr<ResponseT>& response) {
    const auto order_id = SanitizeId(request->order_id);
    if (order_id.empty()) {
      response->success = false;
      response->message = "order_id is empty";
      response->queue_size = static_cast<int>(mission_queue_.size());
      return;
    }

    const auto decision = robot_tasks::PlanWorkflowCancel(
        robot_tasks::WorkflowCancelRequest{
            order_id, request->cancel_active, mission_queue_,
            robot_tasks::WorkflowActiveMission{
                RunnerBusy(), paused_, ActiveMissionId()}});
    mission_queue_ = decision.remaining_queue;
    response->canceled_mission_ids = decision.canceled_mission_ids;
    response->active_cancel_requested = decision.active_cancel_requested;
    for (const auto& mission_id : response->canceled_mission_ids) {
      if (decision.active_cancel_requested && mission_id == ActiveMissionId()) {
        continue;
      }
      ReleaseAllResourcesForMission(mission_id);
      station_transport_executions_.erase(mission_id);
    }

    if (decision.active_cancel_requested) {
      RequestActiveMissionCancel("workflow cancellation requested for " + order_id);
    }

    response->queue_size = static_cast<int>(mission_queue_.size());
    response->success = decision.success;
    response->message = decision.message;
    if (response->success && !RunnerBusy()) {
      SetState("QUEUED", response->message, true);
    }
  }

  template <typename RequestT, typename ResponseT>
  void PauseWorkflow(
      const std::shared_ptr<RequestT>& request,
      const std::shared_ptr<ResponseT>& response) {
    const auto order_id = SanitizeId(request->order_id);
    if (order_id.empty()) {
      response->success = false;
      response->message = "order_id is empty";
      response->queue_size = static_cast<int>(mission_queue_.size());
      return;
    }

    const auto decision = robot_tasks::PlanWorkflowPause(
        robot_tasks::WorkflowPauseRequest{
            order_id, request->pause_active, mission_queue_, paused_workflow_orders_,
            robot_tasks::WorkflowActiveMission{
                RunnerBusy(), paused_, ActiveMissionId()}});
    paused_workflow_orders_ = decision.paused_order_ids;
    response->affected_mission_ids = decision.affected_mission_ids;
    response->active_pause_requested = decision.active_pause_requested;

    if (decision.active_pause_requested) {
      paused_ = decision.runner_paused;
      SendControlRequest<std_srvs::srv::Trigger>(pause_client_, pause_service_name_);
      SetState("PAUSED", "workflow pause requested for " + order_id, true);
    }

    if (decision.success) {
      if (decision.clear_queue_start_requested) {
        queue_start_requested_ = false;
      }
      response->success = true;
      response->message = decision.message;
    } else {
      response->success = false;
      response->message = decision.message;
    }
    response->queue_size = static_cast<int>(mission_queue_.size());
    if (response->success && !RunnerBusy()) {
      SetState("WORKFLOW_PAUSED", response->message, true);
    }
  }

  template <typename RequestT, typename ResponseT>
  void ResumeWorkflow(
      const std::shared_ptr<RequestT>& request,
      const std::shared_ptr<ResponseT>& response) {
    const auto order_id = SanitizeId(request->order_id);
    if (order_id.empty()) {
      response->success = false;
      response->message = "order_id is empty";
      response->queue_size = static_cast<int>(mission_queue_.size());
      return;
    }

    const auto decision = robot_tasks::PlanWorkflowResume(
        robot_tasks::WorkflowResumeRequest{
            order_id, request->resume_active, mission_queue_, paused_workflow_orders_,
            robot_tasks::WorkflowActiveMission{
                RunnerBusy(), paused_, ActiveMissionId()}});
    paused_workflow_orders_ = decision.paused_order_ids;
    response->affected_mission_ids = decision.affected_mission_ids;
    response->active_resume_requested = decision.active_resume_requested;

    if (decision.active_resume_requested) {
      paused_ = decision.runner_paused;
      SendControlRequest<std_srvs::srv::Trigger>(resume_client_, resume_service_name_);
      SetState("RUNNING", "workflow resume requested for " + order_id, true);
    }

    if (decision.success) {
      queue_running_ = queue_running_ || decision.queue_running;
      queue_start_requested_ = queue_start_requested_ || decision.queue_start_requested;
      response->success = true;
      response->message = decision.message;
    } else {
      response->success = false;
      response->message = decision.message;
    }
    response->queue_size = static_cast<int>(mission_queue_.size());
  }

  template <typename RequestT, typename ResponseT>
  void GetWorkflowStatus(
      const std::shared_ptr<RequestT>& request,
      const std::shared_ptr<ResponseT>& response) const {
    const auto order_id = SanitizeId(request->order_id);
    response->order_id = order_id;
    if (order_id.empty()) {
      response->success = false;
      response->message = "order_id is empty";
      response->state = "INVALID";
      return;
    }

    const auto snapshot = robot_tasks::BuildWorkflowStatusSnapshot(
        robot_tasks::WorkflowStatusRequest{
            order_id, mission_queue_,
            robot_tasks::WorkflowActiveMission{
                RunnerBusy(), paused_, ActiveMissionId()},
            paused_workflow_orders_, mission_events_, current_state_});
    response->queued_mission_ids = snapshot.queued_mission_ids;
    response->running_mission_id = snapshot.running_mission_id;
    response->finished_mission_ids = snapshot.finished_mission_ids;
    response->paused = snapshot.paused;
    response->total_steps = snapshot.total_steps;
    response->finished_steps = snapshot.finished_steps;
    response->queued_steps = snapshot.queued_steps;
    response->state = snapshot.state;
    response->success = snapshot.success;
    response->message = snapshot.message;
  }

  template <typename RequestT, typename ResponseT>
  void ReprioritizeQueuedMission(
      const std::shared_ptr<RequestT>& request,
      const std::shared_ptr<ResponseT>& response) {
    const auto mission_id = SanitizeId(request->mission_id);
    if (mission_id.empty()) {
      response->success = false;
      response->message = "mission_id is empty";
      response->queue_size = static_cast<int>(mission_queue_.size());
      return;
    }
    robot_tasks::QueuedMissionReprioritizeRequest reprioritize_request;
    reprioritize_request.queue = mission_queue_;
    reprioritize_request.mission_id = mission_id;
    reprioritize_request.priority = request->priority;
    reprioritize_request.mission_active = RunnerBusy();
    const auto decision = robot_tasks::PlanQueuedMissionReprioritize(reprioritize_request);
    mission_queue_ = decision.remaining_queue;
    response->success = decision.success;
    response->queue_size = decision.queue_size;
    response->message = decision.message;
    if (decision.set_state) {
      SetState(decision.state, decision.state_message, decision.state_recoverable);
    }
  }

  template <typename RequestT, typename ResponseT>
  void ReprioritizeBatch(
      const std::shared_ptr<RequestT>& request,
      const std::shared_ptr<ResponseT>& response) {
    std::vector<std::string> mission_ids;
    mission_ids.reserve(request->mission_ids.size());
    for (const auto& mission_id : request->mission_ids) {
      mission_ids.push_back(SanitizeId(mission_id));
    }
    robot_tasks::QueuedMissionBatchReprioritizeRequest reprioritize_request;
    reprioritize_request.queue = mission_queue_;
    reprioritize_request.mission_ids = mission_ids;
    reprioritize_request.priorities = request->priorities;
    reprioritize_request.mission_active = RunnerBusy();
    const auto decision = robot_tasks::PlanQueuedMissionBatchReprioritize(reprioritize_request);
    mission_queue_ = decision.remaining_queue;
    response->updated_mission_ids = decision.updated_mission_ids;
    response->success = decision.success;
    response->queue_size = decision.queue_size;
    response->message = decision.message;
    if (decision.set_state) {
      SetState(decision.state, decision.state_message, decision.state_recoverable);
    }
  }

  template <typename RequestT, typename ResponseT>
  void PreemptMission(
      const std::shared_ptr<RequestT>& request,
      const std::shared_ptr<ResponseT>& response) {
    const auto profile = robot_tasks::LoadMissionProfile(request->mission_file);
    if (!profile.has_value()) {
      response->success = false;
      response->message = "failed to load mission profile: " + request->mission_file;
      response->queue_size = static_cast<int>(mission_queue_.size());
      return;
    }
    const auto duplicate_gate = robot_tasks::PlanMissionSubmitGate(
        robot_tasks::MissionSubmitGateRequest{
            profile->mission_id, "mission", "mission",
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
            profile->mission_id, "mission", "mission", false, preflight});
    if (!gate.accepted) {
      response->success = false;
      response->mission_id = gate.mission_id;
      response->message = gate.message;
      response->queue_size = static_cast<int>(mission_queue_.size());
      return;
    }

    const bool had_active_mission = RunnerBusy();
    const auto admission = AdmitQueuedMission(
        QueuedMission{*profile, request->mission_file, request->priority,
                      next_queue_sequence_++},
        true, true, "preempting active mission for " + profile->mission_id,
        had_active_mission ? "preempt mission queued and active cancel requested"
                           : "preempt mission queued to start",
        false);
    response->success = admission.success;
    response->mission_id = profile->mission_id;
    response->message = admission.message;
    response->queue_size = admission.queue_size;
  }

  template <typename RequestT, typename ResponseT>
  void EstimateMission(
      const std::shared_ptr<RequestT>& request,
      const std::shared_ptr<ResponseT>& response) {
    std::optional<robot_tasks::MissionProfile> profile;
    if (request->mission_file.empty()) {
      if (!profile_.has_value()) {
        LoadProfile(nullptr);
      }
      profile = profile_;
    } else {
      profile = robot_tasks::LoadMissionProfile(request->mission_file);
    }
    if (!profile.has_value()) {
      response->success = false;
      response->message = "failed to load mission profile for estimate";
      return;
    }
    FillEstimateResponse(
        *profile, request->nominal_speed_mps, request->battery_voltage, response.get());
  }

  template <typename ResponseT>
  void RejectSubmitOrder(
      const std::shared_ptr<ResponseT>& response,
      const std::string& message) const {
    response->accepted = false;
    response->reject_reason = message;
    response->message = message;
    response->queue_size = static_cast<int>(mission_queue_.size());
  }

  template <typename ResponseT>
  void FillSubmitOrderResponse(
      const std::shared_ptr<ResponseT>& response,
      const bool success, const std::string& message, const std::string& mission_id,
      const std::vector<std::string>& mission_ids, const int queue_size) const {
    response->accepted = success;
    response->reject_reason = success ? "" : message;
    response->message = message;
    response->mission_id =
        !mission_id.empty() ? mission_id : (mission_ids.empty() ? "" : mission_ids.front());
    response->mission_ids = mission_ids;
    if (response->mission_ids.empty() && !response->mission_id.empty()) {
      response->mission_ids.push_back(response->mission_id);
    }
    response->queue_size = queue_size;
  }
