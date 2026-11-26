/**
 * @file topic_names.hpp
 * @brief 全项目话题名常量（常量配置层）。
 * @details 所属包：`robot_utils`；所在层次：常量配置层。
 *
 * 这里是**话题名的唯一来源**：节点创建发布者/订阅者时应引用本文件的常量，
 * 不再手写 `"/odom"` 这类字面量（由代码评审与规范检查把关）。
 *
 * 命名规则：`/<域>/<名称>`，不带版本前缀与序号；常量名用 `k` + PascalCase。
 * 服务名（create_service / create_client）不在本文件内，仍由各域包自行定义。
 */

#pragma once

namespace robot_utils::topics
{

    // ---------------- 底盘 ----------------
    constexpr const char *kChassisState = "/chassis/state";        // 底盘状态：连接/故障码/电压
    constexpr const char *kChassisOdomPath = "/chassis/odom_path"; // 里程计轨迹（Path），odom_path_recorder_node 发布

    // ---------------- 传感器 ----------------
    constexpr const char *kScan = "/scan";                                 // 激光扫描（已过滤），laser_filter_node 发布
    constexpr const char *kSimScan = "/sim/scan";                          // 仿真原始扫描
    constexpr const char *kImu = "/imu/data";                              // IMU 数据，供 EKF 融合
    constexpr const char *kSimImu = "/sim/imu";                            // 仿真 IMU 原始数据
    constexpr const char *kImuRawBytes = "/imu/raw_bytes";                 // 100D4 原始字节流（串口桥 → 解析节点）
    constexpr const char *kImuCalibrationState = "/imu/calibration_state"; // 磁力校准状态文本

    // ---------------- 里程计 ----------------
    constexpr const char *kOdom = "/odom";                              // 里程计位姿与速度（driver 或 EKF 发布，二者互斥）
    constexpr const char *kWheelOdom = "/wheel/odom";                   // 纯轮速里程计，未经 EKF 融合
    constexpr const char *kFilteredOdom = "/odometry/filtered";         // EKF（robot_localization）融合后的里程计
    constexpr const char *kGazeboOdom = "/model/mobile_robot/odometry"; // Gazebo 模型里程计（仿真专用）

    // ---------------- 速度指令链路 ----------------
    constexpr const char *kCmdVel = "/cmd_vel";                     // 唯一最终速度指令，仅安全闸门发布
    constexpr const char *kCmdVelMuxed = "/cmd_vel/muxed";          // 仲裁输出，mux → 安全闸门
    constexpr const char *kNav2CmdVel = "/cmd_vel/nav2";            // 导航源（优先级 40）
    constexpr const char *kTrackingCmdVel = "/cmd_vel/tracking";    // 路径跟踪源（优先级 60）
    constexpr const char *kTeleopCmdVel = "/cmd_vel/teleop";        // 遥控源（优先级 100）
    constexpr const char *kVirtualRcCmdVel = "/cmd_vel/virtual_rc"; // 虚拟遥控输入（键盘/仿真运动）
    constexpr const char *kRobot2CmdVel = "/robot_2/cmd_vel";       // 仿真第二台车速度指令

    // ---------------- 遥控/接管 ----------------
    constexpr const char *kTeleopJoy = "/teleop/joy";                    // 虚拟摇杆轴值
    constexpr const char *kTeleopManualState = "/teleop/manual_state";   // 人工接管标志
    constexpr const char *kTeleopTakeover = "/teleop/takeover";          // 接管状态副本
    constexpr const char *kTeleopTakeoverLock = "/teleop/takeover_lock"; // twist_mux 锁通道
    constexpr const char *kTeleopActiveSource = "/teleop/active_source"; // 当前生效指令源描述

    // ---------------- 系统与安全 ----------------
    constexpr const char *kSystemHealth = "/system/health";                // 单一健康结论
    constexpr const char *kSystemFaultState = "/system/fault_state";       // 故障监督器状态机输出
    constexpr const char *kSystemEmergencyStop = "/system/emergency_stop"; // 急停状态

    // ---------------- 导航 ----------------
    constexpr const char *kNavigationTaskState = "/navigation/task_state";                   // 导航任务状态
    constexpr const char *kNavigationTaskStatus = "/navigation/task_status";                 // 任务进度（当前/总路点）
    constexpr const char *kNavigationCurrentGoal = "/navigation/current_goal";               // 当前目标点（transient_local）
    constexpr const char *kNavigationCurrentPath = "/navigation/current_path";               // 当前参考路径（transient_local）
    constexpr const char *kNavigationSafetyState = "/navigation/safety_state";               // 运行期安全状态
    constexpr const char *kNavigationLocalizationHealth = "/navigation/localization_health"; // 定位健康度
    constexpr const char *kNavigationMapState = "/navigation/map_state";                     // 当前激活地图
    constexpr const char *kNavigationMapCatalog = "/navigation/map_catalog";                 // 可用地图清单
    constexpr const char *kNavigationMapZones = "/navigation/map_zones";                     // 语义区域（限速/禁行）

    // ---------------- 路径跟踪 ----------------
    constexpr const char *kReferencePath = "/tracking/reference_path"; // 参考路径，path_publisher → 控制器
    constexpr const char *kTrackingError = "/tracking/error";          // 跟踪误差，控制器 → experiment_runner

    // ---------------- 业务与仿真 ----------------
    constexpr const char *kFacilityPayloadState = "/facility/payload_state"; // 载具状态
    constexpr const char *kExperimentStatistics = "/experiment/statistics";  // 实验结果统计
    constexpr const char *kSimMarkers = "/sim/markers";                      // 仿真可视化标记
    constexpr const char *kSimTimeline = "/sim/timeline";                    // 仿真演示时间线文本
    constexpr const char *kRobot2Odom = "/robot_2/odom";                     // 仿真第二台车里程计

} // namespace robot_utils::topics
