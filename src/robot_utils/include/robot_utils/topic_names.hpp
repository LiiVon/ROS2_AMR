#pragma once

namespace robot_utils::topics {

constexpr const char* kScan = "/scan";
constexpr const char* kImu = "/imu/data";
constexpr const char* kOdom = "/odom";
constexpr const char* kWheelOdom = "/wheel/odom";
constexpr const char* kFilteredOdom = "/odometry/filtered";
constexpr const char* kCmdVel = "/cmd_vel";
constexpr const char* kNav2CmdVel = "/cmd_vel/nav2";
constexpr const char* kTrackingCmdVel = "/cmd_vel/tracking";
constexpr const char* kTeleopCmdVel = "/cmd_vel/teleop";
constexpr const char* kReferencePath = "/tracking/reference_path";
constexpr const char* kTrackingError = "/tracking/error";

}  // namespace robot_utils::topics
