/**
 * @file chassis_packet_test.cpp
 * @brief （源码）。
 * @details 所属包：`robot_hardware`；所在层次：源码。
 * @note 教学说明见 `docs/教学/` 对应章节。
 */

#include "robot_hardware/protocol/packet.hpp"
#include "robot_hardware/kinematics/kinematics.hpp"

#include <gtest/gtest.h>

namespace robot_hardware {

using namespace robot_hardware::protocol;
using namespace robot_hardware::kinematics;

namespace {
constexpr double kPi = 3.14159265358979323846264338327950288;
}  // namespace

TEST(PacketTest, EncodeCommand_ValidCommand_FormatsLineProtocol) {
  Command command;
  command.linear_x_mps = 0.3;
  command.linear_y_mps = 0.1;
  command.angular_z_radps = -0.2;

  EXPECT_EQ(EncodeCommand(command), "CMD 0.3000 0.1000 -0.2000\n");
}

TEST(PacketTest, ParseCommandLine_ValidCommand_ReturnsCommand) {
  const auto command = DecodeCommand("CMD 0.125 -0.500");

  ASSERT_TRUE(command.has_value());
  EXPECT_DOUBLE_EQ(command->linear_x_mps, 0.125);
  EXPECT_DOUBLE_EQ(command->linear_y_mps, 0.0);
  EXPECT_DOUBLE_EQ(command->angular_z_radps, -0.5);
}

TEST(PacketTest, ParseCommandLine_ThreeAxisCommand_ReturnsHolonomicCommand) {
  const auto command = DecodeCommand("CMD 0.125 -0.250 -0.500");

  ASSERT_TRUE(command.has_value());
  EXPECT_DOUBLE_EQ(command->linear_x_mps, 0.125);
  EXPECT_DOUBLE_EQ(command->linear_y_mps, -0.250);
  EXPECT_DOUBLE_EQ(command->angular_z_radps, -0.5);
}

TEST(PacketTest, ParsePacketLine_ValidOdometry_ReturnsOdometryPacket) {
  const auto packet = DecodePacket("ODOM 1.25 0.10 0.02 0.30 -0.04 24.1");

  ASSERT_TRUE(packet.has_value());
  ASSERT_EQ(packet->kind, PacketKind::kOdometry);
  EXPECT_DOUBLE_EQ(packet->odometry.x_m, 1.25);
  EXPECT_DOUBLE_EQ(packet->odometry.y_m, 0.10);
  EXPECT_DOUBLE_EQ(packet->odometry.yaw_rad, 0.02);
  EXPECT_DOUBLE_EQ(packet->odometry.linear_x_mps, 0.30);
  EXPECT_DOUBLE_EQ(packet->odometry.angular_z_radps, -0.04);
  ASSERT_TRUE(packet->odometry.battery_voltage.has_value());
  EXPECT_DOUBLE_EQ(*packet->odometry.battery_voltage, 24.1);
}

TEST(PacketTest, ParsePacketLine_ValidState_ReturnsStatePacket) {
  const auto packet = DecodePacket("STATE running 23.8");

  ASSERT_TRUE(packet.has_value());
  ASSERT_EQ(packet->kind, PacketKind::kState);
  EXPECT_EQ(packet->state.status, "running");
  ASSERT_TRUE(packet->state.battery_voltage.has_value());
  EXPECT_DOUBLE_EQ(*packet->state.battery_voltage, 23.8);
}

TEST(PacketTest, ParsePacketLine_MalformedPacket_ReturnsNullopt) {
  EXPECT_FALSE(DecodePacket("ODOM 1.0 2.0 bad").has_value());
}

TEST(PacketTest, PacketStream_PartialAndStickyLines_DrainsCompletePackets) {
  PacketStream stream;
  stream.Append("ODOM 1 2 0.1 0.3 0.0 24");
  EXPECT_TRUE(stream.DrainPackets().empty());
  stream.Append("\nSTATE running 24.0\n");

  const auto packets = stream.DrainPackets();

  ASSERT_EQ(packets.size(), 2U);
  EXPECT_EQ(packets[0].kind, PacketKind::kOdometry);
  EXPECT_EQ(packets[1].kind, PacketKind::kState);
}

TEST(PacketTest, PacketStream_BufferOverflow_ClearsBufferAndReportsOverflow) {
  PacketStream stream(8);

  stream.Append("0123456789");

  EXPECT_TRUE(stream.Overflowed());
  EXPECT_TRUE(stream.DrainPackets().empty());
}

TEST(PacketTest, PacketStream_MalformedCompleteLine_CountsInvalidLine) {
  PacketStream stream;

  stream.Append("NOPE bad packet\n");
  const auto packets = stream.DrainPackets();

  EXPECT_TRUE(packets.empty());
  EXPECT_EQ(stream.InvalidLineCount(), 1U);
}

TEST(PacketTest, BinaryPacketStream_StickyFrames_DrainsPackets) {
  robot_hardware::OdometryPacket odom;
  odom.x_m = 1.2;
  odom.y_m = -0.3;
  odom.yaw_rad = 0.1;
  odom.linear_x_mps = 0.4;
  odom.angular_z_radps = -0.2;
  odom.battery_voltage = 24.5;
  std::vector<uint8_t> payload;
  auto append = [&payload](const int value) {
    payload.push_back(static_cast<uint8_t>((value >> 8) & 0xff));
    payload.push_back(static_cast<uint8_t>(value & 0xff));
  };
  append(1200);
  append(-300);
  append(100);
  append(400);
  append(-200);
  append(2450);

  const auto frame = EncodeBinaryFrame(0xa1, payload);
  BinaryPacketStream stream;
  stream.Append(std::string(reinterpret_cast<const char*>(frame.data()), frame.size()));

  const auto packets = stream.DrainPackets();
  ASSERT_EQ(packets.size(), 1U);
  EXPECT_EQ(packets[0].kind, PacketKind::kOdometry);
  EXPECT_NEAR(packets[0].odometry.x_m, 1.2, 1e-6);
  EXPECT_NEAR(packets[0].odometry.y_m, -0.3, 1e-6);
  ASSERT_TRUE(packets[0].odometry.battery_voltage.has_value());
  EXPECT_NEAR(*packets[0].odometry.battery_voltage, 24.5, 1e-6);
}

TEST(PacketTest, BinaryPacketStream_BadChecksum_CountsInvalidFrame) {
  auto frame = EncodeMickSpeedCommand(Command{});
  frame[frame.size() - 3] ^= 0x55;
  BinaryPacketStream stream;
  stream.Append(std::string(reinterpret_cast<const char*>(frame.data()), frame.size()));

  EXPECT_TRUE(stream.DrainPackets().empty());
  EXPECT_EQ(stream.InvalidFrameCount(), 1U);
}

TEST(ChassisKinematicsTest, IntegratePose_HolonomicCommand_UsesBodyYVelocity) {
  Command command;
  command.linear_x_mps = 1.0;
  command.linear_y_mps = 0.5;
  command.angular_z_radps = 0.1;

  const auto pose = IntegratePose(Pose2D{}, command, 2.0);

  EXPECT_NEAR(pose.x_m, 2.0, 1e-6);
  EXPECT_NEAR(pose.y_m, 1.0, 1e-6);
  EXPECT_NEAR(pose.yaw_rad, 0.2, 1e-6);
}

TEST(ChassisKinematicsTest, IntegratePose_NormalizesYaw) {
  Command command;
  command.angular_z_radps = 1.0;
  Pose2D start;
  start.yaw_rad = kPi - 0.05;

  const auto pose = IntegratePose(start, command, 0.2);

  EXPECT_GT(pose.yaw_rad, -kPi);
  EXPECT_LT(pose.yaw_rad, kPi);
  EXPECT_NEAR(pose.yaw_rad, -kPi + 0.15, 1e-9);
}

TEST(ChassisKinematicsTest, EstimateWheelSpeeds_MecanumCommand_ProducesFourWheelProfile) {
  Command command;
  command.linear_x_mps = 0.5;
  command.linear_y_mps = 0.2;
  command.angular_z_radps = 0.4;

  const auto speeds = EstimateWheelSpeeds(command, "mecanum", 0.15, 0.42, 0.36);

  EXPECT_NE(speeds.rpm[0], speeds.rpm[1]);
  EXPECT_NE(speeds.rpm[2], speeds.rpm[3]);
}

TEST(ChassisKinematicsTest, NormalizeModel_UnsupportedSteeringModesDegradeToDiffDrive) {
  EXPECT_EQ(NormalizeModel("ackermann"), "diff_drive");
  EXPECT_EQ(NormalizeModel("acker"), "diff_drive");
  EXPECT_EQ(NormalizeModel("four_ws4wd"), "diff_drive");
  EXPECT_EQ(NormalizeModel("4ws4wd"), "diff_drive");
  EXPECT_EQ(NormalizeModel("four_wheel_steer"), "diff_drive");
}

TEST(ChassisKinematicsTest, EstimateWheelSpeeds_FourWheelSteerAliasUsesDiffDriveProfile) {
  Command command;
  command.linear_x_mps = 0.5;
  command.linear_y_mps = 0.2;
  command.angular_z_radps = 0.4;

  const auto degraded = EstimateWheelSpeeds(command, "four_ws4wd", 0.15, 0.42, 0.36);
  const auto diff_drive = EstimateWheelSpeeds(command, "diff_drive", 0.15, 0.42, 0.36);

  EXPECT_EQ(degraded.rpm, diff_drive.rpm);
}

TEST(ChassisKinematicsTest, EstimateChassisCommand_DiffDriveRoundTrip) {
  Command command;
  command.linear_x_mps = 0.45;
  command.angular_z_radps = -0.3;

  const auto speeds = EstimateWheelSpeeds(command, "diff_drive", 0.15, 0.42, 0.36);
  const auto estimated = EstimateChassisCommand(speeds, "diff_drive", 0.15, 0.42, 0.36);

  EXPECT_NEAR(estimated.linear_x_mps, command.linear_x_mps, 1e-9);
  EXPECT_NEAR(estimated.angular_z_radps, command.angular_z_radps, 1e-9);
  EXPECT_NEAR(estimated.linear_y_mps, 0.0, 1e-9);
}

TEST(ChassisKinematicsTest, EstimateChassisCommand_MecanumRoundTrip) {
  Command command;
  command.linear_x_mps = 0.45;
  command.linear_y_mps = -0.2;
  command.angular_z_radps = 0.3;

  const auto speeds = EstimateWheelSpeeds(command, "mecanum", 0.15, 0.42, 0.36);
  const auto estimated = EstimateChassisCommand(speeds, "mecanum", 0.15, 0.42, 0.36);

  EXPECT_NEAR(estimated.linear_x_mps, command.linear_x_mps, 1e-9);
  EXPECT_NEAR(estimated.linear_y_mps, command.linear_y_mps, 1e-9);
  EXPECT_NEAR(estimated.angular_z_radps, command.angular_z_radps, 1e-9);
}

}  // namespace robot_hardware
