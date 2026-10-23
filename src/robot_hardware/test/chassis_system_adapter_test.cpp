/**
 * @file chassis_system_adapter_test.cpp
 * @brief （源码）。
 * @details 所属包：`robot_hardware`；所在层次：源码。
 * @note 教学说明见 `docs/教学/` 对应章节。
 */

#include "robot_hardware/system/adapter.hpp"

#include <deque>
#include <limits>
#include <memory>
#include <optional>
#include <stdexcept>
#include <string>
#include <utility>

#include <gtest/gtest.h>

#include "robot_hardware/transport/mock_backend.hpp"

namespace robot_hardware {

using namespace robot_hardware::protocol;
using namespace robot_hardware::transport;
using namespace robot_hardware::kinematics;
using namespace robot_hardware::system;
namespace {

class ScriptedBackend final : public Backend {
 public:
  explicit ScriptedBackend(std::deque<std::string> reads) : reads_(std::move(reads)) {}

  std::string Name() const override { return "scripted"; }

  BackendKind Kind() const override { return BackendKind::kSerial; }

  bool Open(std::string*) override {
    open_ = true;
    return true;
  }

  void Close() override { open_ = false; }

  bool IsOpen() const override { return open_; }

  bool WriteCommand(const Command& command, std::string*) override {
    last_command_ = command;
    return open_;
  }

  std::optional<std::string> Read(std::string*) override {
    if (reads_.empty()) {
      return std::nullopt;
    }
    std::string next = reads_.front();
    reads_.pop_front();
    return next;
  }

  Command LastCommand() const { return last_command_; }

 private:
  bool open_ = false;
  std::deque<std::string> reads_;
  Command last_command_;
};

}  // namespace

TEST(AdapterTest, Constructor_NullBackend_ThrowsInvalidArgument) {
  EXPECT_THROW(Adapter(nullptr), std::invalid_argument);
}

TEST(AdapterTest, ValidateCalibration_DefaultReferenceConfiguration_Succeeds) {
  Config config;
  std::string error;

  EXPECT_TRUE(ValidateConfig(config, &error));
  EXPECT_TRUE(error.empty());
  EXPECT_DOUBLE_EQ(config.track_width_m, 0.43);
}

TEST(AdapterTest, Constructor_InvalidCalibration_ThrowsBeforeBackendUse) {
  Config config;
  config.track_width_m = 0.0;

  EXPECT_THROW(
      Adapter(std::make_unique<MockBackend>(), config), std::invalid_argument);
}

TEST(AdapterTest, ValidateCalibration_NonFiniteGeometry_Fails) {
  Config config;
  config.wheel_diameter_m = std::numeric_limits<double>::quiet_NaN();
  std::string error;

  EXPECT_FALSE(ValidateConfig(config, &error));
  EXPECT_NE(error.find("wheel_diameter_m"), std::string::npos);
}

TEST(AdapterTest, ValidateCalibration_NonIdentityEncoderCorrection_FailsClearly) {
  Config config;
  config.left_direction_sign = -1;
  std::string error;

  EXPECT_FALSE(ValidateConfig(config, &error));
  EXPECT_NE(error.find("raw wheel encoders"), std::string::npos);
}

TEST(AdapterTest, Write_ClosedBackend_ReturnsError) {
  Adapter adapter(std::make_unique<MockBackend>());

  std::string error;
  const bool ok = adapter.Write(Command{}, &error);

  EXPECT_FALSE(ok);
  EXPECT_EQ(error, "chassis backend is not open");
}

TEST(AdapterTest, Read_MockDiffDriveCommand_IntegratesPlanarState) {
  auto backend = std::make_unique<MockBackend>();
  const MockBackend* backend_ptr = backend.get();
  Adapter adapter(std::move(backend));
  ASSERT_TRUE(adapter.Open(nullptr));
  Command command;
  command.linear_x_mps = 0.5;
  command.linear_y_mps = 0.25;
  command.angular_z_radps = 0.2;
  ASSERT_TRUE(adapter.Write(command, nullptr));

  Snapshot state;
  ASSERT_TRUE(adapter.Read(2.0, &state, nullptr));

  EXPECT_TRUE(state.backend_open);
  EXPECT_TRUE(state.connected);
  EXPECT_EQ(state.backend_name, "mock");
  EXPECT_NEAR(state.x_m, 1.0, 1e-9);
  EXPECT_NEAR(state.y_m, 0.0, 1e-9);
  EXPECT_NEAR(state.yaw_rad, 0.4, 1e-9);
  EXPECT_NEAR(state.linear_x_mps, 0.5, 1e-9);
  EXPECT_NEAR(state.linear_y_mps, 0.0, 1e-9);
  EXPECT_NEAR(state.angular_z_radps, 0.2, 1e-9);
  EXPECT_NEAR(backend_ptr->LastCommand().linear_y_mps, 0.0, 1e-9);
  EXPECT_NE(state.wheel_speeds.rpm[0], state.wheel_speeds.rpm[1]);
}

TEST(AdapterTest, Read_MockDiffDrive_UsesConfiguredEffectiveTrackWidth) {
  Config config;
  config.track_width_m = 0.50;
  Adapter adapter(std::make_unique<MockBackend>(), config);
  ASSERT_TRUE(adapter.Open(nullptr));
  Command command;
  command.angular_z_radps = 1.0;
  ASSERT_TRUE(adapter.Write(command, nullptr));

  Snapshot state;
  ASSERT_TRUE(adapter.Read(0.1, &state, nullptr));
  const WheelSpeeds expected = EstimateWheelSpeeds(
      command, "diff_drive", config.wheel_diameter_m, config.wheel_base_m,
      config.track_width_m);

  EXPECT_NEAR(state.wheel_speeds.rpm[0], expected.rpm[0], 1e-9);
  EXPECT_NEAR(state.wheel_speeds.rpm[1], expected.rpm[1], 1e-9);
}

TEST(AdapterTest, Read_MockMecanumCommand_PreservesSidewaysVelocity) {
  auto backend = std::make_unique<MockBackend>();
  const MockBackend* backend_ptr = backend.get();
  Config config;
  config.kinematics_model = "mecanum";
  Adapter adapter(std::move(backend), config);
  ASSERT_TRUE(adapter.Open(nullptr));
  Command command;
  command.linear_x_mps = 0.5;
  command.linear_y_mps = 0.25;
  ASSERT_TRUE(adapter.Write(command, nullptr));

  Snapshot state;
  ASSERT_TRUE(adapter.Read(1.0, &state, nullptr));

  EXPECT_NEAR(state.x_m, 0.5, 1e-9);
  EXPECT_NEAR(state.y_m, 0.25, 1e-9);
  EXPECT_NEAR(state.linear_y_mps, 0.25, 1e-9);
  EXPECT_NEAR(backend_ptr->LastCommand().linear_y_mps, 0.25, 1e-9);
}

TEST(AdapterTest, Read_MockFourWheelSteerAliasDegradesToDiffDrive) {
  auto backend = std::make_unique<MockBackend>();
  const MockBackend* backend_ptr = backend.get();
  Config config;
  config.kinematics_model = "four_ws4wd";
  Adapter adapter(std::move(backend), config);
  ASSERT_TRUE(adapter.Open(nullptr));
  Command command;
  command.linear_x_mps = 0.5;
  command.linear_y_mps = 0.25;
  ASSERT_TRUE(adapter.Write(command, nullptr));

  Snapshot state;
  ASSERT_TRUE(adapter.Read(1.0, &state, nullptr));

  EXPECT_NEAR(state.x_m, 0.5, 1e-9);
  EXPECT_NEAR(state.y_m, 0.0, 1e-9);
  EXPECT_NEAR(state.linear_y_mps, 0.0, 1e-9);
  EXPECT_NEAR(backend_ptr->LastCommand().linear_y_mps, 0.0, 1e-9);
}

TEST(AdapterTest, Read_TextPackets_UpdatesMeasuredState) {
  auto backend = std::make_unique<ScriptedBackend>(
      std::deque<std::string>{"ODOM 1.25 0.10 0.02 0.30 -0.04 24.1\nSTATE simulated 23.8\n"});
  Adapter adapter(std::move(backend));
  ASSERT_TRUE(adapter.Open(nullptr));

  Snapshot state;
  ASSERT_TRUE(adapter.Read(0.02, &state, nullptr));

  EXPECT_TRUE(state.connected);
  EXPECT_NEAR(state.x_m, 1.25, 1e-9);
  EXPECT_NEAR(state.y_m, 0.10, 1e-9);
  EXPECT_NEAR(state.yaw_rad, 0.02, 1e-9);
  EXPECT_NEAR(state.linear_x_mps, 0.30, 1e-9);
  EXPECT_NEAR(state.angular_z_radps, -0.04, 1e-9);
  EXPECT_EQ(state.hardware_status, "simulated");
  EXPECT_NEAR(state.battery_voltage, 23.8, 1e-9);
  EXPECT_EQ(state.packet_sequence, 2U);
}

TEST(AdapterTest, SetKinematicsModel_DiffDriveToMecanum_PreservesSidewaysVelocity) {
  auto backend = std::make_unique<MockBackend>();
  const MockBackend* backend_ptr = backend.get();
  Adapter adapter(std::move(backend));
  ASSERT_TRUE(adapter.Open(nullptr));
  adapter.SetKinematicsModel("mecanum");
  Command command;
  command.linear_y_mps = 0.25;
  ASSERT_TRUE(adapter.Write(command, nullptr));

  Snapshot state;
  ASSERT_TRUE(adapter.Read(1.0, &state, nullptr));

  EXPECT_NEAR(state.y_m, 0.25, 1e-9);
  EXPECT_NEAR(backend_ptr->LastCommand().linear_y_mps, 0.25, 1e-9);
}

TEST(AdapterTest, ResetOdometry_IntegratedMockState_ClearsPoseOnly) {
  Adapter adapter(std::make_unique<MockBackend>());
  ASSERT_TRUE(adapter.Open(nullptr));
  Command command;
  command.linear_x_mps = 0.5;
  ASSERT_TRUE(adapter.Write(command, nullptr));
  Snapshot state;
  ASSERT_TRUE(adapter.Read(1.0, &state, nullptr));
  ASSERT_NEAR(state.x_m, 0.5, 1e-9);

  adapter.ResetOdometry();
  state = adapter.State();

  EXPECT_NEAR(state.x_m, 0.0, 1e-9);
  EXPECT_NEAR(state.y_m, 0.0, 1e-9);
  EXPECT_NEAR(state.yaw_rad, 0.0, 1e-9);
  EXPECT_NEAR(state.linear_x_mps, 0.5, 1e-9);
}

TEST(AdapterTest, Read_MalformedTextPacket_ReturnsObservableError) {
  auto backend = std::make_unique<ScriptedBackend>(std::deque<std::string>{"NOPE bad packet\n"});
  Adapter adapter(std::move(backend));
  ASSERT_TRUE(adapter.Open(nullptr));

  std::string error;
  Snapshot state;
  const bool ok = adapter.Read(0.02, &state, &error);

  EXPECT_FALSE(ok);
  EXPECT_EQ(error, "ignored malformed chassis packet line");
  EXPECT_FALSE(state.connected);
}

TEST(AdapterTest, Close_OpenBackend_DropsConnectionState) {
  Adapter adapter(std::make_unique<MockBackend>());
  ASSERT_TRUE(adapter.Open(nullptr));

  adapter.Close();

  EXPECT_FALSE(adapter.State().backend_open);
  EXPECT_FALSE(adapter.State().connected);
}

}  // namespace robot_hardware
