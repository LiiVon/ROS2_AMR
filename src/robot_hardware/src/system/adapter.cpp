#include "robot_hardware/system/adapter.hpp"

#include <algorithm>
#include <cmath>
#include <optional>
#include <stdexcept>
#include <utility>

namespace robot_hardware::system
{
    using protocol::Command;
    using protocol::Packet;
    using protocol::PacketKind;
    using protocol::PacketStream;
    using protocol::BinaryPacketStream;
    using transport::Backend;
    using transport::BackendKind;
    using kinematics::Pose2D;
    using kinematics::WheelSpeeds;
    using kinematics::IntegratePose;
    using kinematics::EstimateWheelSpeeds;
    using kinematics::NormalizeModel;

    // 配置校验
    bool ValidateConfig(const Config &config, std::string *error)
    {
        const auto fail = [error](const std::string &msg)
        {
            if (error)
            {
                *error = msg;
            }
            return false;
        };

        if (config.calibration_profile.empty())
        {
            return fail("Calibration profile name cannot be empty.");
        }

        for (const auto &value : {
                 std::pair{"wheel_diameter_m", config.wheel_diameter_m},
                 std::pair{"wheel_base_m", config.wheel_base_m},
                 std::pair{"track_width_m", config.track_width_m},
                 std::pair{"left_encoder_scale", config.left_encoder_scale},
                 std::pair{"right_encoder_scale", config.right_encoder_scale},
                 std::pair{"fallback_battery_voltage", config.fallback_battery_voltage}})
        {
            // 是否有限且大于0
            if (!std::isfinite(value.second) || value.second <= 0.0)
            {
                return fail(std::string(value.first) + " must be a positive finite number.");
            }
        }
        if ((config.left_direction_sign != -1 && config.left_direction_sign != 1) ||
            (config.right_direction_sign != -1 && config.right_direction_sign != 1))
        {
            return fail("wheel direction signs must be either -1 or 1");
        }
        if (config.left_encoder_scale != 1.0 || config.right_encoder_scale != 1.0 ||
            config.left_direction_sign != 1 || config.right_direction_sign != 1)
        {
            return fail(
                "non-identity encoder scale or direction is not supported because the current "
                "backends expose body-frame odometry rather than raw wheel encoders");
        }
        if (error != nullptr)
        {
            error->clear();
        }
        return true;
    }

    Adapter::Adapter(std::unique_ptr<Backend> backend, Config config)
        : backend_(std::move(backend)), config_(std::move(config))
    {
        if (!backend_)
        {
            throw std::invalid_argument("backend cannot be null");
        }
        std::string error;
        if (!ValidateConfig(config_, &error))
        {
            throw std::invalid_argument("Invalid Config: " + error);
        }
        config_.kinematics_model = NormalizeModel(config_.kinematics_model);
        state_.backend_name = backend_->Name();
        state_.battery_voltage = config_.fallback_battery_voltage;
    }

    std::string Adapter::BackendName() const
    {
        return backend_->Name();
    }

    bool Adapter::Open(std::string *error)
    {
        const bool opened = backend_->Open(error);
        RefreshBackendState();
        if (opened && backend_->Kind() == BackendKind::kMock)
        {
            state_.connected = true;
        }
        return opened;
    }

    void Adapter::Close()
    {
        backend_->Close();
        RefreshBackendState();
        state_.connected = false;
    }

    bool Adapter::IsOpen() const
    {
        return backend_->IsOpen();
    }

    void Adapter::SetKinematicsModel(const std::string &model)
    {
        config_.kinematics_model = NormalizeModel(model);
    }

    bool Adapter::Write(const Command &command, std::string *error)
    {
        RefreshBackendState();
        if (!state_.backend_open)
        {
            if (error)
            {
                *error = "chassis backend is not open";
            }
            return false;
        }

        const Command normalized = NormalizeCommand(command);
        if (!backend_->WriteCommand(normalized, error))
        {
            return false;
        }
        command_ = normalized;
        return true;
    }

    bool Adapter::Read(double period_seconds, Snapshot *state, std::string *error)
    {
        RefreshBackendState();
        if (!state_.backend_open)
        {
            if (state)
            {
                *state = state_;
            }
            if (error)
            {
                *error = "chassis backend is not open";
            }
            return false;
        }

        bool ok = true;

        if (backend_->Kind() == BackendKind::kMock)
        {
            IntegrateMockMotion(period_seconds);
            state_.connected = true;
        }
        else
        {
            ok = ReadBackend(error);
        }

        if (state)
        {
            *state = state_;
        }
        return ok && state_.connected;
    }

    const Snapshot &Adapter::State() const
    {
        return state_;
    }

    void Adapter::ResetOdometry()
    {
        state_.x_m = 0.0;
        state_.y_m = 0.0;
        state_.yaw_rad = 0.0;
    }

    Command Adapter::NormalizeCommand(const Command &command) const
    {
        Command normalized = command;
        if (config_.kinematics_model != "mecanum")
        {
            normalized.linear_y_mps = 0.0;
        }
        return normalized;
    }

    void Adapter::IntegrateMockMotion(const double period_seconds)
    {
        Pose2D pose{state_.x_m, state_.y_m, state_.yaw_rad};
        pose = IntegratePose(pose, command_, std::max(0.0, period_seconds));
        state_.x_m = pose.x_m;
        state_.y_m = pose.y_m;
        state_.yaw_rad = pose.yaw_rad;
        state_.linear_x_mps = command_.linear_x_mps;
        state_.linear_y_mps = command_.linear_y_mps;
        state_.angular_z_radps = command_.angular_z_radps;
        state_.wheel_speeds =
            EstimateWheelSpeeds(command_, config_.kinematics_model, config_.wheel_diameter_m,
                                config_.wheel_base_m, config_.track_width_m);
    }

    bool Adapter::ReadBackend(std::string *error)
    {
        if (error != nullptr)
        {
            error->clear();
        }
        const std::optional<std::string> bytes = backend_->Read(error);
        if (!bytes.has_value())
        {
            return error == nullptr || error->empty();
        }

        if (config_.protocol == "mick_binary")
        {
            binary_stream_.Append(*bytes);
            if (binary_stream_.Overflowed())
            {
                binary_stream_.ClearOverflow();
                if (error != nullptr)
                {
                    *error = "cleared oversized chassis binary receive buffer";
                }
                return false;
            }
            for (const auto &packet : binary_stream_.DrainPackets())
            {
                ApplyPacket(packet);
            }
            if (binary_stream_.InvalidFrameCount() > 0)
            {
                binary_stream_.ClearInvalidFrameCount();
                if (error != nullptr)
                {
                    *error = "ignored malformed Mick binary frame";
                }
                return false;
            }
            return true;
        }

        text_stream_.Append(*bytes);
        if (text_stream_.Overflowed())
        {
            text_stream_.ClearOverflow();
            if (error != nullptr)
            {
                *error = "cleared oversized chassis receive buffer";
            }
            return false;
        }
        for (const auto &packet : text_stream_.DrainPackets())
        {
            ApplyPacket(packet);
        }
        if (text_stream_.InvalidLineCount() > 0)
        {
            text_stream_.ClearInvalidLineCount();
            if (error != nullptr)
            {
                *error = "ignored malformed chassis packet line";
            }
            return false;
        }
        return true;
    }

    void Adapter::ApplyPacket(const Packet &packet)
    {
        if (packet.kind == PacketKind::kOdometry)
        {
            const auto &odom = packet.odometry;
            state_.x_m = odom.x_m;
            state_.y_m = odom.y_m;
            state_.yaw_rad = odom.yaw_rad;
            state_.linear_x_mps = odom.linear_x_mps;
            state_.linear_y_mps = 0.0;
            state_.angular_z_radps = odom.angular_z_radps;
            Command measured;
            measured.linear_x_mps = odom.linear_x_mps;
            measured.angular_z_radps = odom.angular_z_radps;
            state_.wheel_speeds =
                EstimateWheelSpeeds(measured, config_.kinematics_model, config_.wheel_diameter_m,
                                    config_.wheel_base_m, config_.track_width_m);
            if (odom.battery_voltage.has_value())
            {
                state_.battery_voltage = *odom.battery_voltage;
            }
            ++state_.packet_sequence;
            state_.connected = true;
            return;
        }

        if (!packet.state.status.empty())
        {
            state_.hardware_status = packet.state.status;
        }
        if (packet.state.battery_voltage.has_value())
        {
            state_.battery_voltage = *packet.state.battery_voltage;
        }
        ++state_.packet_sequence;
        state_.connected = true;
    }

    void Adapter::RefreshBackendState()
    {
        state_.backend_name = backend_->Name();
        state_.backend_open = backend_->IsOpen();
        if (!state_.backend_open)
        {
            state_.connected = false;
        }
    }

}