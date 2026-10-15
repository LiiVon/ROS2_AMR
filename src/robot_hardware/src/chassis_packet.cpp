#include "robot_hardware/chassis_packet.hpp"

#include <array>
#include <cmath>
#include <iomanip>
#include <sstream>
#include <algorithm>

namespace robot_hardware
{
    namespace
    {
        // 处理换行符
        std::string TrimCarriageReturn(std::string line)
        {
            if (!line.empty() && line.back() == '\r')
            {
                line.pop_back();
            }
            return line;
        }

        // 将 16 位整数转换成两个字节
        void AppendInt16(std::vector<uint8_t> *out, const int16_t value)
        {
            out->push_back(static_cast<uint8_t>((value >> 8) & 0xFF));
            out->push_back(static_cast<uint8_t>(value & 0xFF));
        }

        // 从指定位置读取两个字节，组合成一个有符号 16 位整数
        int16_t ReadInt16(const std::vector<uint8_t> &data, std::size_t offset)
        {
            return static_cast<int16_t>(
                static_cast<uint16_t>(data.at(offset)) << 8 |
                static_cast<uint16_t>(data.at(offset + 1)));
        }

        uint16_t ReadUint16(const std::vector<uint8_t> &data, std::size_t offset)
        {
            return static_cast<uint16_t>(
                static_cast<uint16_t>(data.at(offset)) << 8 |
                static_cast<uint16_t>(data.at(offset + 1)));
        }

        // 限制数值范围
        int16_t ClampToInt16(const double value)
        {
            const double clamped = std::clamp(value, -32768.0, 32767.0);
            return static_cast<int16_t>(std::lround(clamped));
        }
    }

    std::string EncodeCommand(const ChassisCommand &command)
    {
        std::ostringstream oss;
        oss.setf(std::ios::fixed);
        oss << std::setprecision(4)
            << "CMD "
            << command.linear_x_mps << " "
            << command.linear_y_mps << " "
            << command.angular_z_rads << "\n";
        return oss.str();
    }

    std::string EncodeOdometry(const ChassisOdometryPacket &odometry)
    {
        std::ostringstream oss;
        oss.setf(std::ios::fixed);
        oss << std::setprecision(4)
            << "ODOM "
            << odometry.x_m << " "
            << odometry.y_m << " "
            << odometry.yaw_rad << " "
            << odometry.linear_x_maps << " "
            << odometry.linear_y_maps << " "
            << odometry.angular_z_rads;
        if (odometry.battery_voltage.has_value())
        {
            oss << " " << odometry.battery_voltage.value();
        }
        oss << "\n";
        return oss.str();
    }

    std::string EncodeState(const ChassisStatePacket &state)
    {
        std::ostringstream oss;
        oss.setf(std::ios::fixed);
        oss << "STATE "
            << state.status;
        if (state.battery_voltage.has_value())
        {
            oss << " " << state.battery_voltage.value();
        }
        oss << "\n";
        return oss.str();
    }

    std::optional<ChassisCommand> DecodeCommand(const std::string &text)
    {
        const std::string line = TrimCarriageReturn(text);
        std::istringstream iss(line);
        std::string tag;
        ChassisCommand command;

        // 检查命令标识和第一个速度参数
        if (!(iss >> tag >> command.linear_x_maps) || tag != "CMD")
        {
            return std::nullopt;
        }

        // 读取第二个参数
        double second = 0.0;
        if (!(iss >> second))
        {
            return std::nullopt;
        }

        // 尝试读取第三个参数
        double third = 0.0;
        if (iss >> third)
        {
            command.linear_y_maps = second;
            command.angular_z_rads = third;
        }
        else
        {
            // 如果不是正常到达字符串末尾，而是解析失败，则拒绝
            if (!iss.eof())
            {
                return std::nullopt;
            }
        }

        // 检查是否还有多余字段
        std::string trailing;
        if (iss >> trailing)
        {
            return std::nullopt;
        }
        return command;
    }

    std::optional<ChassisPacket> DecodePacket(const std::string &text)
    {
        const std::string line = TrimCarriageReturn(text);
        std::istringstream iss(line);
        std::string tag;
        iss >> tag;

        if (tag == "ODOM")
        {
            ChassisOdometryPacket odometry;
            if (!(iss >> odometry.x_m >> odometry.y_m >> odometry.yaw_rad >> odometry.linear_x_maps >> odometry.linear_y_maps >> odometry.angular_z_rads))
            {
                return std::nullopt;
            }

            double battery = 0.0;
            if (iss >> battery)
            {
                odometry.battery_voltage = battery;
            }

            std::string trailing;
            if (iss >> trailing)
            {
                return std::nullopt;
            }

            ChassisPacket packet;
            packet.kind = ChassisPacketKind::kOdometry;
            packet.odometry = odometry;
            return packet;
        }

        if (tag == "STATE")
        {
            ChassisStatePacket state;
            if (!(iss >> state.status))
            {
                return std::nullopt;
            }

            double battery = 0.0;
            if (iss >> battery)
            {
                state.battery_voltage = battery;
            }

            std::string trailing;
            if (iss >> trailing)
            {
                return std::nullopt;
            }

            ChassisPacket packet;
            packet.kind = ChassisPacketKind::kState;
            packet.state = state;
            return packet;
        }
        return std::nullopt;
    }

    std::vector<uint8_t> EncodeMickBinaryFrame(const uint8_t type, const std::vector<uint8_t> &data)
    {
        std::vector<uint8_t> frame;
        frame.reserve(data.size() + 7);
        frame.push_back(0xae);
        frame.push_back(0xea);
        frame.push_back(static_cast<uint8_t>(data.size() + 2U));
        frame.push_back(type);
        frame.insert(frame.end(), data.begin(), data.end());
        uint8_t checksum = 0;
        for (std::size_t i = 2; i < frame.size(); ++i)
        {
            checksum = static_cast<uint8_t>(checksum + frame[i]);
        }
        frame.push_back(checksum);
        frame.push_back(0xef);
        frame.push_back(0xfe);
        return frame;
    }

    std::vector<uint8_t> EncodeMickSpeedCommand(const ChassisCommand &command)
    {
        std::vector<uint8_t> data;
        data.reserve(6);
        AppendInt16(&data, ClampToInt16(command.linear_x_maps * 1000.0));  // 转换为毫米/秒
        AppendInt16(&data, ClampToInt16(command.linear_y_maps * 1000.0));  // 转换为毫米/秒
        AppendInt16(&data, ClampToInt16(command.angular_z_rads * 1000.0)); // 转换为毫米/秒
        return EncodeMickBinaryFrame(0xf3, data);
    }

    std::vector<uint8_t> EncodeMickClearOdometryCommand()
    {
        return EncodeMickBinaryFrame(0xe1, {0x01, 0x00, 0x00, 0x00});
    }

    std::optional<ChassisPacket> DecodeMickBinaryPacket(const MickBinaryFrame &frame)
    {
        if (frame.type == 0xa1 && frame.data.size() >= 12U)
        {
            ChassisOdometryPacket odometry;
            odometry.x_m = ReadInt16(frame.data, 0) / 1000.0;
            odometry.y_m = ReadInt16(frame.data, 2) / 1000.0;
            odometry.yaw_rad = ReadInt16(frame.data, 4) / 1000.0;
            odometry.linear_x_maps = ReadInt16(frame.data, 6) / 1000.0;
            odometry.linear_y_maps = ReadInt16(frame.data, 6) / 1000.0;
            odometry.angular_z_rads = ReadInt16(frame.data, 8) / 1000.0;
            odometry.battery_voltage = ReadUint16(frame.data, 10) / 100.0;
            ChassisPacket packet;
            packet.kind = ChassisPacketKind::kOdometry;
            packet.odometry = odometry;
            return packet;
        }

        if (frame.type == 0xa2)
        {
            ChassisStatePacket state;
            state.status = frame.data.empty() || frame.data[0] == 0 ? "running" : "fault";
            if (frame.data.size() >= 3U)
            {
                state.battery_voltage = ReadUint16(frame.data, 1) / 100.0;
            }
            ChassisPacket packet;
            packet.kind = ChassisPacketKind::kState;
            packet.state = state;
            return packet;
        }

        return std::nullopt;
    }

    // ChassisPacketStream 类的实现
    ChassisPacketStream::ChassisPacketStream(std::size_t max_buffer_size)
        : max_buffer_size_(max_buffer_size)
    {
    }

    void ChassisPacketStream::Append(const char *data, const std::size_t size)
    {
        buffer_.append(data, size);
        if (buffer_.size() > max_buffer_size_)
        {
            buffer_.clear();
            overflowed_ = true;
        }
    }

    void ChassisPacketStream::Append(const std::string &data)
    {
        Append(data.data(), data.size());
    }

    std::vector<ChassisPacket> ChassisPacketStream::DrainPackets()
    {
        std::vector<ChassisPacket> packets;
        std::size_t newLine = std::string::npos;
        while ((newLine = buffer_.find('\n')) != std::string::npos)
        {
            const std::string line = buffer_.substr(0, newLine);
            buffer_.erase(0, newLine + 1);
            if (auto packet = DecodePacket(line))
            {
                packets.push_back(*packet);
            }
            else if (!line.empty())
            {
                ++invalid_line_count_;
            }
        }
        return packets;
    }

    bool ChassisPacketStream::Overflowed() const { return overflowed_; }

    std::size_t ChassisPacketStream::InvalidLineCount() const { return invalid_line_count_; }

    void ChassisPacketStream::ClearOverflow() { overflowed_ = false; }

    void ChassisPacketStream::ClearInvalidLineCount() { invalid_line_count_ = 0; }

    void ChassisPacketStream::Clear()
    {
        buffer_.clear();
        overflowed_ = false;
        invalid_line_count_ = 0;
    }

    // MickBinaryPacketStream 类的实现
    MickBinaryPacketStream::MickBinaryPacketStream(const std::size_t max_buffer_size)
        : max_buffer_size_(max_buffer_size) {}

    void MickBinaryPacketStream::Append(const uint8_t *data, const std::size_t size)
    {
        buffer_.insert(buffer_.end(), data, data + size);
        if (buffer_.size() > max_buffer_size_)
        {
            buffer_.clear();
            overflowed_ = true;
        }
    }

    void MickBinaryPacketStream::Append(const std::vector<uint8_t> &data)
    {
        Append(data.data(), data.size());
    }

    std::vector<ChassisPacket> MickBinaryPacketStream::DrainPackets()
    {
        std::vector<ChassisPacket> packets;
        while (buffer_.size() >= 7U)
        {
            const std::array<uint8_t, 2> magic{0xae, 0xea};
            const auto header = std::search(buffer_.begin(), buffer_.end(), magic.begin(), magic.end());
            if (header == buffer_.end())
            {
                buffer_.clear();
                return packets;
            }
            buffer_.erase(buffer_.begin(), header);
            if (buffer_.size() < 7U)
            {
                return packets;
            }
            const std::size_t length = buffer_[2];
            const std::size_t total_length = 2U + 1U + length + 2U;
            if (length < 2U)
            {
                buffer_.erase(buffer_.begin());
                ++invalid_frame_count_;
                continue;
            }
            if (buffer_.size() < total_length)
            {
                return packets;
            }
            if (buffer_[total_length - 2U] != 0xef || buffer_[total_length - 1U] != 0xfe)
            {
                buffer_.erase(buffer_.begin());
                ++invalid_frame_count_;
                continue;
            }
            uint8_t checksum = 0;
            for (std::size_t i = 2; i < total_length - 3U; ++i)
            {
                checksum = static_cast<uint8_t>(checksum + buffer_[i]);
            }
            if (checksum != buffer_[total_length - 3U])
            {
                buffer_.erase(buffer_.begin(), buffer_.begin() + total_length);
                ++invalid_frame_count_;
                continue;
            }
            MickBinaryFrame frame;
            frame.type = buffer_[3];
            frame.data.assign(buffer_.begin() + 4, buffer_.begin() + total_length - 3U);
            if (auto packet = DecodeMickBinaryPacket(frame))
            {
                packets.push_back(*packet);
            }
            buffer_.erase(buffer_.begin(), buffer_.begin() + total_length);
        }
        return packets;
    }

    bool MickBinaryPacketStream::Overflowed() const { return overflowed_; }

    std::size_t MickBinaryPacketStream::InvalidFrameCount() const { return invalid_frame_count_; }

    void MickBinaryPacketStream::ClearOverflow() { overflowed_ = false; }

    void MickBinaryPacketStream::ClearInvalidFrameCount() { invalid_frame_count_ = 0; }

    void MickBinaryPacketStream::Clear()
    {
        buffer_.clear();
        overflowed_ = false;
        invalid_frame_count_ = 0;
    }
}