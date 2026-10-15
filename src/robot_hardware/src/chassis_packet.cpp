#include "robot_hardware/chassis_packet.hpp"

#include <algorithm>
#include <array>
#include <cmath>
#include <cstdint>
#include <iomanip>
#include <limits>
#include <sstream>
#include <stdexcept>

namespace robot_hardware
{
    namespace
    {
        // ============================================================
        // 辅助函数：仅供当前 cpp 文件内部使用
        // ============================================================

        // 移除文本行末尾的 '\r'，兼容 Windows 的 "\r\n" 换行
        std::string TrimCarriageReturn(std::string line)
        {
            if (!line.empty() && line.back() == '\r')
            {
                line.pop_back();
            }

            return line;
        }

        // 将有符号 16 位整数按大端序写入两个字节
        void AppendInt16(std::vector<uint8_t>& out, int16_t value)
        {
            const uint16_t bits = static_cast<uint16_t>(value);

            out.push_back(
                static_cast<uint8_t>((bits >> 8) & 0xFFU));

            out.push_back(
                static_cast<uint8_t>(bits & 0xFFU));
        }

        // 从指定偏移读取两个字节，按大端序解释为无符号 16 位整数
        uint16_t ReadUInt16(
            const std::vector<uint8_t>& data,
            std::size_t offset)
        {
            return static_cast<uint16_t>(
                (static_cast<uint16_t>(data.at(offset)) << 8) |
                static_cast<uint16_t>(data.at(offset + 1U)));
        }

        // 从指定偏移读取两个字节，按大端序解释为有符号 16 位整数
        int16_t ReadInt16(
            const std::vector<uint8_t>& data,
            std::size_t offset)
        {
            const uint16_t bits = ReadUInt16(data, offset);

            // 先转换为可容纳整个范围的有符号整数，
            // 再还原 16 位补码表示的负数。
            if (bits <= static_cast<uint16_t>(
                            std::numeric_limits<int16_t>::max()))
            {
                return static_cast<int16_t>(bits);
            }

            const int32_t signed_value =
                static_cast<int32_t>(bits) - 0x10000;

            return static_cast<int16_t>(signed_value);
        }

        // 将有限浮点数四舍五入，并限制在 int16_t 范围内
        int16_t ClampToInt16(double value)
        {
            if (!std::isfinite(value))
            {
                throw std::invalid_argument(
                    "Cannot encode a non-finite value as int16_t");
            }

            const double clamped =
                std::clamp(value, -32768.0, 32767.0);

            return static_cast<int16_t>(std::lround(clamped));
        }

        // 读取可选的有限浮点数。
        // 字段不存在时返回 true，且不修改 optional。
        // 字段存在但格式错误、不是有限数或后面还有多余字段时返回 false。
        bool ReadOptionalFiniteDouble(
            std::istringstream& iss,
            std::optional<double>& value)
        {
            iss >> std::ws;

            // 没有剩余内容：可选字段确实不存在
            if (iss.eof())
            {
                return true;
            }

            double parsed = 0.0;

            if (!(iss >> parsed) || !std::isfinite(parsed))
            {
                return false;
            }

            // 一个可选字段读取完毕后，不应再出现其他字段
            iss >> std::ws;

            if (!iss.eof())
            {
                return false;
            }

            value = parsed;
            return true;
        }

        // 检查多个速度、位置和角度字段是否全部为有限数
        bool IsFiniteOdometry(const ChassisOdometryPacket& odometry)
        {
            return std::isfinite(odometry.x_m) &&
                   std::isfinite(odometry.y_m) &&
                   std::isfinite(odometry.yaw_rad) &&
                   std::isfinite(odometry.linear_x_mps) &&
                   std::isfinite(odometry.linear_y_mps) &&
                   std::isfinite(odometry.angular_z_rads);
        }
    }

    // ================================================================
    // 一、文本协议编码
    // ================================================================

    // 将底盘速度指令编码为：
    // CMD linear_x_mps linear_y_mps angular_z_rads\n
    std::string EncodeCommand(const ChassisCommand& command)
    {
        std::ostringstream oss;

        oss << std::fixed << std::setprecision(4)
            << "CMD "
            << command.linear_x_mps << ' '
            << command.linear_y_mps << ' '
            << command.angular_z_rads << '\n';

        return oss.str();
    }

    // 将里程计数据编码为文本
    // ODOM x y yaw vx vy wz [battery]\n
    std::string EncodeOdometry(
        const ChassisOdometryPacket& odometry)
    {
        std::ostringstream oss;

        oss << std::fixed << std::setprecision(4)
            << "ODOM "
            << odometry.x_m << ' '
            << odometry.y_m << ' '
            << odometry.yaw_rad << ' '
            << odometry.linear_x_mps << ' '
            << odometry.linear_y_mps << ' '
            << odometry.angular_z_rads;

        if (odometry.battery_voltage.has_value())
        {
            oss << ' ' << *odometry.battery_voltage;
        }

        oss << '\n';

        return oss.str();
    }

    // 将底盘状态编码为文本
    // STATE status [battery]\n
    std::string EncodeState(const ChassisStatePacket& state)
    {
        std::ostringstream oss;

        oss << std::fixed << std::setprecision(4)
            << "STATE "
            << state.status;

        if (state.battery_voltage.has_value())
        {
            oss << ' ' << *state.battery_voltage;
        }

        oss << '\n';

        return oss.str();
    }

    // ================================================================
    // 二、文本协议解码
    // ================================================================

    // 解码速度指令。
    // 当前协议要求 CMD 后必须有三个速度参数。
    std::optional<ChassisCommand> DecodeCommand(
        const std::string& text)
    {
        const std::string line = TrimCarriageReturn(text);
        std::istringstream iss(line);

        std::string tag;
        std::string trailing;
        ChassisCommand command;

        if (!(iss >> tag
                  >> command.linear_x_mps
                  >> command.linear_y_mps
                  >> command.angular_z_rads) ||
            tag != "CMD")
        {
            return std::nullopt;
        }

        if (!std::isfinite(command.linear_x_mps) ||
            !std::isfinite(command.linear_y_mps) ||
            !std::isfinite(command.angular_z_rads))
        {
            return std::nullopt;
        }

        // 拒绝多余字段
        if (iss >> trailing)
        {
            return std::nullopt;
        }

        return command;
    }

    // 解码统一的数据包。
    // 根据首字段判断是 ODOM 还是 STATE。
    std::optional<ChassisPacket> DecodePacket(
        const std::string& text)
    {
        const std::string line = TrimCarriageReturn(text);
        std::istringstream iss(line);

        std::string tag;

        if (!(iss >> tag))
        {
            return std::nullopt;
        }

        // ------------------------------------------------------------
        // ODOM：里程计数据
        // ------------------------------------------------------------
        if (tag == "ODOM")
        {
            ChassisOdometryPacket odometry;

            if (!(iss >> odometry.x_m
                      >> odometry.y_m
                      >> odometry.yaw_rad
                      >> odometry.linear_x_mps
                      >> odometry.linear_y_mps
                      >> odometry.angular_z_rads))
            {
                return std::nullopt;
            }

            if (!IsFiniteOdometry(odometry))
            {
                return std::nullopt;
            }

            if (!ReadOptionalFiniteDouble(
                    iss, odometry.battery_voltage))
            {
                return std::nullopt;
            }

            ChassisPacket packet;
            packet.kind = ChassisPacketKind::kOdometry;
            packet.odometry = odometry;

            return packet;
        }

        // ------------------------------------------------------------
        // STATE：底盘状态数据
        // ------------------------------------------------------------
        if (tag == "STATE")
        {
            ChassisStatePacket state;

            if (!(iss >> state.status))
            {
                return std::nullopt;
            }

            if (!ReadOptionalFiniteDouble(
                    iss, state.battery_voltage))
            {
                return std::nullopt;
            }

            ChassisPacket packet;
            packet.kind = ChassisPacketKind::kState;
            packet.state = state;

            return packet;
        }

        // 不认识的消息类型
        return std::nullopt;
    }

    // ================================================================
    // 三、Mick 二进制协议编码
    // ================================================================

    // 组装完整二进制帧。
    //
    // 帧格式：
    // AE EA | LENGTH | TYPE | DATA | CHECKSUM | EF FE
    //
    // LENGTH = TYPE 的 1 字节 + CHECKSUM 的 1 字节 + DATA 长度
    std::vector<uint8_t> EncodeMickBinaryFrame(
        const uint8_t type,
        const std::vector<uint8_t>& data)
    {
        // LENGTH 只有 1 字节，最大为 255。
        // LENGTH = data.size() + 2，因此载荷最多 253 字节。
        if (data.size() > 253U)
        {
            throw std::length_error(
                "Mick binary payload exceeds 253 bytes");
        }

        std::vector<uint8_t> frame;
        frame.reserve(data.size() + 7U);

        // 帧头
        frame.push_back(0xAE);
        frame.push_back(0xEA);

        // 长度字段
        frame.push_back(
            static_cast<uint8_t>(data.size() + 2U));

        // 数据类型
        frame.push_back(type);

        // 数据载荷
        frame.insert(frame.end(), data.begin(), data.end());

        // 校验和：从 LENGTH 开始，累加至载荷最后一个字节
        uint8_t checksum = 0;

        for (std::size_t i = 2; i < frame.size(); ++i)
        {
            checksum = static_cast<uint8_t>(
                checksum + frame[i]);
        }

        frame.push_back(checksum);

        // 帧尾
        frame.push_back(0xEF);
        frame.push_back(0xFE);

        return frame;
    }

    // 将速度命令编码为 Mick 二进制指令。
    //
    // 每个速度字段使用 2 字节有符号整数。
    // 两个线速度按 mm/s 编码，角速度按 mrad/s 编码。
    std::vector<uint8_t> EncodeMickSpeedCommand(
        const ChassisCommand& command)
    {
        std::vector<uint8_t> data;
        data.reserve(6);

        AppendInt16(
            data,
            ClampToInt16(command.linear_x_mps * 1000.0));

        AppendInt16(
            data,
            ClampToInt16(command.linear_y_mps * 1000.0));

        AppendInt16(
            data,
            ClampToInt16(command.angular_z_rads * 1000.0));

        // 0xF3：当前实现使用的速度命令类型
        return EncodeMickBinaryFrame(0xF3, data);
    }

    // 生成清零里程计的二进制命令
    std::vector<uint8_t> EncodeMickClearOdometryCommand()
    {
        return EncodeMickBinaryFrame(
            0xE1, {0x01, 0x00, 0x00, 0x00});
    }

    // ================================================================
    // 四、Mick 二进制协议解码
    // ================================================================

    // 将已经提取出 type 和 data 的二进制帧解码为 ChassisPacket。
    // 本函数不负责帧头、帧尾和校验和检查；
    // 这些工作由 MickBinaryPacketStream 完成。
    std::optional<ChassisPacket> DecodeMickBinaryPacket(
        const MickBinaryFrame& frame)
    {
        // ------------------------------------------------------------
        // 0xA1：里程计数据
        //
        // 暂按原实现的 12 字节布局：
        // 0..1   x
        // 2..3   y
        // 4..5   yaw
        // 6..7   linear_x
        // 8..9   angular_z
        // 10..11 battery_voltage
        //
        // 当前布局没有独立的 linear_y 字段，因此设为 0。
        // 必须以实际 Mick 协议文档核实这些字段。
        // ------------------------------------------------------------
        if (frame.type == 0xA1)
        {
            if (frame.data.size() != 12U)
            {
                return std::nullopt;
            }

            ChassisOdometryPacket odometry;

            odometry.x_m =
                ReadInt16(frame.data, 0) / 1000.0;

            odometry.y_m =
                ReadInt16(frame.data, 2) / 1000.0;

            odometry.yaw_rad =
                ReadInt16(frame.data, 4) / 1000.0;

            odometry.linear_x_mps =
                ReadInt16(frame.data, 6) / 1000.0;

            // 原代码重复读取偏移 6，会错误地让两个线速度相同。
            // 当前假定的 12 字节协议没有独立的 y 方向速度字段。
            odometry.linear_y_mps = 0.0;

            odometry.angular_z_rads =
                ReadInt16(frame.data, 8) / 1000.0;

            odometry.battery_voltage =
                ReadUInt16(frame.data, 10) / 100.0;

            ChassisPacket packet;
            packet.kind = ChassisPacketKind::kOdometry;
            packet.odometry = odometry;

            return packet;
        }

        // ------------------------------------------------------------
        // 0xA2：底盘状态
        //
        // 当前假定：
        // 1 字节：状态码
        // 可选 2 字节：电池电压
        // ------------------------------------------------------------
        if (frame.type == 0xA2)
        {
            // 合法载荷长度为 1 或 3。
            // 0 字节代表缺失状态码，2 字节代表不完整的电压字段。
            if (frame.data.size() != 1U &&
                frame.data.size() != 3U)
            {
                return std::nullopt;
            }

            ChassisStatePacket state;

            state.status =
                frame.data[0] == 0 ? "running" : "fault";

            if (frame.data.size() == 3U)
            {
                state.battery_voltage =
                    ReadUInt16(frame.data, 1) / 100.0;
            }

            ChassisPacket packet;
            packet.kind = ChassisPacketKind::kState;
            packet.state = state;

            return packet;
        }

        // 未支持的二进制帧类型
        return std::nullopt;
    }

    // ================================================================
    // 五、文本数据流解析器 ChassisPacketStream
    // ================================================================

    ChassisPacketStream::ChassisPacketStream(
        std::size_t max_buffer_size)
        : max_buffer_size_(max_buffer_size)
    {
    }

    // 追加一段文本数据。
    // 先检查容量，避免超大输入先触发大规模内存分配。
    void ChassisPacketStream::Append(
        const char* data,
        std::size_t size)
    {
        if (size == 0U)
        {
            return;
        }

        // 调用约定：size 非零时，data 必须指向有效内存。
        if (size > max_buffer_size_ ||
            buffer_.size() > max_buffer_size_ - size)
        {
            buffer_.clear();
            overflowed_ = true;
            return;
        }

        buffer_.append(data, size);
    }

    // 方便调用者直接追加 std::string
    void ChassisPacketStream::Append(
        const std::string& data)
    {
        Append(data.data(), data.size());
    }

    // 从缓冲区中提取所有完整文本行并解析。
    std::vector<ChassisPacket>
    ChassisPacketStream::DrainPackets()
    {
        std::vector<ChassisPacket> packets;

        std::size_t newline = std::string::npos;

        while ((newline = buffer_.find('\n')) !=
               std::string::npos)
        {
            // 取出换行符之前的一行
            const std::string line =
                buffer_.substr(0, newline);

            // 移除这一行及其换行符
            buffer_.erase(0, newline + 1U);

            // 忽略空行，包括仅包含行末回车符的行
            if (TrimCarriageReturn(line).empty())
            {
                continue;
            }

            if (auto packet = DecodePacket(line))
            {
                packets.push_back(*packet);
            }
            else
            {
                ++invalid_line_count_;
            }
        }

        // 不完整的最后一行保留在 buffer_ 中，
        // 等待下一次 Append()。
        return packets;
    }

    bool ChassisPacketStream::Overflowed() const
    {
        return overflowed_;
    }

    std::size_t ChassisPacketStream::InvalidLineCount() const
    {
        return invalid_line_count_;
    }

    void ChassisPacketStream::ClearOverflow()
    {
        overflowed_ = false;
    }

    void ChassisPacketStream::ClearInvalidLineCount()
    {
        invalid_line_count_ = 0;
    }

    void ChassisPacketStream::Clear()
    {
        buffer_.clear();
        overflowed_ = false;
        invalid_line_count_ = 0;
    }

    // ================================================================
    // 六、二进制数据流解析器 MickBinaryPacketStream
    // ================================================================

    MickBinaryPacketStream::MickBinaryPacketStream(
        std::size_t max_buffer_size)
        : max_buffer_size_(max_buffer_size)
    {
    }

    // 追加二进制字节。
    void MickBinaryPacketStream::Append(
        const uint8_t* data,
        std::size_t size)
    {
        if (size == 0U)
        {
            return;
        }

        // 调用约定：size 非零时，data 必须指向有效内存。
        if (size > max_buffer_size_ ||
            buffer_.size() > max_buffer_size_ - size)
        {
            buffer_.clear();
            overflowed_ = true;
            return;
        }

        buffer_.insert(
            buffer_.end(),
            data,
            data + size);
    }

    // 方便调用者直接追加字节向量
    void MickBinaryPacketStream::Append(
        const std::vector<uint8_t>& data)
    {
        Append(data.data(), data.size());
    }

    // 在二进制缓冲区中寻找并提取所有完整数据帧。
    std::vector<ChassisPacket>
    MickBinaryPacketStream::DrainPackets()
    {
        std::vector<ChassisPacket> packets;

        constexpr std::array<uint8_t, 2> magic{
            0xAE, 0xEA
        };

        // 最短帧为：
        // 2 字节帧头 + 1 字节长度 + 2 字节长度内容 + 2 字节帧尾
        while (buffer_.size() >= 7U)
        {
            // 寻找帧头 AE EA
            const auto header = std::search(
                buffer_.begin(),
                buffer_.end(),
                magic.begin(),
                magic.end());

            // 找不到完整帧头：清除无用字节，
            // 但保留可能是半个帧头的末尾 AE。
            if (header == buffer_.end())
            {
                if (buffer_.back() == 0xAE)
                {
                    const uint8_t last_byte = buffer_.back();
                    buffer_.assign(1U, last_byte);
                }
                else
                {
                    buffer_.clear();
                }

                return packets;
            }

            // 丢弃帧头之前的噪声字节
            buffer_.erase(buffer_.begin(), header);

            // 找到帧头后，可能还没有足够的数据读取完整头部
            if (buffer_.size() < 7U)
            {
                return packets;
            }

            // LENGTH 表示 TYPE + DATA + CHECKSUM 的总长度
            const std::size_t length = buffer_[2];

            // 完整帧长度 = 2 字节帧头 + 1 字节长度 + LENGTH + 2 字节帧尾
            const std::size_t total_length = 3U + length + 2U;

            // 至少要有 TYPE 和 CHECKSUM 两个字节
            if (length < 2U)
            {
                buffer_.erase(buffer_.begin());
                ++invalid_frame_count_;
                continue;
            }

            // 数据尚未收齐，保留缓存，等待下一次 Append()
            if (buffer_.size() < total_length)
            {
                return packets;
            }

            // 检查帧尾 EF FE
            if (buffer_[total_length - 2U] != 0xEF ||
                buffer_[total_length - 1U] != 0xFE)
            {
                // 候选帧可能含有错误长度，只丢弃一个字节后重新同步
                buffer_.erase(buffer_.begin());
                ++invalid_frame_count_;
                continue;
            }

            // 重新计算校验和。
            // 范围与 EncodeMickBinaryFrame() 一致：
            // LENGTH、TYPE 和 DATA，不包含 CHECKSUM。
            uint8_t checksum = 0;

            for (std::size_t i = 2;
                 i < total_length - 3U;
                 ++i)
            {
                checksum = static_cast<uint8_t>(
                    checksum + buffer_[i]);
            }

            // 校验和不正确，尝试重新同步
            if (checksum != buffer_[total_length - 3U])
            {
                buffer_.erase(buffer_.begin());
                ++invalid_frame_count_;
                continue;
            }

            // 帧结构和校验均通过，提取类型和有效载荷
            MickBinaryFrame frame;

            frame.type = buffer_[3];

            frame.data.assign(
                buffer_.begin() + 4,
                buffer_.begin() + total_length - 3U);

            // 将载荷转换为统一的数据包
            if (auto packet = DecodeMickBinaryPacket(frame))
            {
                packets.push_back(*packet);
            }
            else
            {
                // 帧结构有效，但数据类型或载荷无法解析
                ++invalid_frame_count_;
            }

            // 移除已经处理的完整帧，继续寻找下一帧
            buffer_.erase(
                buffer_.begin(),
                buffer_.begin() + total_length);
        }

        return packets;
    }

    bool MickBinaryPacketStream::Overflowed() const
    {
        return overflowed_;
    }

    std::size_t MickBinaryPacketStream::InvalidFrameCount() const
    {
        return invalid_frame_count_;
    }

    void MickBinaryPacketStream::ClearOverflow()
    {
        overflowed_ = false;
    }

    void MickBinaryPacketStream::ClearInvalidFrameCount()
    {
        invalid_frame_count_ = 0;
    }

    void MickBinaryPacketStream::Clear()
    {
        buffer_.clear();
        overflowed_ = false;
        invalid_frame_count_ = 0;
    }
}