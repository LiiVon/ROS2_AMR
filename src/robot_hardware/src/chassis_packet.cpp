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

        // Windows	\r\n  回车 + 换行
        // Linux	\n    换行
        // 去掉字符串末尾的 \r， 避免 Windows 和 Linux 换行符不同，导致后续协议解析出错。
        // CMD 1.0 2.0\r   直接去除了 \r
        // CMD 1.0 2.0\r\n   std::getline() 会去掉 \n，剩下 \r，然后再去除 \r
        std::string TrimCarriageReturn(std::string line)
        {
            if (!line.empty() && line.back() == '\r')
            {
                line.pop_back();
            }

            return line;
        }

        // 把一个有符号的 16 位整数（int16_t）拆成两个字节，并按照大端序放进字节数组中
        // int16_t value = 1000;  对应两个字节 0x03 0xE8
        // 规定都按照 大端序，这样不同平台的字节序就不会影响数据的传输和解析。
        void AppendInt16(std::vector<uint8_t> &out, int16_t value)
        {
            const uint16_t bits = static_cast<uint16_t>(value);

            // 先右移8位，高8位变成低8位，然后取低8位，得到高字节
            out.push_back(
                static_cast<uint8_t>((bits >> 8) & 0xFFU));

            // 直接取低8位，得到低字节
            out.push_back(
                static_cast<uint8_t>(bits & 0xFFU));
        }

        // 从指定偏移读取两个字节，按大端序解释为无符号 16 位整数
        uint16_t ReadUInt16(
            const std::vector<uint8_t> &data,
            std::size_t offset)
        {
            // data.at(0) 访问第一个字节，也就是高字节，高字节左移8位
            // data.at(1) 访问第二个字节，也就是低字节，直接取低字节
            return static_cast<uint16_t>(
                (static_cast<uint16_t>(data.at(offset)) << 8) |
                static_cast<uint16_t>(data.at(offset + 1U)));
        }

        // 从指定偏移读取两个字节，按大端序解释为有符号 16 位整数
        int16_t ReadInt16(
            const std::vector<uint8_t> &data,
            std::size_t offset)
        {
            const uint16_t bits = ReadUInt16(data, offset);

            // 小于 int16_t 的最大值，说明是正数或0，直接转换为 int16_t 即可。
            if (bits <= static_cast<uint16_t>(
                            std::numeric_limits<int16_t>::max()))
            {
                return static_cast<int16_t>(bits);
            }

            // 大于 int16_t 的最大值，说明是负数，按补码规则还原为负数。
            const int32_t signed_value =
                static_cast<int32_t>(bits) - 0x10000;

            return static_cast<int16_t>(signed_value);
        }

        // 将有限浮点数四舍五入，并钳制在 int16_t 范围内
        // 与旧版一致：超限不发异常，而是饱和处理，保证节点不会因异常退出
        int16_t EncodeInt16Checked(double value)
        {
            if (!std::isfinite(value))
            {
                return 0;
            }

            const double clamped =
                std::clamp(value, -32768.0, 32767.0);

            return static_cast<int16_t>(std::lround(clamped));
        }

        // 读取可选的有限浮点数。
        // 字段不存在时返回 true，且不修改 optional。
        // 字段存在但格式错误、不是有限数或后面还有多余字段时返回 false。
        bool ReadOptionalFiniteDouble(
            std::istringstream &iss,
            std::optional<double> &value)
        {
            // 跳过输入流开头的空格、制表符、换行符等空白字符
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

        // 将非有限值饱和为 0，避免 NaN/Inf 进入通信链路
        double SanitizeFinite(double value)
        {
            return std::isfinite(value) ? value : 0.0;
        }

        // 检查多个速度、位置和角度字段是否全部为有限数
        bool IsFiniteOdometry(const ChassisOdometryPacket &odometry)
        {
            return std::isfinite(odometry.x_m) &&
                   std::isfinite(odometry.y_m) &&
                   std::isfinite(odometry.yaw_rad) &&
                   std::isfinite(odometry.linear_x_mps) &&
                   std::isfinite(odometry.linear_y_mps) &&
                   std::isfinite(odometry.angular_z_radps);
        }
    }

    // ================================================================
    // 一、文本协议编码
    // ================================================================

    // 将底盘速度指令编码为：
    // 内部数据结构  ---> 文本协议
    // CMD linear_x_mps linear_y_mps angular_z_radps\n
    std::string EncodeCommand(const ChassisCommand &command)
    {
        std::ostringstream oss;

        oss << std::fixed << std::setprecision(4)
            << "CMD "
            << SanitizeFinite(command.linear_x_mps) << ' '
            << SanitizeFinite(command.linear_y_mps) << ' '
            << SanitizeFinite(command.angular_z_radps) << '\n';

        return oss.str();
    }

    // 将里程计数据编码为文本
    // 内部数据结构  ---> 文本协议
    // ODOM x y yaw vx vy wz [battery]\n
    std::string EncodeOdometry(
        const ChassisOdometryPacket &odometry)
    {
        // 保持与旧版线协议一致：x y yaw vx wz [batt]（不发送横向速度）
        std::ostringstream oss;

        oss << std::fixed << std::setprecision(4)
            << "ODOM "
            << SanitizeFinite(odometry.x_m) << ' '
            << SanitizeFinite(odometry.y_m) << ' '
            << SanitizeFinite(odometry.yaw_rad) << ' '
            << SanitizeFinite(odometry.linear_x_mps) << ' '
            << SanitizeFinite(odometry.angular_z_radps);

        if (odometry.battery_voltage.has_value() &&
            std::isfinite(*odometry.battery_voltage))
        {
            oss << ' ' << *odometry.battery_voltage;
        }

        oss << '\n';

        return oss.str();
    }

    // 将底盘状态编码为文本
    // 内部数据结构  ---> 文本协议
    // STATE status [battery]\n
    std::string EncodeState(const ChassisStatePacket &state)
    {
        // 检查状态描述是否合法
        if (state.status.empty() ||
            state.status.find_first_of(" \t\r\n\v\f") != std::string::npos)
        {
            throw std::invalid_argument(
                "Invalid chassis status");
        }

        // 检查电池电压是否为有限数
        if (state.battery_voltage.has_value() &&
            !std::isfinite(*state.battery_voltage))
        {
            throw std::invalid_argument(
                "Invalid battery voltage");
        }

        // 构建文本数据
        std::ostringstream oss;

        oss << "STATE " << state.status;

        // 如果存在电池电压，则追加电压数据
        if (state.battery_voltage.has_value())
        {
            oss << ' '
                << std::fixed << std::setprecision(4)
                << *state.battery_voltage;
        }

        oss << '\n';

        return oss.str();
    }

    // ================================================================
    // 二、文本协议解码
    // ================================================================

    // 解码速度指令。
    // 当前协议要求 CMD 后必须有三个速度参数。
    // 文本协议  ---> 内部数据结构
    std::optional<ChassisCommand> DecodeCommand(
        const std::string &text)
    {
        const std::string line = TrimCarriageReturn(text);
        std::istringstream iss(line);

        std::string tag;
        ChassisCommand command;

        if (!(iss >> tag >> command.linear_x_mps))
        {
            return std::nullopt;
        }

        if (tag != "CMD")
        {
            return std::nullopt;
        }

        // 兼容旧格式：CMD <vx> <wz>；新格式：CMD <vx> <vy> <wz>
        std::vector<double> values;
        double value = 0.0;
        while (iss >> value)
        {
            values.push_back(value);
        }

        if (values.size() == 1U)
        {
            command.angular_z_radps = values[0];
        }
        else if (values.size() == 2U)
        {
            command.linear_y_mps = values[0];
            command.angular_z_radps = values[1];
        }
        else
        {
            return std::nullopt;
        }

        if (!std::isfinite(command.linear_x_mps) ||
            !std::isfinite(command.linear_y_mps) ||
            !std::isfinite(command.angular_z_radps))
        {
            return std::nullopt;
        }

        return command;
    }

    // 解码统一的数据包。
    // 根据首字段判断是 ODOM 还是 STATE。
    // 文本协议  ---> 内部数据结构
    std::optional<ChassisPacket> DecodePacket(
        const std::string &text)
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

            if (!(iss >> odometry.x_m >> odometry.y_m >> odometry.yaw_rad >> odometry.linear_x_mps))
            {
                return std::nullopt;
            }

            // 兼容三种形态：
            //   5 个数：x y yaw vx wz
            //   6 个数：x y yaw vx wz batt（旧版带电池）
            //   7 个数：x y yaw vx vy wz batt（全向扩展）
            std::vector<double> rest;
            double value = 0.0;
            while (iss >> value)
            {
                rest.push_back(value);
            }

            if (rest.size() == 1U)
            {
                odometry.angular_z_radps = rest[0];
            }
            else if (rest.size() == 2U)
            {
                odometry.angular_z_radps = rest[0];
                odometry.battery_voltage = rest[1];
            }
            else if (rest.size() == 3U)
            {
                odometry.linear_y_mps = rest[0];
                odometry.angular_z_radps = rest[1];
                odometry.battery_voltage = rest[2];
            }
            else
            {
                return std::nullopt;
            }

            if (!IsFiniteOdometry(odometry))
            {
                return std::nullopt;
            }

            if (odometry.battery_voltage.has_value() &&
                !std::isfinite(*odometry.battery_voltage))
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
    // 内部数据结构----> Mick二进制数据
    std::vector<uint8_t> EncodeMickBinaryFrame(
        const uint8_t type,
        const std::vector<uint8_t> &data)
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

        // 帧头 AE EA
        frame.push_back(0xAE);
        frame.push_back(0xEA);

        // 长度字段 LENGTH
        frame.push_back(
            static_cast<uint8_t>(data.size() + 2U));

        // 数据类型 TYPE
        frame.push_back(type);

        // 数据载荷 DATA
        frame.insert(frame.end(), data.begin(), data.end());

        // 校验和：从 LENGTH 开始，累加至载荷最后一个字节  CHECKSUM
        uint8_t checksum = 0;

        for (std::size_t i = 2; i < frame.size(); ++i)
        {
            checksum = static_cast<uint8_t>(
                checksum + frame[i]);
        }

        frame.push_back(checksum);

        // 帧尾 EF FE
        frame.push_back(0xEF);
        frame.push_back(0xFE);

        return frame;
    }

    // 将速度命令编码为 Mick 二进制指令。
    //
    // 每个速度字段使用 2 字节有符号整数。
    // 两个线速度按 mm/s 编码，角速度按 mrad/s 编码。
    // 内部数据结构----> Mick二进制数据
    std::vector<uint8_t> EncodeMickSpeedCommand(
        const ChassisCommand &command)
    {
        std::vector<uint8_t> data;
        data.reserve(6);

        AppendInt16(
            data,
            EncodeInt16Checked(command.linear_x_mps * 1000.0));

        AppendInt16(
            data,
            EncodeInt16Checked(command.linear_y_mps * 1000.0));

        AppendInt16(
            data,
            EncodeInt16Checked(command.angular_z_radps * 1000.0));

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
    // 本函数不负责帧头、帧尾和校验和检查, 这些工作由 MickBinaryPacketStream 完成。
    // Mick二进制数据 ---> 内部数据结构
    std::optional<ChassisPacket> DecodeMickBinaryPacket(
        const MickBinaryFrame &frame)
    {
        // ------------------------------------------------------------
        // 0xA1：里程计数据
        // ------------------------------------------------------------
        if (frame.type == 0xA1)
        {
            // 兼容旧版 12 字节（无横向速度）与扩展 14 字节（含横向速度）
            if (frame.data.size() != 12U &&
                frame.data.size() != 14U)
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

            if (frame.data.size() == 14U)
            {
                odometry.linear_y_mps =
                    ReadInt16(frame.data, 8) / 1000.0;

                odometry.angular_z_radps =
                    ReadInt16(frame.data, 10) / 1000.0;

                odometry.battery_voltage =
                    ReadUInt16(frame.data, 12) / 100.0;
            }
            else
            {
                odometry.angular_z_radps =
                    ReadInt16(frame.data, 8) / 1000.0;

                odometry.battery_voltage =
                    ReadUInt16(frame.data, 10) / 100.0;
            }

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
            // 合法载荷长度：0（旧版空载荷视为运行中）、1（仅状态码）、3（状态码+电压）
            if (frame.data.size() != 0U &&
                frame.data.size() != 1U &&
                frame.data.size() != 3U)
            {
                return std::nullopt;
            }

            ChassisStatePacket state;

            state.status =
                frame.data.empty() || frame.data[0] == 0 ? "running" : "fault";

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

    ChassisPacketStream::ChassisPacketStream(std::size_t max_buffer_size)
        : max_buffer_size_(max_buffer_size)
    {
    }

    // 追加一段文本数据。
    // 先检查容量，避免超大输入先触发大规模内存分配。
    void ChassisPacketStream::Append(const char *data, std::size_t size)
    {
        if (size == 0U)
        {
            return;
        }

        if (data == nullptr)
        {
            throw std::invalid_argument(
                "Cannot append from a null pointer");
        }

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
    void ChassisPacketStream::Append(const std::string &data)
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
    void MickBinaryPacketStream::Append(const char *data, std::size_t size)
    {
        if (size == 0U)
        {
            return;
        }

        if (data == nullptr)
        {
            throw std::invalid_argument(
                "Cannot append from a null pointer");
        }

        // 调用约定：size 非零时，data 必须指向有效内存。
        if (size > max_buffer_size_ ||
            buffer_.size() > max_buffer_size_ - size)
        {
            buffer_.clear();
            overflowed_ = true;
            return;
        }

        buffer_.insert(buffer_.end(), data, data + size);
    }

    // 方便调用者直接追加字节向量
    void MickBinaryPacketStream::Append(const std::string &data)
    {
        Append(data.data(), data.size());
    }

    // 在二进制缓冲区中寻找并提取所有完整数据帧。
    std::vector<ChassisPacket>
    MickBinaryPacketStream::DrainPackets()
    {
        std::vector<ChassisPacket> packets;

        constexpr std::array<uint8_t, 2> magic{0xAE, 0xEA};

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