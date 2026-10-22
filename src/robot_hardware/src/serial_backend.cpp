#include "robot_hardware/serial_backend.hpp"

#include <cerrno>
#include <cstring>
#include <fcntl.h>
#include <string>
#include <termios.h>
#include <unistd.h>
#include <utility>
#include <stdexcept>

namespace robot_hardware
{
    namespace
    {
        // 自己的 baud rate → Linux termios baud rate
        speed_t baudToTermios(int baud_rate)
        {
            switch (baud_rate)
            {
            case 9600:
                return B9600;
            case 19200:
                return B19200;
            case 38400:
                return B38400;
            case 57600:
                return B57600;
            case 115200:
                return B115200;
            case 230400:
                return B230400;
            case 460800:
                return B460800;
            case 921600:
                return B921600;
            default:
                throw std::invalid_argument("Unsupported baud rate: " + std::to_string(baud_rate));
            }
        }

        bool ConfigureSerialPort(const int fd, const int baud, std::string *error)
        {
            // Linux 保存串口配置的一张“配置表”， 创建一份串口配置
            termios tty{};

            // 把当前串口的配置读取出来，放进 tty
            if (tcgetattr(fd, &tty) != 0)
            {
                if (error)
                {
                    *error = "Failed to get terminal attributes: " + std::string(std::strerror(errno));
                }
                return false;
            }

            // 把串口设置成“原始数据模式”，不进行任何处理
            cfmakeraw(&tty);
            const speed_t speed = baudToTermios(baud);

            // 设置波特率 ,输入输出的波特率
            cfsetispeed(&tty, speed);
            cfsetospeed(&tty, speed);

            // 设置串口格式
            tty.c_cflag |= static_cast<tcflag_t>(CLOCAL | CREAD);
            tty.c_cflag &= static_cast<tcflag_t>(~PARENB);
            tty.c_cflag &= static_cast<tcflag_t>(~CSTOPB);
            tty.c_cflag &= static_cast<tcflag_t>(~CSIZE);
            tty.c_cflag |= CS8;

            // 控制 read() 等待数据的行为，
            tty.c_cc[VMIN] = 0;
            tty.c_cc[VTIME] = 1;

            // 在内存里修改配置,把配置写回串口
            if (tcsetattr(fd, TCSANOW, &tty) != 0)
            {
                if (error != nullptr)
                {
                    *error = std::strerror(errno);
                }
                return false;
            }
            return true;
        }
    }

    SerialBackend::SerialBackend(ChassisBackendConfig config) : config_(std::move(config)) {}
    
    SerialBackend::~SerialBackend()
    {
        Close();
    }
    
    std::string SerialBackend::Name() const
    {
        return "SerialBackend";
    }

    bool SerialBackend::Open(std::string *error)
    {
        // 先关闭旧串口
        Close();

        // 读写、非控制终端、非阻塞方式打开串口设备
        fd_ = open(config_.serial_device.c_str(), O_RDWR | O_NOCTTY | O_NONBLOCK);
        if (fd_ < 0)
        {
            if (error)
            {
                *error = "Failed to open serial device: " + std::string(std::strerror(errno));
            }
            return false;
        }

        std::string config_error;
        if (!ConfigureSerialPort(fd_, config_.serial_baud, &config_error))
        {
            if (error)
            {
                *error = "Failed to configure serial port: " + config_error;
            }
            Close();
            return false;
        }
        return true;
    }

    void SerialBackend::Close()
    {
        if (fd_ >= 0)
        {
            close(fd_);
            fd_ = -1;
        }
    }

    bool SerialBackend::IsOpen() const
    {
        return fd_ >= 0;
    }

    bool SerialBackend::WriteCommand(const ChassisCommand &command, std::string *error)
    {
        if (!IsOpen())
        {
            if (error)
            {
                *error = "Serial port is not open.";
            }
            return false;
        }

        // 将内存数据结构中的数据  转换成真正需要发送的数据
        const auto binary_packet = EncodeMickSpeedCommand(command);
        const std::string test_packet = EncodeCommand(command);

        const char *data = config_.protocol == "mick_binary" ? reinterpret_cast<const char *>(binary_packet.data()) : test_packet.data();
        const std::size_t size = config_.protocol == "mick_binary" ? binary_packet.size() : test_packet.size();

        // 把数据交给 Linux 串口驱动
        const ssize_t written = write(fd_, data, size);
        if (written < 0)
        {
            if (error)
            {
                *error = "Failed to write to serial port: " + std::string(std::strerror(errno));
            }
            return false;
        }
        return written >= 0 || errno == EAGAIN || errno == EWOULDBLOCK;
    }

    std::optional<std::string> SerialBackend::Read(std::string *error)
    {
        if (!IsOpen())
        {
            if (error)
            {
                *error = "Serial port is not open.";
            }
            return std::nullopt;
        }

        char buffer[512];
        const ssize_t bytes_read = read(fd_, buffer, sizeof(buffer));
        if (bytes_read < 0)
        {
            if (error)
            {
                *error = "Failed to read from serial port: " + std::string(std::strerror(errno));
            }
            return std::nullopt;
        }
        else if (bytes_read == 0)
        {
            // No data available
            return std::nullopt;
        }

        return std::optional<std::string>(std::string(buffer, bytes_read));
    }
}