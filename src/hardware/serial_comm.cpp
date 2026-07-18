#include "hardware/serial_comm.hpp"
#include <fcntl.h>
#include <unistd.h>
#include <cstring>
#include <spdlog/spdlog.h>
#include <cstdio>

SerialComm::SerialComm(const std::string& port, int baudrate)
    : port_(port), baudrate_(baudrate), fd_(-1) {}

SerialComm::~SerialComm() { close(); }

bool SerialComm::open() {
    fd_ = ::open(port_.c_str(), O_RDWR | O_NOCTTY | O_SYNC);
    if (fd_ < 0) {
        spdlog::error("Serial open failed: {}", port_);
        return false;
    }
    tcgetattr(fd_, &tty_);
    cfsetospeed(&tty_, B115200);
    cfsetispeed(&tty_, B115200);
    tty_.c_cflag = (tty_.c_cflag & ~CSIZE) | CS8;
    tty_.c_iflag &= ~IGNBRK;
    tty_.c_lflag = 0;
    tty_.c_oflag = 0;
    tty_.c_cc[VMIN] = 0;
    tty_.c_cc[VTIME] = 5;
    tcsetattr(fd_, TCSANOW, &tty_);
    spdlog::info("Serial port {} opened", port_);
    return true;
}

void SerialComm::close() {
    if (fd_ > 0) { ::close(fd_); fd_ = -1; }
}

//uint8_t SerialComm::calculateChecksum(const std::vector<uint8_t>& data) {
    //uint8_t sum = 0;
    //for (size_t i = 0; i < data.size(); ++i) sum ^= data[i];
    //return sum;
//}

std::vector<uint8_t> SerialComm::pack(const ControlCommand& cmd) {
    std::vector<uint8_t> data;

    // 1. 帧头 (注意顺序：5A A5)
    data.push_back(0x5A);
    data.push_back(0xA5);

    // 2. mode (1字节) - 使用 AUTO_AIM 模式，通常对应枚举值 1 或 2
    // 根据你的 GimbalMode 定义: IDLE=0, AUTO_AIM=1, SMALL_BUFF=2, BIG_BUFF=3
    uint8_t mode = 1;  // AUTO_AIM
    data.push_back(mode);

    // 3. yaw (float, 4字节, 小端)
    float yaw = cmd.yaw_angle;
    uint8_t* yaw_ptr = reinterpret_cast<uint8_t*>(&yaw);
    data.insert(data.end(), yaw_ptr, yaw_ptr + 4);

    // 4. yaw_vel (float, 4字节) - 暂时设为 0
    float yaw_vel = 0.0f;
    uint8_t* yaw_vel_ptr = reinterpret_cast<uint8_t*>(&yaw_vel);
    data.insert(data.end(), yaw_vel_ptr, yaw_vel_ptr + 4);

    // 5. yaw_acc (float, 4字节) - 暂时设为 0
    float yaw_acc = 0.0f;
    uint8_t* yaw_acc_ptr = reinterpret_cast<uint8_t*>(&yaw_acc);
    data.insert(data.end(), yaw_acc_ptr, yaw_acc_ptr + 4);

    // 6. pitch (float, 4字节, 小端)
    float pitch = cmd.pitch_angle;
    uint8_t* pitch_ptr = reinterpret_cast<uint8_t*>(&pitch);
    data.insert(data.end(), pitch_ptr, pitch_ptr + 4);

    // 7. pitch_vel (float, 4字节) - 暂时设为 0
    float pitch_vel = 0.0f;
    uint8_t* pitch_vel_ptr = reinterpret_cast<uint8_t*>(&pitch_vel);
    data.insert(data.end(), pitch_vel_ptr, pitch_vel_ptr + 4);

    // 8. pitch_acc (float, 4字节) - 暂时设为 0
    float pitch_acc = 0.0f;
    uint8_t* pitch_acc_ptr = reinterpret_cast<uint8_t*>(&pitch_acc);
    data.insert(data.end(), pitch_acc_ptr, pitch_acc_ptr + 4);

    // 9. 帧尾 (7F FE)
    data.push_back(0x7F);
    data.push_back(0xFE);

    return data;
}

bool SerialComm::sendCommand(const ControlCommand& cmd) {
    if (fd_ < 0) return false;
    auto data = pack(cmd);


        // ===== 打印发送的十六进制数据 =====
    std::string hex;
    for (size_t i = 0; i < data.size(); ++i) {
        char buf[4];
        sprintf(buf, "%02X ", data[i]);
        hex += buf;
    }
    spdlog::info("Sending: {}", hex);
    // ==================================


    int written = ::write(fd_, data.data(), data.size());
    if (written != (int)data.size()) {
        spdlog::warn("Serial write incomplete");
        return false;
    }
    return true;
}
