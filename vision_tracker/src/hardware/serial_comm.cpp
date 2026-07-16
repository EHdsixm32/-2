#include "hardware/serial_comm.hpp"
#include <fcntl.h>
#include <unistd.h>
#include <cstring>
#include <spdlog/spdlog.h>

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

uint8_t SerialComm::calculateChecksum(const std::vector<uint8_t>& data) {
    uint8_t sum = 0;
    for (size_t i = 0; i < data.size(); ++i) sum ^= data[i];
    return sum;
}

std::vector<uint8_t> SerialComm::pack(const ControlCommand& cmd) {
    std::vector<uint8_t> data;
    data.push_back(0xA5);
    data.push_back(0x5A);
    float yaw = cmd.yaw_angle;
    float pitch = cmd.pitch_angle;
    uint8_t* yaw_ptr = reinterpret_cast<uint8_t*>(&yaw);
    uint8_t* pitch_ptr = reinterpret_cast<uint8_t*>(&pitch);
    data.insert(data.end(), yaw_ptr, yaw_ptr + 4);
    data.insert(data.end(), pitch_ptr, pitch_ptr + 4);
    data.push_back(cmd.laser_enable ? 1 : 0);
    data.push_back(calculateChecksum(data));
    return data;
}

bool SerialComm::sendCommand(const ControlCommand& cmd) {
    if (fd_ < 0) return false;
    auto data = pack(cmd);
    int written = ::write(fd_, data.data(), data.size());
    if (written != (int)data.size()) {
        spdlog::warn("Serial write incomplete");
        return false;
    }
    return true;
}
