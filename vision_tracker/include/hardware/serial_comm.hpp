#pragma once
#include <string>
#include <vector>
#include <cstdint>
#include <termios.h>

struct ControlCommand {
    float yaw_angle;
    float pitch_angle;
    bool laser_enable;
};

class SerialComm {
public:
    SerialComm(const std::string& port, int baudrate = 115200);
    ~SerialComm();
    bool open();
    void close();
    bool sendCommand(const ControlCommand& cmd);
    bool isOpen() const { return fd_ > 0; }

private:
    int fd_;
    std::string port_;
    int baudrate_;
    struct termios tty_;

    std::vector<uint8_t> pack(const ControlCommand& cmd);
    uint8_t calculateChecksum(const std::vector<uint8_t>& data);
};
