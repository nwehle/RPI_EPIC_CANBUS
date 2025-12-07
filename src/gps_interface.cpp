/*
 * GPS Interface Implementation
 */

#include "gps_interface.h"
#include <fcntl.h>
#include <unistd.h>
#include <cstring>
#include <cstdio>

GPSInterface::GPSInterface() : fd_(-1), enabled_(false) {}

GPSInterface::~GPSInterface() {
    close();
}

bool GPSInterface::init(const std::string& device, speed_t baudRate) {
    device_ = device;

    // Open serial port
    fd_ = open(device.c_str(), O_RDWR | O_NOCTTY | O_NONBLOCK);
    if (fd_ < 0) {
        std::perror(("open " + device).c_str());
        return false;
    }

    // Configure serial port
    struct termios tty;
    std::memset(&tty, 0, sizeof(tty));

    if (tcgetattr(fd_, &tty) != 0) {
        std::perror("tcgetattr");
        ::close(fd_);
        fd_ = -1;
        return false;
    }

    // Set baud rate
    cfsetospeed(&tty, baudRate);
    cfsetispeed(&tty, baudRate);

    // 8N1 mode
    tty.c_cflag &= ~PARENB;        // No parity
    tty.c_cflag &= ~CSTOPB;        // 1 stop bit
    tty.c_cflag &= ~CSIZE;
    tty.c_cflag |= CS8;            // 8 data bits
    tty.c_cflag &= ~CRTSCTS;       // No hardware flow control
    tty.c_cflag |= CREAD | CLOCAL; // Enable receiver, ignore modem lines

    // Raw input mode
    tty.c_lflag &= ~(ICANON | ECHO | ECHOE | ISIG);
    tty.c_iflag &= ~(IXON | IXOFF | IXANY);  // No software flow control
    tty.c_iflag &= ~(IGNBRK | BRKINT | PARMRK | ISTRIP | INLCR | IGNCR | ICRNL);

    // Raw output mode
    tty.c_oflag &= ~OPOST;

    // Non-blocking read
    tty.c_cc[VMIN] = 0;
    tty.c_cc[VTIME] = 0;

    if (tcsetattr(fd_, TCSANOW, &tty) != 0) {
        std::perror("tcsetattr");
        ::close(fd_);
        fd_ = -1;
        return false;
    }

    // Flush any pending data
    tcflush(fd_, TCIOFLUSH);

    std::printf("GPS interface '%s' initialized\n", device.c_str());
    return true;
}

void GPSInterface::close() {
    if (fd_ >= 0) {
        ::close(fd_);
        fd_ = -1;
    }
}

bool GPSInterface::readData() {
    if (fd_ < 0) return false;

    char buf[256];
    ssize_t n = read(fd_, buf, sizeof(buf));

    if (n <= 0) return false;

    bool parsed = false;
    for (ssize_t i = 0; i < n; ++i) {
        if (parser_.processChar(buf[i], gpsData_)) {
            parsed = true;
            if (!enabled_) {
                enabled_ = true;
            }
        }
    }

    return parsed;
}
