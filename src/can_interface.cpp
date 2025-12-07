/*
 * CAN Interface Implementation using SocketCAN
 */

#include "can_interface.h"
#include <cstring>
#include <unistd.h>
#include <fcntl.h>
#include <sys/ioctl.h>
#include <sys/socket.h>
#include <net/if.h>
#include <linux/can/raw.h>
#include <cstdio>

CANInterface::CANInterface() : socket_(-1) {
    std::memset(&txFrame_, 0, sizeof(txFrame_));
}

CANInterface::~CANInterface() {
    close();
}

bool CANInterface::init(const std::string& interface, uint32_t bitrate) {
    interface_ = interface;

    // Bring up the CAN interface with specified bitrate using system commands
    char cmd[256];
    std::snprintf(cmd, sizeof(cmd), "sudo ip link set %s down 2>/dev/null", interface.c_str());
    std::system(cmd);

    std::snprintf(cmd, sizeof(cmd), "sudo ip link set %s type can bitrate %u", interface.c_str(), bitrate);
    if (std::system(cmd) != 0) {
        std::fprintf(stderr, "Failed to set CAN bitrate. Is the interface '%s' available?\n", interface.c_str());
        return false;
    }

    std::snprintf(cmd, sizeof(cmd), "sudo ip link set %s up", interface.c_str());
    if (std::system(cmd) != 0) {
        std::fprintf(stderr, "Failed to bring up CAN interface '%s'\n", interface.c_str());
        return false;
    }

    // Create socket
    socket_ = socket(PF_CAN, SOCK_RAW, CAN_RAW);
    if (socket_ < 0) {
        std::perror("socket");
        return false;
    }

    // Set non-blocking mode
    int flags = fcntl(socket_, F_GETFL, 0);
    fcntl(socket_, F_SETFL, flags | O_NONBLOCK);

    // Get interface index
    struct ifreq ifr;
    std::strncpy(ifr.ifr_name, interface.c_str(), IFNAMSIZ - 1);
    ifr.ifr_name[IFNAMSIZ - 1] = '\0';
    if (ioctl(socket_, SIOCGIFINDEX, &ifr) < 0) {
        std::perror("ioctl SIOCGIFINDEX");
        ::close(socket_);
        socket_ = -1;
        return false;
    }

    // Bind to interface
    struct sockaddr_can addr;
    std::memset(&addr, 0, sizeof(addr));
    addr.can_family = AF_CAN;
    addr.can_ifindex = ifr.ifr_ifindex;

    if (bind(socket_, reinterpret_cast<struct sockaddr*>(&addr), sizeof(addr)) < 0) {
        std::perror("bind");
        ::close(socket_);
        socket_ = -1;
        return false;
    }

    std::printf("CAN interface '%s' initialized at %u bps\n", interface.c_str(), bitrate);
    return true;
}

void CANInterface::close() {
    if (socket_ >= 0) {
        ::close(socket_);
        socket_ = -1;
    }
}

bool CANInterface::setFilter(uint32_t canId, uint32_t mask) {
    if (socket_ < 0) return false;

    struct can_filter filter;
    filter.can_id = canId;
    filter.can_mask = mask | CAN_EFF_FLAG | CAN_RTR_FLAG;

    if (setsockopt(socket_, SOL_CAN_RAW, CAN_RAW_FILTER, &filter, sizeof(filter)) < 0) {
        std::perror("setsockopt CAN_RAW_FILTER");
        return false;
    }

    return true;
}

void CANInterface::writeInt32BigEndian(int32_t value, uint8_t* out) {
    out[0] = static_cast<uint8_t>((value >> 24) & 0xFF);
    out[1] = static_cast<uint8_t>((value >> 16) & 0xFF);
    out[2] = static_cast<uint8_t>((value >> 8) & 0xFF);
    out[3] = static_cast<uint8_t>(value & 0xFF);
}

void CANInterface::writeFloat32BigEndian(float value, uint8_t* out) {
    union { float f; uint32_t u; } conv;
    conv.f = value;
    out[0] = static_cast<uint8_t>((conv.u >> 24) & 0xFF);
    out[1] = static_cast<uint8_t>((conv.u >> 16) & 0xFF);
    out[2] = static_cast<uint8_t>((conv.u >> 8) & 0xFF);
    out[3] = static_cast<uint8_t>(conv.u & 0xFF);
}

int32_t CANInterface::readInt32BigEndian(const uint8_t* in) {
    return (static_cast<int32_t>(in[0]) << 24) |
           (static_cast<int32_t>(in[1]) << 16) |
           (static_cast<int32_t>(in[2]) << 8) |
           static_cast<int32_t>(in[3]);
}

float CANInterface::readFloat32BigEndian(const uint8_t* in) {
    union { float f; uint32_t u; } conv;
    conv.u = (static_cast<uint32_t>(in[0]) << 24) |
             (static_cast<uint32_t>(in[1]) << 16) |
             (static_cast<uint32_t>(in[2]) << 8) |
             static_cast<uint32_t>(in[3]);
    return conv.f;
}

bool CANInterface::sendFrame(uint32_t canId, const uint8_t* data, uint8_t len) {
    if (socket_ < 0 || len > 8) return false;

    txFrame_.can_id = canId;
    txFrame_.can_dlc = len;
    std::memcpy(txFrame_.data, data, len);

    ssize_t nbytes = write(socket_, &txFrame_, sizeof(struct can_frame));
    return (nbytes == sizeof(struct can_frame));
}

bool CANInterface::sendVariableSetFrame(int32_t varHash, float value) {
    uint8_t data[8];
    writeInt32BigEndian(varHash, &data[0]);
    writeFloat32BigEndian(value, &data[4]);
    return sendFrame(0x780 + 1, data, 8);  // CAN_ID_VARIABLE_SET
}

bool CANInterface::sendVariableSetFrameU32(int32_t varHash, uint32_t value) {
    uint8_t data[8];
    writeInt32BigEndian(varHash, &data[0]);
    writeInt32BigEndian(static_cast<int32_t>(value), &data[4]);
    return sendFrame(0x780 + 1, data, 8);  // CAN_ID_VARIABLE_SET
}

bool CANInterface::sendVariableRequestFrame(int32_t varHash) {
    uint8_t data[4];
    writeInt32BigEndian(varHash, &data[0]);
    return sendFrame(0x700 + 1, data, 4);  // CAN_ID_VAR_REQUEST
}

int CANInterface::receiveFrames(FrameCallback callback) {
    if (socket_ < 0) return 0;

    int count = 0;
    struct can_frame frame;

    while (true) {
        ssize_t nbytes = read(socket_, &frame, sizeof(struct can_frame));
        if (nbytes < 0) {
            break;  // No more frames (EAGAIN/EWOULDBLOCK)
        }
        if (nbytes == sizeof(struct can_frame)) {
            callback(frame);
            count++;
        }
    }

    return count;
}
