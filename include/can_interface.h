/*
 * CAN Interface for Raspberry Pi using SocketCAN
 * Handles communication with the Waveshare 2-channel CAN HAT
 */

#ifndef CAN_INTERFACE_H
#define CAN_INTERFACE_H

#include <cstdint>
#include <string>
#include <functional>
#include <linux/can.h>

class CANInterface {
public:
    // Callback type for received frames
    using FrameCallback = std::function<void(const struct can_frame&)>;

    CANInterface();
    ~CANInterface();

    // Initialize CAN interface (e.g., "can0" or "can1")
    bool init(const std::string& interface, uint32_t bitrate = 500000);

    // Close CAN interface
    void close();

    // Check if interface is open
    bool isOpen() const { return socket_ >= 0; }

    // Send a CAN frame (non-blocking)
    bool sendFrame(uint32_t canId, const uint8_t* data, uint8_t len);

    // Send variable set frame (EPIC protocol)
    bool sendVariableSetFrame(int32_t varHash, float value);
    bool sendVariableSetFrameU32(int32_t varHash, uint32_t value);
    bool sendVariableRequestFrame(int32_t varHash);

    // Receive frames (non-blocking, returns number of frames processed)
    int receiveFrames(FrameCallback callback);

    // Set hardware filter to accept only specific CAN IDs
    bool setFilter(uint32_t canId, uint32_t mask = 0x7FF);

    // Big-endian conversion utilities
    static void writeInt32BigEndian(int32_t value, uint8_t* out);
    static void writeFloat32BigEndian(float value, uint8_t* out);
    static int32_t readInt32BigEndian(const uint8_t* in);
    static float readFloat32BigEndian(const uint8_t* in);

private:
    int socket_;
    std::string interface_;
    struct can_frame txFrame_;
};

#endif // CAN_INTERFACE_H
