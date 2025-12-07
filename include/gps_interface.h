/*
 * GPS Interface for Raspberry Pi
 * Handles USB serial GPS communication (Adafruit Ultimate GPS)
 */

#ifndef GPS_INTERFACE_H
#define GPS_INTERFACE_H

#include "nmea_parser.h"
#include <string>
#include <termios.h>

class GPSInterface {
public:
    GPSInterface();
    ~GPSInterface();

    // Initialize GPS serial port (e.g., "/dev/ttyUSB0" or "/dev/ttyACM0")
    bool init(const std::string& device, speed_t baudRate = B9600);

    // Close GPS interface
    void close();

    // Check if interface is open
    bool isOpen() const { return fd_ >= 0; }

    // Read and parse GPS data (non-blocking)
    // Returns true if new valid data was parsed
    bool readData();

    // Get current GPS data
    const GPSData& getData() const { return gpsData_; }

    // Check if GPS has been enabled (received valid data at least once)
    bool isEnabled() const { return enabled_; }

    // Check if GPS has fix
    bool hasFix() const { return gpsData_.hasFix; }

private:
    int fd_;
    std::string device_;
    NMEAParser parser_;
    GPSData gpsData_;
    bool enabled_;
};

#endif // GPS_INTERFACE_H
