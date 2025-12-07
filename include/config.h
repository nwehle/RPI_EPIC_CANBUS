/*
 * Configuration System for RPI_EPIC_CANBUS
 * Allows flexible GPIO pin assignment via config file
 */

#ifndef CONFIG_H
#define CONFIG_H

#include <cstdint>
#include <string>
#include <vector>
#include <array>

// GPIO pin role definitions
enum class PinRole {
    UNUSED = 0,
    DIGITAL_INPUT,    // Digital input with pull-up
    DIGITAL_OUTPUT,   // Digital output (slow GPIO)
    VSS_INPUT,        // Wheel speed sensor input
    PWM_OUTPUT        // PWM output (on/off for now)
};

// Single GPIO pin configuration
struct PinConfig {
    int gpioNum = -1;           // BCM GPIO number (-1 = unused)
    PinRole role = PinRole::UNUSED;
    int channelIndex = 0;       // Index within its role (e.g., input 0-15, output 0-7)
    std::string name;           // Optional friendly name
};

// I/O mode presets
enum class IOMode {
    CUSTOM = 0,           // Use config file
    INPUTS_ONLY,          // 16 digital inputs, no outputs
    BALANCED,             // 8 inputs + 4 outputs + 4 VSS + 1 spare
    OUTPUTS_FOCUS,        // 4 inputs + 8 outputs + 4 VSS + 1 spare
    VSS_FOCUS             // 8 inputs + 4 outputs + 4 VSS + 1 spare (same as balanced)
};

// Complete configuration
struct Config {
    // Device paths
    std::string canInterface = "can0";
    std::string gpsDevice = "/dev/ttyUSB0";
    std::string i2cDevice = "/dev/i2c-1";
    std::string gpioChip = "/dev/gpiochip4";

    // GPS settings
    int gpsBaudRate = 9600;

    // I/O mode
    IOMode ioMode = IOMode::BALANCED;

    // GPIO pin configurations (up to 27 GPIOs on Pi)
    std::vector<PinConfig> pins;

    // Derived counts (populated after loading)
    int numDigitalInputs = 0;
    int numDigitalOutputs = 0;
    int numVssInputs = 0;
    int numPwmOutputs = 0;

    // Get pins by role
    std::vector<int> getDigitalInputPins() const;
    std::vector<int> getDigitalOutputPins() const;
    std::vector<int> getVssInputPins() const;
    std::vector<int> getPwmOutputPins() const;
};

// Configuration loader
class ConfigLoader {
public:
    // Load configuration from file
    static bool loadFromFile(const std::string& path, Config& config);

    // Save configuration to file
    static bool saveToFile(const std::string& path, const Config& config);

    // Apply a preset I/O mode
    static void applyPreset(IOMode mode, Config& config);

    // Generate default config file
    static bool generateDefaultConfig(const std::string& path);

private:
    static bool parseLine(const std::string& line, Config& config);
    static PinRole parseRole(const std::string& roleStr);
    static std::string roleToString(PinRole role);
};

#endif // CONFIG_H
