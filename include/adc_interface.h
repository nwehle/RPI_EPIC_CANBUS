/*
 * ADS1115 ADC Interface for Raspberry Pi
 * Supports up to 4 ADS1115 modules for 16 analog channels
 * Each ADS1115 provides 4 single-ended channels
 */

#ifndef ADC_INTERFACE_H
#define ADC_INTERFACE_H

#include <cstdint>
#include <string>
#include <array>

class ADCInterface {
public:
    // Number of channels per ADS1115 module
    static constexpr int CHANNELS_PER_MODULE = 4;
    // Maximum number of modules (for 16 channels)
    static constexpr int MAX_MODULES = 4;
    // Total number of channels
    static constexpr int TOTAL_CHANNELS = MAX_MODULES * CHANNELS_PER_MODULE;

    // Default I2C addresses for ADS1115 modules
    // ADDR pin: GND=0x48, VDD=0x49, SDA=0x4A, SCL=0x4B
    static constexpr uint8_t DEFAULT_ADDRESSES[MAX_MODULES] = {0x48, 0x49, 0x4A, 0x4B};

    ADCInterface();
    ~ADCInterface();

    // Initialize ADC interface
    // i2cDevice: e.g., "/dev/i2c-1"
    // addresses: array of I2C addresses for each module (use 0 to skip a module)
    bool init(const std::string& i2cDevice,
              const std::array<uint8_t, MAX_MODULES>& addresses = {0x48, 0x49, 0x4A, 0x4B});

    // Close I2C bus
    void close();

    // Check if interface is open
    bool isOpen() const { return fd_ >= 0; }

    // Read a single channel (0-15)
    // Returns raw ADC value (0-32767 for positive voltages in single-ended mode)
    int16_t readChannel(uint8_t channel);

    // Read all 16 channels at once
    void readAllChannels(float* values);

    // Convert raw ADC value to voltage (assuming ±4.096V full scale)
    static float rawToVoltage(int16_t raw);

    // Get which modules are present
    const std::array<bool, MAX_MODULES>& getModulesPresent() const { return modulesPresent_; }

private:
    int fd_;
    std::array<uint8_t, MAX_MODULES> addresses_;
    std::array<bool, MAX_MODULES> modulesPresent_;

    // ADS1115 register addresses
    static constexpr uint8_t REG_CONVERSION = 0x00;
    static constexpr uint8_t REG_CONFIG = 0x01;

    // Configuration bits
    static constexpr uint16_t CONFIG_OS_SINGLE = 0x8000;  // Start single conversion
    static constexpr uint16_t CONFIG_MUX_BASE = 0x4000;   // Single-ended AIN0
    static constexpr uint16_t CONFIG_PGA_4096 = 0x0200;   // ±4.096V range
    static constexpr uint16_t CONFIG_MODE_SINGLE = 0x0100; // Single-shot mode
    static constexpr uint16_t CONFIG_DR_860SPS = 0x00E0;  // 860 samples per second
    static constexpr uint16_t CONFIG_COMP_DISABLE = 0x0003; // Disable comparator

    // Select I2C slave address
    bool selectAddress(uint8_t addr);

    // Write 16-bit register
    bool writeRegister(uint8_t reg, uint16_t value);

    // Read 16-bit register
    int16_t readRegister(uint8_t reg);

    // Check if module is responding
    bool probeModule(uint8_t addr);
};

#endif // ADC_INTERFACE_H
