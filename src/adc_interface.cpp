/*
 * ADS1115 ADC Interface Implementation
 */

#include "adc_interface.h"
#include <fcntl.h>
#include <unistd.h>
#include <sys/ioctl.h>
#include <linux/i2c-dev.h>
#include <cstdio>
#include <cstring>

// Define I2C_SLAVE if not defined
#ifndef I2C_SLAVE
#define I2C_SLAVE 0x0703
#endif

constexpr uint8_t ADCInterface::DEFAULT_ADDRESSES[MAX_MODULES];

ADCInterface::ADCInterface() : fd_(-1) {
    addresses_.fill(0);
    modulesPresent_.fill(false);
}

ADCInterface::~ADCInterface() {
    close();
}

bool ADCInterface::init(const std::string& i2cDevice,
                        const std::array<uint8_t, MAX_MODULES>& addresses) {
    addresses_ = addresses;

    // Open I2C bus
    fd_ = open(i2cDevice.c_str(), O_RDWR);
    if (fd_ < 0) {
        std::perror(("open " + i2cDevice).c_str());
        return false;
    }

    // Probe each module to see which are present
    int modulesFound = 0;
    for (int i = 0; i < MAX_MODULES; ++i) {
        if (addresses_[i] != 0) {
            modulesPresent_[i] = probeModule(addresses_[i]);
            if (modulesPresent_[i]) {
                std::printf("ADS1115 module %d found at 0x%02X (channels %d-%d)\n",
                           i, addresses_[i], i * CHANNELS_PER_MODULE,
                           i * CHANNELS_PER_MODULE + CHANNELS_PER_MODULE - 1);
                modulesFound++;
            }
        }
    }

    if (modulesFound == 0) {
        std::fprintf(stderr, "Warning: No ADS1115 modules found on I2C bus\n");
    }

    std::printf("ADC interface initialized: %d module(s) found\n", modulesFound);
    return true;  // Return true even if no modules found (optional hardware)
}

void ADCInterface::close() {
    if (fd_ >= 0) {
        ::close(fd_);
        fd_ = -1;
    }
}

bool ADCInterface::selectAddress(uint8_t addr) {
    if (fd_ < 0) return false;
    return ioctl(fd_, I2C_SLAVE, addr) >= 0;
}

bool ADCInterface::writeRegister(uint8_t reg, uint16_t value) {
    uint8_t buf[3];
    buf[0] = reg;
    buf[1] = (value >> 8) & 0xFF;  // MSB first
    buf[2] = value & 0xFF;

    return write(fd_, buf, 3) == 3;
}

int16_t ADCInterface::readRegister(uint8_t reg) {
    uint8_t buf[2];

    // Write register address
    if (write(fd_, &reg, 1) != 1) return 0;

    // Read 2 bytes
    if (read(fd_, buf, 2) != 2) return 0;

    return static_cast<int16_t>((buf[0] << 8) | buf[1]);
}

bool ADCInterface::probeModule(uint8_t addr) {
    if (!selectAddress(addr)) return false;

    // Try to read config register
    uint8_t reg = REG_CONFIG;
    if (write(fd_, &reg, 1) != 1) return false;

    uint8_t buf[2];
    if (read(fd_, buf, 2) != 2) return false;

    // Check if we got a valid config value (default is 0x8583)
    uint16_t config = (buf[0] << 8) | buf[1];
    return (config != 0x0000 && config != 0xFFFF);
}

int16_t ADCInterface::readChannel(uint8_t channel) {
    if (fd_ < 0 || channel >= TOTAL_CHANNELS) return 0;

    // Determine which module and local channel
    uint8_t moduleIndex = channel / CHANNELS_PER_MODULE;
    uint8_t localChannel = channel % CHANNELS_PER_MODULE;

    if (!modulesPresent_[moduleIndex]) return 0;
    if (!selectAddress(addresses_[moduleIndex])) return 0;

    // Build config register value
    // MUX[14:12]: 100=AIN0, 101=AIN1, 110=AIN2, 111=AIN3 (single-ended vs GND)
    uint16_t mux = (0x04 + localChannel) << 12;
    uint16_t config = CONFIG_OS_SINGLE |   // Start conversion
                      mux |                 // Select channel
                      CONFIG_PGA_4096 |     // ±4.096V range
                      CONFIG_MODE_SINGLE |  // Single-shot mode
                      CONFIG_DR_860SPS |    // 860 SPS (fastest)
                      CONFIG_COMP_DISABLE;  // Disable comparator

    // Write config to start conversion
    if (!writeRegister(REG_CONFIG, config)) return 0;

    // Wait for conversion (max ~1.2ms at 860 SPS)
    usleep(1500);

    // Read conversion result
    return readRegister(REG_CONVERSION);
}

void ADCInterface::readAllChannels(float* values) {
    if (!values) return;

    for (int i = 0; i < TOTAL_CHANNELS; ++i) {
        int16_t raw = readChannel(i);
        // Convert to float compatible with Arduino ADC (0-1023 range)
        // ADS1115 is 16-bit signed, but single-ended mode gives 0-32767
        // Scale to 0-1023 to match Arduino behavior
        if (raw < 0) raw = 0;
        values[i] = static_cast<float>(raw) * 1023.0f / 32767.0f;
    }
}

float ADCInterface::rawToVoltage(int16_t raw) {
    // For ±4.096V range, 1 LSB = 4.096V / 32768 = 0.125mV
    return raw * 4.096f / 32768.0f;
}
