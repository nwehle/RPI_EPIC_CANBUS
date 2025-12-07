/*
 * Configuration System Implementation
 */

#include "config.h"
#include <fstream>
#include <sstream>
#include <algorithm>
#include <cctype>
#include <cstdio>

// Available GPIOs on Pi 5 (excluding I2C, SPI used by CAN HAT)
// Reserved: GPIO 2,3 (I2C), GPIO 7,8,9,10,11 (SPI), GPIO 24,25 (CAN INT)
static const std::vector<int> AVAILABLE_GPIOS = {
    4, 5, 6, 12, 13, 14, 15, 16, 17, 18, 19, 20, 21, 22, 23, 26, 27
};

std::vector<int> Config::getDigitalInputPins() const {
    std::vector<int> result;
    for (const auto& pin : pins) {
        if (pin.role == PinRole::DIGITAL_INPUT && pin.gpioNum >= 0) {
            result.push_back(pin.gpioNum);
        }
    }
    return result;
}

std::vector<int> Config::getDigitalOutputPins() const {
    std::vector<int> result;
    for (const auto& pin : pins) {
        if (pin.role == PinRole::DIGITAL_OUTPUT && pin.gpioNum >= 0) {
            result.push_back(pin.gpioNum);
        }
    }
    return result;
}

std::vector<int> Config::getVssInputPins() const {
    std::vector<int> result;
    for (const auto& pin : pins) {
        if (pin.role == PinRole::VSS_INPUT && pin.gpioNum >= 0) {
            result.push_back(pin.gpioNum);
        }
    }
    return result;
}

std::vector<int> Config::getPwmOutputPins() const {
    std::vector<int> result;
    for (const auto& pin : pins) {
        if (pin.role == PinRole::PWM_OUTPUT && pin.gpioNum >= 0) {
            result.push_back(pin.gpioNum);
        }
    }
    return result;
}

static std::string trim(const std::string& str) {
    size_t start = str.find_first_not_of(" \t\r\n");
    if (start == std::string::npos) return "";
    size_t end = str.find_last_not_of(" \t\r\n");
    return str.substr(start, end - start + 1);
}

static std::string toLower(const std::string& str) {
    std::string result = str;
    std::transform(result.begin(), result.end(), result.begin(), ::tolower);
    return result;
}

PinRole ConfigLoader::parseRole(const std::string& roleStr) {
    std::string lower = toLower(trim(roleStr));
    if (lower == "input" || lower == "digital_input" || lower == "in") {
        return PinRole::DIGITAL_INPUT;
    } else if (lower == "output" || lower == "digital_output" || lower == "out") {
        return PinRole::DIGITAL_OUTPUT;
    } else if (lower == "vss" || lower == "vss_input" || lower == "wheel") {
        return PinRole::VSS_INPUT;
    } else if (lower == "pwm" || lower == "pwm_output") {
        return PinRole::PWM_OUTPUT;
    }
    return PinRole::UNUSED;
}

std::string ConfigLoader::roleToString(PinRole role) {
    switch (role) {
        case PinRole::DIGITAL_INPUT: return "input";
        case PinRole::DIGITAL_OUTPUT: return "output";
        case PinRole::VSS_INPUT: return "vss";
        case PinRole::PWM_OUTPUT: return "pwm";
        default: return "unused";
    }
}

void ConfigLoader::applyPreset(IOMode mode, Config& config) {
    config.pins.clear();
    config.ioMode = mode;

    int inputIdx = 0, outputIdx = 0, vssIdx = 0;

    switch (mode) {
        case IOMode::INPUTS_ONLY:
            // All 17 available GPIOs as inputs (we'll use 16)
            for (size_t i = 0; i < AVAILABLE_GPIOS.size() && inputIdx < 16; ++i) {
                PinConfig pin;
                pin.gpioNum = AVAILABLE_GPIOS[i];
                pin.role = PinRole::DIGITAL_INPUT;
                pin.channelIndex = inputIdx++;
                pin.name = "DIN" + std::to_string(pin.channelIndex);
                config.pins.push_back(pin);
            }
            break;

        case IOMode::BALANCED:
            // 8 inputs + 4 outputs + 4 VSS = 16 GPIOs
            // Inputs: GPIO 4, 5, 6, 12, 13, 14, 15, 16
            for (int gpio : {4, 5, 6, 12, 13, 14, 15, 16}) {
                PinConfig pin;
                pin.gpioNum = gpio;
                pin.role = PinRole::DIGITAL_INPUT;
                pin.channelIndex = inputIdx++;
                pin.name = "DIN" + std::to_string(pin.channelIndex);
                config.pins.push_back(pin);
            }
            // Outputs: GPIO 17, 18, 19, 20
            for (int gpio : {17, 18, 19, 20}) {
                PinConfig pin;
                pin.gpioNum = gpio;
                pin.role = PinRole::DIGITAL_OUTPUT;
                pin.channelIndex = outputIdx++;
                pin.name = "DOUT" + std::to_string(pin.channelIndex);
                config.pins.push_back(pin);
            }
            // VSS: GPIO 21, 22, 23, 26
            for (int gpio : {21, 22, 23, 26}) {
                PinConfig pin;
                pin.gpioNum = gpio;
                pin.role = PinRole::VSS_INPUT;
                pin.channelIndex = vssIdx++;
                pin.name = "VSS" + std::to_string(pin.channelIndex);
                config.pins.push_back(pin);
            }
            break;

        case IOMode::OUTPUTS_FOCUS:
            // 4 inputs + 8 outputs + 4 VSS = 16 GPIOs
            // Inputs: GPIO 4, 5, 6, 12
            for (int gpio : {4, 5, 6, 12}) {
                PinConfig pin;
                pin.gpioNum = gpio;
                pin.role = PinRole::DIGITAL_INPUT;
                pin.channelIndex = inputIdx++;
                pin.name = "DIN" + std::to_string(pin.channelIndex);
                config.pins.push_back(pin);
            }
            // Outputs: GPIO 13, 14, 15, 16, 17, 18, 19, 20
            for (int gpio : {13, 14, 15, 16, 17, 18, 19, 20}) {
                PinConfig pin;
                pin.gpioNum = gpio;
                pin.role = PinRole::DIGITAL_OUTPUT;
                pin.channelIndex = outputIdx++;
                pin.name = "DOUT" + std::to_string(pin.channelIndex);
                config.pins.push_back(pin);
            }
            // VSS: GPIO 21, 22, 23, 26
            for (int gpio : {21, 22, 23, 26}) {
                PinConfig pin;
                pin.gpioNum = gpio;
                pin.role = PinRole::VSS_INPUT;
                pin.channelIndex = vssIdx++;
                pin.name = "VSS" + std::to_string(pin.channelIndex);
                config.pins.push_back(pin);
            }
            break;

        case IOMode::VSS_FOCUS:
        default:
            // Same as balanced
            applyPreset(IOMode::BALANCED, config);
            config.ioMode = mode;
            break;
    }

    // Update counts
    config.numDigitalInputs = inputIdx;
    config.numDigitalOutputs = outputIdx;
    config.numVssInputs = vssIdx;
    config.numPwmOutputs = 0;
}

bool ConfigLoader::parseLine(const std::string& line, Config& config) {
    std::string trimmed = trim(line);

    // Skip empty lines and comments
    if (trimmed.empty() || trimmed[0] == '#' || trimmed[0] == ';') {
        return true;
    }

    // Find equals sign
    size_t eqPos = trimmed.find('=');
    if (eqPos == std::string::npos) {
        return false;
    }

    std::string key = toLower(trim(trimmed.substr(0, eqPos)));
    std::string value = trim(trimmed.substr(eqPos + 1));

    // Parse known keys
    if (key == "can_interface") {
        config.canInterface = value;
    } else if (key == "gps_device") {
        config.gpsDevice = value;
    } else if (key == "i2c_device") {
        config.i2cDevice = value;
    } else if (key == "gpio_chip") {
        config.gpioChip = value;
    } else if (key == "gps_baud") {
        config.gpsBaudRate = std::stoi(value);
    } else if (key == "io_mode") {
        std::string lower = toLower(value);
        if (lower == "inputs_only" || lower == "inputs") {
            config.ioMode = IOMode::INPUTS_ONLY;
        } else if (lower == "balanced" || lower == "mixed") {
            config.ioMode = IOMode::BALANCED;
        } else if (lower == "outputs_focus" || lower == "outputs") {
            config.ioMode = IOMode::OUTPUTS_FOCUS;
        } else if (lower == "custom") {
            config.ioMode = IOMode::CUSTOM;
        }
    } else if (key.substr(0, 4) == "gpio") {
        // Parse GPIO pin definition: gpio4 = input
        // or: gpio4 = input, DIN0
        try {
            int gpioNum = std::stoi(key.substr(4));

            // Check if there's a name after comma
            std::string roleStr = value;
            std::string name;
            size_t commaPos = value.find(',');
            if (commaPos != std::string::npos) {
                roleStr = trim(value.substr(0, commaPos));
                name = trim(value.substr(commaPos + 1));
            }

            PinConfig pin;
            pin.gpioNum = gpioNum;
            pin.role = parseRole(roleStr);
            pin.name = name;

            if (pin.role != PinRole::UNUSED) {
                config.pins.push_back(pin);
            }
        } catch (...) {
            return false;
        }
    }

    return true;
}

bool ConfigLoader::loadFromFile(const std::string& path, Config& config) {
    std::ifstream file(path);
    if (!file.is_open()) {
        std::fprintf(stderr, "Could not open config file: %s\n", path.c_str());
        return false;
    }

    config.pins.clear();

    std::string line;
    int lineNum = 0;
    while (std::getline(file, line)) {
        lineNum++;
        if (!parseLine(line, config)) {
            std::fprintf(stderr, "Config parse error at line %d: %s\n", lineNum, line.c_str());
        }
    }

    // If using preset mode and no custom pins defined, apply preset
    if (config.ioMode != IOMode::CUSTOM && config.pins.empty()) {
        applyPreset(config.ioMode, config);
    } else {
        // Assign channel indices for custom config
        int inputIdx = 0, outputIdx = 0, vssIdx = 0, pwmIdx = 0;
        for (auto& pin : config.pins) {
            switch (pin.role) {
                case PinRole::DIGITAL_INPUT:
                    pin.channelIndex = inputIdx++;
                    break;
                case PinRole::DIGITAL_OUTPUT:
                    pin.channelIndex = outputIdx++;
                    break;
                case PinRole::VSS_INPUT:
                    pin.channelIndex = vssIdx++;
                    break;
                case PinRole::PWM_OUTPUT:
                    pin.channelIndex = pwmIdx++;
                    break;
                default:
                    break;
            }
        }
        config.numDigitalInputs = inputIdx;
        config.numDigitalOutputs = outputIdx;
        config.numVssInputs = vssIdx;
        config.numPwmOutputs = pwmIdx;
    }

    std::printf("Loaded config: %zu GPIO pins configured\n", config.pins.size());
    std::printf("  Digital inputs:  %d\n", config.numDigitalInputs);
    std::printf("  Digital outputs: %d\n", config.numDigitalOutputs);
    std::printf("  VSS inputs:      %d\n", config.numVssInputs);
    std::printf("  PWM outputs:     %d\n", config.numPwmOutputs);

    return true;
}

bool ConfigLoader::saveToFile(const std::string& path, const Config& config) {
    std::ofstream file(path);
    if (!file.is_open()) {
        return false;
    }

    file << "# RPI_EPIC_CANBUS Configuration\n";
    file << "# Generated configuration file\n\n";

    file << "# Device paths\n";
    file << "can_interface = " << config.canInterface << "\n";
    file << "gps_device = " << config.gpsDevice << "\n";
    file << "i2c_device = " << config.i2cDevice << "\n";
    file << "gpio_chip = " << config.gpioChip << "\n";
    file << "gps_baud = " << config.gpsBaudRate << "\n\n";

    file << "# I/O Mode: inputs_only, balanced, outputs_focus, custom\n";
    switch (config.ioMode) {
        case IOMode::INPUTS_ONLY: file << "io_mode = inputs_only\n"; break;
        case IOMode::BALANCED: file << "io_mode = balanced\n"; break;
        case IOMode::OUTPUTS_FOCUS: file << "io_mode = outputs_focus\n"; break;
        default: file << "io_mode = custom\n"; break;
    }
    file << "\n";

    file << "# GPIO Pin Assignments (BCM numbering)\n";
    file << "# Available GPIOs: 4, 5, 6, 12, 13, 14, 15, 16, 17, 18, 19, 20, 21, 22, 23, 26, 27\n";
    file << "# Reserved: 2,3 (I2C), 7,8,9,10,11 (SPI), 24,25 (CAN INT)\n";
    file << "# Roles: input, output, vss, pwm, unused\n";
    file << "# Format: gpio<num> = <role>, <optional_name>\n\n";

    for (const auto& pin : config.pins) {
        file << "gpio" << pin.gpioNum << " = " << roleToString(pin.role);
        if (!pin.name.empty()) {
            file << ", " << pin.name;
        }
        file << "\n";
    }

    return true;
}

bool ConfigLoader::generateDefaultConfig(const std::string& path) {
    Config config;
    applyPreset(IOMode::BALANCED, config);
    return saveToFile(path, config);
}
