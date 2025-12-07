/*
 * GPIO Interface Implementation using libgpiod v2
 * Supports dynamic pin configuration
 */

#include "gpio_interface.h"
#include <cstdio>
#include <cstring>

GPIOInterface::GPIOInterface()
    : chip_(nullptr)
    , digitalInputRequest_(nullptr)
    , digitalOutputRequest_(nullptr)
    , vssInputRequest_(nullptr)
    , pwmOutputRequest_(nullptr)
    , numDigitalInputs_(0)
    , numDigitalOutputs_(0)
    , numVssInputs_(0)
    , numPwmOutputs_(0)
    , configured_(false) {
}

GPIOInterface::~GPIOInterface() {
    close();
}

bool GPIOInterface::init(const std::string& chipPath) {
    chipPath_ = chipPath;
    chip_ = gpiod_chip_open(chipPath.c_str());
    if (!chip_) {
        std::fprintf(stderr, "Failed to open GPIO chip '%s'\n", chipPath.c_str());
        return false;
    }

    std::printf("GPIO interface initialized: %s\n", chipPath.c_str());
    return true;
}

bool GPIOInterface::configureDigitalInputs(const std::vector<int>& pins) {
    if (!chip_) return false;

    // Release any existing request
    if (digitalInputRequest_) {
        gpiod_line_request_release(digitalInputRequest_);
        digitalInputRequest_ = nullptr;
    }
    digitalInputOffsets_.clear();
    numDigitalInputs_ = 0;

    if (pins.empty()) return true;

    // Build offsets array
    for (int gpio : pins) {
        if (gpio >= 0 && numDigitalInputs_ < MAX_DIGITAL_INPUTS) {
            digitalInputOffsets_.push_back(static_cast<unsigned int>(gpio));
            numDigitalInputs_++;
        }
    }

    if (digitalInputOffsets_.empty()) return true;

    // Create line settings for input with pull-up
    struct gpiod_line_settings* settings = gpiod_line_settings_new();
    if (!settings) {
        std::fprintf(stderr, "Failed to create line settings\n");
        return false;
    }

    gpiod_line_settings_set_direction(settings, GPIOD_LINE_DIRECTION_INPUT);
    gpiod_line_settings_set_bias(settings, GPIOD_LINE_BIAS_PULL_UP);

    // Create line config
    struct gpiod_line_config* lineConfig = gpiod_line_config_new();
    if (!lineConfig) {
        gpiod_line_settings_free(settings);
        std::fprintf(stderr, "Failed to create line config\n");
        return false;
    }

    int ret = gpiod_line_config_add_line_settings(lineConfig,
        digitalInputOffsets_.data(), digitalInputOffsets_.size(), settings);
    gpiod_line_settings_free(settings);

    if (ret < 0) {
        gpiod_line_config_free(lineConfig);
        std::fprintf(stderr, "Failed to add line settings\n");
        return false;
    }

    // Create request config
    struct gpiod_request_config* reqConfig = gpiod_request_config_new();
    if (reqConfig) {
        gpiod_request_config_set_consumer(reqConfig, "epic_din");
    }

    // Request the lines
    digitalInputRequest_ = gpiod_chip_request_lines(chip_, reqConfig, lineConfig);

    if (reqConfig) gpiod_request_config_free(reqConfig);
    gpiod_line_config_free(lineConfig);

    if (!digitalInputRequest_) {
        std::fprintf(stderr, "Failed to request digital input lines\n");
        return false;
    }

    for (size_t i = 0; i < digitalInputOffsets_.size(); ++i) {
        std::printf("  Digital input %zu: GPIO %u\n", i, digitalInputOffsets_[i]);
    }

    configured_ = true;
    return true;
}

bool GPIOInterface::configureDigitalOutputs(const std::vector<int>& pins) {
    if (!chip_) return false;

    // Release any existing request
    if (digitalOutputRequest_) {
        gpiod_line_request_release(digitalOutputRequest_);
        digitalOutputRequest_ = nullptr;
    }
    digitalOutputOffsets_.clear();
    numDigitalOutputs_ = 0;

    if (pins.empty()) return true;

    // Build offsets array
    for (int gpio : pins) {
        if (gpio >= 0 && numDigitalOutputs_ < MAX_DIGITAL_OUTPUTS) {
            digitalOutputOffsets_.push_back(static_cast<unsigned int>(gpio));
            numDigitalOutputs_++;
        }
    }

    if (digitalOutputOffsets_.empty()) return true;

    // Create line settings for output
    struct gpiod_line_settings* settings = gpiod_line_settings_new();
    if (!settings) return false;

    gpiod_line_settings_set_direction(settings, GPIOD_LINE_DIRECTION_OUTPUT);
    gpiod_line_settings_set_output_value(settings, GPIOD_LINE_VALUE_INACTIVE);

    // Create line config
    struct gpiod_line_config* lineConfig = gpiod_line_config_new();
    if (!lineConfig) {
        gpiod_line_settings_free(settings);
        return false;
    }

    gpiod_line_config_add_line_settings(lineConfig,
        digitalOutputOffsets_.data(), digitalOutputOffsets_.size(), settings);
    gpiod_line_settings_free(settings);

    // Create request config
    struct gpiod_request_config* reqConfig = gpiod_request_config_new();
    if (reqConfig) {
        gpiod_request_config_set_consumer(reqConfig, "epic_dout");
    }

    // Request the lines
    digitalOutputRequest_ = gpiod_chip_request_lines(chip_, reqConfig, lineConfig);

    if (reqConfig) gpiod_request_config_free(reqConfig);
    gpiod_line_config_free(lineConfig);

    if (!digitalOutputRequest_) {
        std::fprintf(stderr, "Failed to request digital output lines\n");
        return false;
    }

    for (size_t i = 0; i < digitalOutputOffsets_.size(); ++i) {
        std::printf("  Digital output %zu: GPIO %u\n", i, digitalOutputOffsets_[i]);
    }

    return true;
}

bool GPIOInterface::configureVssInputs(const std::vector<int>& pins) {
    if (!chip_) return false;

    // Release any existing request
    if (vssInputRequest_) {
        gpiod_line_request_release(vssInputRequest_);
        vssInputRequest_ = nullptr;
    }
    vssInputOffsets_.clear();
    vssEdgeCounts_.clear();
    vssLastState_.clear();
    numVssInputs_ = 0;

    if (pins.empty()) return true;

    // Build offsets array
    for (int gpio : pins) {
        if (gpio >= 0 && numVssInputs_ < MAX_VSS_INPUTS) {
            vssInputOffsets_.push_back(static_cast<unsigned int>(gpio));
            vssEdgeCounts_.push_back(0);
            vssLastState_.push_back(1);  // Assume HIGH initially
            numVssInputs_++;
        }
    }

    if (vssInputOffsets_.empty()) return true;

    // Create line settings for input with pull-up
    struct gpiod_line_settings* settings = gpiod_line_settings_new();
    if (!settings) return false;

    gpiod_line_settings_set_direction(settings, GPIOD_LINE_DIRECTION_INPUT);
    gpiod_line_settings_set_bias(settings, GPIOD_LINE_BIAS_PULL_UP);

    // Create line config
    struct gpiod_line_config* lineConfig = gpiod_line_config_new();
    if (!lineConfig) {
        gpiod_line_settings_free(settings);
        return false;
    }

    gpiod_line_config_add_line_settings(lineConfig,
        vssInputOffsets_.data(), vssInputOffsets_.size(), settings);
    gpiod_line_settings_free(settings);

    // Create request config
    struct gpiod_request_config* reqConfig = gpiod_request_config_new();
    if (reqConfig) {
        gpiod_request_config_set_consumer(reqConfig, "epic_vss");
    }

    // Request the lines
    vssInputRequest_ = gpiod_chip_request_lines(chip_, reqConfig, lineConfig);

    if (reqConfig) gpiod_request_config_free(reqConfig);
    gpiod_line_config_free(lineConfig);

    if (!vssInputRequest_) {
        std::fprintf(stderr, "Failed to request VSS input lines\n");
        return false;
    }

    for (size_t i = 0; i < vssInputOffsets_.size(); ++i) {
        std::printf("  VSS input %zu: GPIO %u\n", i, vssInputOffsets_[i]);
    }

    return true;
}

bool GPIOInterface::configurePwmOutputs(const std::vector<int>& pins) {
    if (!chip_) return false;

    // Release any existing request
    if (pwmOutputRequest_) {
        gpiod_line_request_release(pwmOutputRequest_);
        pwmOutputRequest_ = nullptr;
    }
    pwmOutputOffsets_.clear();
    numPwmOutputs_ = 0;

    if (pins.empty()) return true;

    // Build offsets array
    for (int gpio : pins) {
        if (gpio >= 0 && numPwmOutputs_ < MAX_PWM_OUTPUTS) {
            pwmOutputOffsets_.push_back(static_cast<unsigned int>(gpio));
            numPwmOutputs_++;
        }
    }

    if (pwmOutputOffsets_.empty()) return true;

    // Create line settings for output
    struct gpiod_line_settings* settings = gpiod_line_settings_new();
    if (!settings) return false;

    gpiod_line_settings_set_direction(settings, GPIOD_LINE_DIRECTION_OUTPUT);
    gpiod_line_settings_set_output_value(settings, GPIOD_LINE_VALUE_INACTIVE);

    // Create line config
    struct gpiod_line_config* lineConfig = gpiod_line_config_new();
    if (!lineConfig) {
        gpiod_line_settings_free(settings);
        return false;
    }

    gpiod_line_config_add_line_settings(lineConfig,
        pwmOutputOffsets_.data(), pwmOutputOffsets_.size(), settings);
    gpiod_line_settings_free(settings);

    // Create request config
    struct gpiod_request_config* reqConfig = gpiod_request_config_new();
    if (reqConfig) {
        gpiod_request_config_set_consumer(reqConfig, "epic_pwm");
    }

    // Request the lines
    pwmOutputRequest_ = gpiod_chip_request_lines(chip_, reqConfig, lineConfig);

    if (reqConfig) gpiod_request_config_free(reqConfig);
    gpiod_line_config_free(lineConfig);

    if (!pwmOutputRequest_) {
        std::fprintf(stderr, "Failed to request PWM output lines\n");
        return false;
    }

    for (size_t i = 0; i < pwmOutputOffsets_.size(); ++i) {
        std::printf("  PWM output %zu: GPIO %u\n", i, pwmOutputOffsets_[i]);
    }

    return true;
}

void GPIOInterface::close() {
    if (digitalInputRequest_) {
        gpiod_line_request_release(digitalInputRequest_);
        digitalInputRequest_ = nullptr;
    }
    if (digitalOutputRequest_) {
        gpiod_line_request_release(digitalOutputRequest_);
        digitalOutputRequest_ = nullptr;
    }
    if (vssInputRequest_) {
        gpiod_line_request_release(vssInputRequest_);
        vssInputRequest_ = nullptr;
    }
    if (pwmOutputRequest_) {
        gpiod_line_request_release(pwmOutputRequest_);
        pwmOutputRequest_ = nullptr;
    }

    digitalInputOffsets_.clear();
    digitalOutputOffsets_.clear();
    vssInputOffsets_.clear();
    pwmOutputOffsets_.clear();

    if (chip_) {
        gpiod_chip_close(chip_);
        chip_ = nullptr;
    }

    numDigitalInputs_ = 0;
    numDigitalOutputs_ = 0;
    numVssInputs_ = 0;
    numPwmOutputs_ = 0;
    configured_ = false;
}

uint16_t GPIOInterface::readDigitalInputs() {
    if (!digitalInputRequest_ || digitalInputOffsets_.empty()) return 0;

    uint16_t bits = 0;

    // Read all values at once
    enum gpiod_line_value values[MAX_DIGITAL_INPUTS];
    int ret = gpiod_line_request_get_values(digitalInputRequest_, values);

    if (ret < 0) return 0;

    for (int i = 0; i < numDigitalInputs_ && i < 16; ++i) {
        // Invert: LOW = 1 (grounded button pressed), HIGH = 0
        if (values[i] == GPIOD_LINE_VALUE_INACTIVE) {
            bits |= (1u << i);
        }
    }

    return bits;
}

void GPIOInterface::setDigitalOutputs(uint16_t bits) {
    if (!digitalOutputRequest_ || digitalOutputOffsets_.empty()) return;

    enum gpiod_line_value values[MAX_DIGITAL_OUTPUTS];

    for (int i = 0; i < numDigitalOutputs_ && i < 16; ++i) {
        values[i] = (bits & (1u << i)) ? GPIOD_LINE_VALUE_ACTIVE : GPIOD_LINE_VALUE_INACTIVE;
    }

    gpiod_line_request_set_values(digitalOutputRequest_, values);
}

void GPIOInterface::setPwmOutputs(uint16_t bits) {
    if (!pwmOutputRequest_ || pwmOutputOffsets_.empty()) return;

    enum gpiod_line_value values[MAX_PWM_OUTPUTS];

    for (int i = 0; i < numPwmOutputs_ && i < 16; ++i) {
        values[i] = (bits & (1u << i)) ? GPIOD_LINE_VALUE_ACTIVE : GPIOD_LINE_VALUE_INACTIVE;
    }

    gpiod_line_request_set_values(pwmOutputRequest_, values);
}

void GPIOInterface::pollVSS() {
    if (!vssInputRequest_ || vssInputOffsets_.empty()) return;

    enum gpiod_line_value values[MAX_VSS_INPUTS];
    int ret = gpiod_line_request_get_values(vssInputRequest_, values);

    if (ret < 0) return;

    // Detect falling edges
    for (int i = 0; i < numVssInputs_; ++i) {
        int currentState = (values[i] == GPIOD_LINE_VALUE_ACTIVE) ? 1 : 0;
        // Detect falling edge (HIGH -> LOW)
        if (vssLastState_[i] == 1 && currentState == 0) {
            if (vssEdgeCounts_[i] < 0xFFFFFFFE) {
                vssEdgeCounts_[i]++;
            }
        }
        vssLastState_[i] = currentState;
    }
}

void GPIOInterface::getVSSCounts(uint32_t* counts) {
    if (!counts) return;

    for (int i = 0; i < numVssInputs_ && i < MAX_VSS_INPUTS; ++i) {
        counts[i] = vssEdgeCounts_[i];
    }
}
