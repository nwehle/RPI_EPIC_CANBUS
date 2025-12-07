/*
 * GPIO Interface Implementation using libgpiod
 * Supports dynamic pin configuration
 */

#include "gpio_interface.h"
#include <cstdio>
#include <cstring>

GPIOInterface::GPIOInterface()
    : chip_(nullptr)
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

    // Release any existing lines
    for (auto& line : digitalInputLines_) {
        if (line) {
            gpiod_line_release(line);
        }
    }
    digitalInputLines_.clear();

    numDigitalInputs_ = 0;
    for (int gpio : pins) {
        if (gpio < 0) continue;
        if (numDigitalInputs_ >= MAX_DIGITAL_INPUTS) break;

        struct gpiod_line* line = gpiod_chip_get_line(chip_, gpio);
        if (line) {
            if (gpiod_line_request_input_flags(line, "epic_din",
                    GPIOD_LINE_REQUEST_FLAG_BIAS_PULL_UP) < 0) {
                std::fprintf(stderr, "Failed to configure digital input GPIO %d\n", gpio);
                digitalInputLines_.push_back(nullptr);
            } else {
                digitalInputLines_.push_back(line);
                std::printf("  Digital input %d: GPIO %d\n", numDigitalInputs_, gpio);
            }
        } else {
            digitalInputLines_.push_back(nullptr);
        }
        numDigitalInputs_++;
    }

    configured_ = true;
    return true;
}

bool GPIOInterface::configureDigitalOutputs(const std::vector<int>& pins) {
    if (!chip_) return false;

    // Release any existing lines
    for (auto& line : digitalOutputLines_) {
        if (line) {
            gpiod_line_release(line);
        }
    }
    digitalOutputLines_.clear();

    numDigitalOutputs_ = 0;
    for (int gpio : pins) {
        if (gpio < 0) continue;
        if (numDigitalOutputs_ >= MAX_DIGITAL_OUTPUTS) break;

        struct gpiod_line* line = gpiod_chip_get_line(chip_, gpio);
        if (line) {
            if (gpiod_line_request_output(line, "epic_dout", 0) < 0) {
                std::fprintf(stderr, "Failed to configure digital output GPIO %d\n", gpio);
                digitalOutputLines_.push_back(nullptr);
            } else {
                digitalOutputLines_.push_back(line);
                std::printf("  Digital output %d: GPIO %d\n", numDigitalOutputs_, gpio);
            }
        } else {
            digitalOutputLines_.push_back(nullptr);
        }
        numDigitalOutputs_++;
    }

    return true;
}

bool GPIOInterface::configureVssInputs(const std::vector<int>& pins) {
    if (!chip_) return false;

    // Release any existing lines
    for (auto& line : vssInputLines_) {
        if (line) {
            gpiod_line_release(line);
        }
    }
    vssInputLines_.clear();
    vssEdgeCounts_.clear();
    vssLastState_.clear();

    numVssInputs_ = 0;
    for (int gpio : pins) {
        if (gpio < 0) continue;
        if (numVssInputs_ >= MAX_VSS_INPUTS) break;

        struct gpiod_line* line = gpiod_chip_get_line(chip_, gpio);
        if (line) {
            if (gpiod_line_request_input_flags(line, "epic_vss",
                    GPIOD_LINE_REQUEST_FLAG_BIAS_PULL_UP) < 0) {
                std::fprintf(stderr, "Failed to configure VSS input GPIO %d\n", gpio);
                vssInputLines_.push_back(nullptr);
            } else {
                vssInputLines_.push_back(line);
                std::printf("  VSS input %d: GPIO %d\n", numVssInputs_, gpio);
            }
        } else {
            vssInputLines_.push_back(nullptr);
        }
        vssEdgeCounts_.push_back(0);
        vssLastState_.push_back(1);  // Assume HIGH initially
        numVssInputs_++;
    }

    return true;
}

bool GPIOInterface::configurePwmOutputs(const std::vector<int>& pins) {
    if (!chip_) return false;

    // Release any existing lines
    for (auto& line : pwmOutputLines_) {
        if (line) {
            gpiod_line_release(line);
        }
    }
    pwmOutputLines_.clear();

    numPwmOutputs_ = 0;
    for (int gpio : pins) {
        if (gpio < 0) continue;
        if (numPwmOutputs_ >= MAX_PWM_OUTPUTS) break;

        struct gpiod_line* line = gpiod_chip_get_line(chip_, gpio);
        if (line) {
            if (gpiod_line_request_output(line, "epic_pwm", 0) < 0) {
                std::fprintf(stderr, "Failed to configure PWM output GPIO %d\n", gpio);
                pwmOutputLines_.push_back(nullptr);
            } else {
                pwmOutputLines_.push_back(line);
                std::printf("  PWM output %d: GPIO %d\n", numPwmOutputs_, gpio);
            }
        } else {
            pwmOutputLines_.push_back(nullptr);
        }
        numPwmOutputs_++;
    }

    return true;
}

void GPIOInterface::close() {
    // Release all lines
    for (auto& line : digitalInputLines_) {
        if (line) {
            gpiod_line_release(line);
            line = nullptr;
        }
    }
    digitalInputLines_.clear();

    for (auto& line : digitalOutputLines_) {
        if (line) {
            gpiod_line_release(line);
            line = nullptr;
        }
    }
    digitalOutputLines_.clear();

    for (auto& line : pwmOutputLines_) {
        if (line) {
            gpiod_line_release(line);
            line = nullptr;
        }
    }
    pwmOutputLines_.clear();

    for (auto& line : vssInputLines_) {
        if (line) {
            gpiod_line_release(line);
            line = nullptr;
        }
    }
    vssInputLines_.clear();

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
    if (!configured_) return 0;

    uint16_t bits = 0;
    for (int i = 0; i < numDigitalInputs_ && i < 16; ++i) {
        if (i < static_cast<int>(digitalInputLines_.size()) && digitalInputLines_[i]) {
            int val = gpiod_line_get_value(digitalInputLines_[i]);
            // Invert: LOW = 1 (grounded button pressed), HIGH = 0
            if (val == 0) {
                bits |= (1u << i);
            }
        }
    }
    return bits;
}

void GPIOInterface::setDigitalOutputs(uint16_t bits) {
    if (!configured_) return;

    for (int i = 0; i < numDigitalOutputs_ && i < 16; ++i) {
        if (i < static_cast<int>(digitalOutputLines_.size()) && digitalOutputLines_[i]) {
            int value = (bits & (1u << i)) ? 1 : 0;
            gpiod_line_set_value(digitalOutputLines_[i], value);
        }
    }
}

void GPIOInterface::setPwmOutputs(uint16_t bits) {
    if (!configured_) return;

    for (int i = 0; i < numPwmOutputs_ && i < 16; ++i) {
        if (i < static_cast<int>(pwmOutputLines_.size()) && pwmOutputLines_[i]) {
            int value = (bits & (1u << i)) ? 1 : 0;
            gpiod_line_set_value(pwmOutputLines_[i], value);
        }
    }
}

void GPIOInterface::pollVSS() {
    if (!configured_) return;

    // Poll each VSS input and detect falling edges
    for (int i = 0; i < numVssInputs_; ++i) {
        if (i < static_cast<int>(vssInputLines_.size()) && vssInputLines_[i]) {
            int currentState = gpiod_line_get_value(vssInputLines_[i]);
            // Detect falling edge (HIGH -> LOW)
            if (i < static_cast<int>(vssLastState_.size()) &&
                vssLastState_[i] == 1 && currentState == 0) {
                if (i < static_cast<int>(vssEdgeCounts_.size()) &&
                    vssEdgeCounts_[i] < 0xFFFFFFFE) {
                    vssEdgeCounts_[i]++;
                }
            }
            if (i < static_cast<int>(vssLastState_.size())) {
                vssLastState_[i] = currentState;
            }
        }
    }
}

void GPIOInterface::getVSSCounts(uint32_t* counts) {
    if (!counts) return;

    for (int i = 0; i < numVssInputs_ && i < MAX_VSS_INPUTS; ++i) {
        if (i < static_cast<int>(vssEdgeCounts_.size())) {
            counts[i] = vssEdgeCounts_[i];
        } else {
            counts[i] = 0;
        }
    }
}
