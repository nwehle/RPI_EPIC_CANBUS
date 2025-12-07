/*
 * RPI_EPIC_CANBUS - Raspberry Pi CAN I/O Expander for epicEFI
 * ----------------------------------------------------------------
 * Port of MEGA_EPIC_CANBUS Arduino firmware for Raspberry Pi 5
 *
 * Hardware:
 *   - Raspberry Pi 5
 *   - Waveshare 2-channel isolated CAN bus expansion HAT
 *   - Adafruit Ultimate GPS with USB
 *   - CQRobot ADS1115 16-bit ADC (up to 4 modules for 16 channels)
 *
 * Author: Gennady Gurov (ported to RPi)
 * Made for: epicEFI project
 */

#include "epic_protocol.h"
#include "can_interface.h"
#include "gps_interface.h"
#include "adc_interface.h"
#include "gpio_interface.h"
#include "nmea_parser.h"
#include "config.h"

#include <cstdio>
#include <cstdlib>
#include <cstring>
#include <cmath>
#include <csignal>
#include <chrono>
#include <thread>
#include <atomic>
#include <getopt.h>

// Global flag for graceful shutdown
static std::atomic<bool> running(true);

// Global configuration
static Config config;

// Signal handler
void signalHandler(int sig) {
    (void)sig;
    running = false;
}

// Get current time in milliseconds
static uint64_t millis() {
    using namespace std::chrono;
    return duration_cast<milliseconds>(steady_clock::now().time_since_epoch()).count();
}

// ============== Smart Transmission State ==============

static TxChannelState analogTxState[16];
static TxChannelState digitalTxState;
static TxChannelState vssTxState[4];
static TxChannelState gpsTxState[8];

// Current input values
static float currentAnalogValues[16] = {0};
static uint16_t currentDigitalBits = 0;
static float currentVssValues[4] = {0};

// VSS calculation state
struct VSSChannel {
    uint32_t lastCount = 0;
    uint64_t lastCalcTime = 0;
    float pulsesPerSecond = 0.0f;
};
static VSSChannel vssChannels[4];

// GPS state
static GPSData lastTransmittedGpsData = {};

// ============== Smart Transmission Functions ==============

static bool hasAnalogChanged(int channel, float newValue) {
    float diff = newValue - analogTxState[channel].lastTransmittedValue;
    if (diff < 0) diff = -diff;
    return diff >= TX_ANALOG_THRESHOLD;
}

static bool hasDigitalChanged(uint16_t newBits) {
    return newBits != static_cast<uint16_t>(digitalTxState.lastTransmittedValue);
}

static bool hasVSSChanged(int channel, float newValue) {
    float diff = newValue - vssTxState[channel].lastTransmittedValue;
    if (diff < 0) diff = -diff;
    return diff >= TX_VSS_THRESHOLD;
}

static void updateTxState(TxChannelState* state, bool changed, uint64_t nowMs) {
    if (changed) {
        state->hasChanged = true;
        state->state = TX_STATE_CHANGED;
    } else {
        state->hasChanged = false;
        if (state->state == TX_STATE_CHANGED &&
            (nowMs - state->lastTxTimeMs) >= TX_INTERVAL_FAST_MS) {
            state->state = TX_STATE_STABLE;
        }
    }
}

static bool shouldTransmit(TxChannelState* state, uint64_t nowMs) {
    if (state->hasChanged) {
        return (nowMs - state->lastTxTimeMs) >= TX_INTERVAL_FAST_MS;
    } else {
        return (nowMs - state->lastTxTimeMs) >= TX_INTERVAL_SLOW_MS;
    }
}

static void transmitIfNeeded(CANInterface& can, TxChannelState* state,
                              int32_t varHash, float value, uint64_t nowMs) {
    if (shouldTransmit(state, nowMs)) {
        can.sendVariableSetFrame(varHash, value);
        state->lastTransmittedValue = value;
        state->lastTxTimeMs = nowMs;
        state->hasChanged = false;
        if (state->state == TX_STATE_CHANGED) {
            state->state = TX_STATE_STABLE;
        }
    }
}

// ============== VSS Rate Calculation ==============

static void calculateVSSRates(GPIOInterface& gpio, int numVss) {
    static uint64_t lastCalcTime = 0;
    uint64_t now = millis();

    uint64_t timeDelta = now - lastCalcTime;
    if (timeDelta >= VSS_CALC_INTERVAL_MS && timeDelta < 1000) {
        float timeDeltaSeconds = timeDelta / 1000.0f;

        uint32_t counts[4];
        gpio.getVSSCounts(counts);

        for (int i = 0; i < numVss && i < 4; ++i) {
            uint32_t countDelta = counts[i] - vssChannels[i].lastCount;
            vssChannels[i].pulsesPerSecond = countDelta / timeDeltaSeconds;
            vssChannels[i].lastCount = counts[i];
            currentVssValues[i] = vssChannels[i].pulsesPerSecond;
        }
        lastCalcTime = now;
    } else if (timeDelta >= 1000) {
        // Reset on overflow
        lastCalcTime = now;
        uint32_t counts[4];
        gpio.getVSSCounts(counts);
        for (int i = 0; i < numVss && i < 4; ++i) {
            vssChannels[i].lastCount = counts[i];
            vssChannels[i].pulsesPerSecond = 0.0f;
            currentVssValues[i] = 0.0f;
        }
    }
}

// ============== GPS Transmission ==============

static bool hasGPSChanged(int gpsVarIndex, const GPSData& gpsData) {
    switch (gpsVarIndex) {
        case 0: {  // packed_hmsd
            uint32_t current = packGPSHMSD(gpsData.hours, gpsData.minutes,
                                            gpsData.seconds, gpsData.days);
            uint32_t last = packGPSHMSD(lastTransmittedGpsData.hours,
                                         lastTransmittedGpsData.minutes,
                                         lastTransmittedGpsData.seconds,
                                         lastTransmittedGpsData.days);
            return current != last;
        }
        case 1: {  // packed_myqsat
            uint32_t current = packGPSMYQSAT(gpsData.months, gpsData.years,
                                              gpsData.quality, gpsData.satellites);
            uint32_t last = packGPSMYQSAT(lastTransmittedGpsData.months,
                                           lastTransmittedGpsData.years,
                                           lastTransmittedGpsData.quality,
                                           lastTransmittedGpsData.satellites);
            return current != last;
        }
        case 2:
            return std::fabs(gpsData.accuracy - lastTransmittedGpsData.accuracy) > GPS_FLOAT_THRESHOLD;
        case 3:
            return std::fabs(gpsData.altitude - lastTransmittedGpsData.altitude) > GPS_FLOAT_THRESHOLD;
        case 4:
            return std::fabs(gpsData.course - lastTransmittedGpsData.course) > GPS_FLOAT_THRESHOLD;
        case 5:
            return std::fabs(gpsData.latitude - lastTransmittedGpsData.latitude) > GPS_FLOAT_THRESHOLD;
        case 6:
            return std::fabs(gpsData.longitude - lastTransmittedGpsData.longitude) > GPS_FLOAT_THRESHOLD;
        case 7:
            return std::fabs(gpsData.speed - lastTransmittedGpsData.speed) > GPS_FLOAT_THRESHOLD;
        default:
            return false;
    }
}

static void transmitGPSIfNeeded(CANInterface& can, GPSInterface& gps) {
    if (!gps.isEnabled() || !gps.getData().dataValid) {
        return;
    }

    const GPSData& gpsData = gps.getData();
    uint64_t nowMs = millis();

    // HMSD packed
    bool changed0 = hasGPSChanged(0, gpsData);
    updateTxState(&gpsTxState[0], changed0, nowMs);
    if (shouldTransmit(&gpsTxState[0], nowMs)) {
        uint32_t value = packGPSHMSD(gpsData.hours, gpsData.minutes,
                                      gpsData.seconds, gpsData.days);
        can.sendVariableSetFrameU32(VAR_HASH_GPS_HMSD_PACKED, value);
        gpsTxState[0].lastTransmittedValue = static_cast<float>(value);
        gpsTxState[0].lastTxTimeMs = nowMs;
        lastTransmittedGpsData.hours = gpsData.hours;
        lastTransmittedGpsData.minutes = gpsData.minutes;
        lastTransmittedGpsData.seconds = gpsData.seconds;
        lastTransmittedGpsData.days = gpsData.days;
    }

    // MYQSAT packed
    bool changed1 = hasGPSChanged(1, gpsData);
    updateTxState(&gpsTxState[1], changed1, nowMs);
    if (shouldTransmit(&gpsTxState[1], nowMs)) {
        uint32_t value = packGPSMYQSAT(gpsData.months, gpsData.years,
                                        gpsData.quality, gpsData.satellites);
        can.sendVariableSetFrameU32(VAR_HASH_GPS_MYQSAT_PACKED, value);
        gpsTxState[1].lastTransmittedValue = static_cast<float>(value);
        gpsTxState[1].lastTxTimeMs = nowMs;
        lastTransmittedGpsData.months = gpsData.months;
        lastTransmittedGpsData.years = gpsData.years;
        lastTransmittedGpsData.quality = gpsData.quality;
        lastTransmittedGpsData.satellites = gpsData.satellites;
    }

    // Only transmit position data if we have a fix
    if (gpsData.hasFix) {
        // Accuracy
        bool changed2 = hasGPSChanged(2, gpsData);
        updateTxState(&gpsTxState[2], changed2, nowMs);
        if (shouldTransmit(&gpsTxState[2], nowMs)) {
            can.sendVariableSetFrame(VAR_HASH_GPS_ACCURACY, gpsData.accuracy);
            gpsTxState[2].lastTransmittedValue = gpsData.accuracy;
            gpsTxState[2].lastTxTimeMs = nowMs;
            lastTransmittedGpsData.accuracy = gpsData.accuracy;
        }

        // Altitude
        bool changed3 = hasGPSChanged(3, gpsData);
        updateTxState(&gpsTxState[3], changed3, nowMs);
        if (shouldTransmit(&gpsTxState[3], nowMs)) {
            can.sendVariableSetFrame(VAR_HASH_GPS_ALTITUDE, gpsData.altitude);
            gpsTxState[3].lastTransmittedValue = gpsData.altitude;
            gpsTxState[3].lastTxTimeMs = nowMs;
            lastTransmittedGpsData.altitude = gpsData.altitude;
        }

        // Course
        bool changed4 = hasGPSChanged(4, gpsData);
        updateTxState(&gpsTxState[4], changed4, nowMs);
        if (shouldTransmit(&gpsTxState[4], nowMs)) {
            can.sendVariableSetFrame(VAR_HASH_GPS_COURSE, gpsData.course);
            gpsTxState[4].lastTransmittedValue = gpsData.course;
            gpsTxState[4].lastTxTimeMs = nowMs;
            lastTransmittedGpsData.course = gpsData.course;
        }

        // Latitude
        bool changed5 = hasGPSChanged(5, gpsData);
        updateTxState(&gpsTxState[5], changed5, nowMs);
        if (shouldTransmit(&gpsTxState[5], nowMs)) {
            can.sendVariableSetFrame(VAR_HASH_GPS_LATITUDE, gpsData.latitude);
            gpsTxState[5].lastTransmittedValue = gpsData.latitude;
            gpsTxState[5].lastTxTimeMs = nowMs;
            lastTransmittedGpsData.latitude = gpsData.latitude;
        }

        // Longitude
        bool changed6 = hasGPSChanged(6, gpsData);
        updateTxState(&gpsTxState[6], changed6, nowMs);
        if (shouldTransmit(&gpsTxState[6], nowMs)) {
            can.sendVariableSetFrame(VAR_HASH_GPS_LONGITUDE, gpsData.longitude);
            gpsTxState[6].lastTransmittedValue = gpsData.longitude;
            gpsTxState[6].lastTxTimeMs = nowMs;
            lastTransmittedGpsData.longitude = gpsData.longitude;
        }

        // Speed
        bool changed7 = hasGPSChanged(7, gpsData);
        updateTxState(&gpsTxState[7], changed7, nowMs);
        if (shouldTransmit(&gpsTxState[7], nowMs)) {
            can.sendVariableSetFrame(VAR_HASH_GPS_SPEED, gpsData.speed);
            gpsTxState[7].lastTransmittedValue = gpsData.speed;
            gpsTxState[7].lastTxTimeMs = nowMs;
            lastTransmittedGpsData.speed = gpsData.speed;
        }
    }
}

// ============== CAN Frame Handler ==============

static void handleCanFrame(GPIOInterface& gpio, const struct can_frame& frame) {
    if (frame.can_id == CAN_ID_VAR_RESPONSE && frame.can_dlc == 8) {
        int32_t hash = CANInterface::readInt32BigEndian(frame.data);
        if (hash == VAR_HASH_OUT_SLOW) {
            float value = CANInterface::readFloat32BigEndian(&frame.data[4]);
            uint32_t rawBits = (value >= 0.0f) ? static_cast<uint32_t>(value + 0.5f) : 0u;

            // Extract output bits (0-7 for digital outputs)
            uint8_t outputBits = static_cast<uint8_t>(rawBits & 0xFF);
            gpio.setDigitalOutputs(outputBits);

            // Extract PWM bits (8-17)
            uint16_t pwmBits = static_cast<uint16_t>((rawBits >> 8) & 0x3FF);
            gpio.setPwmOutputs(pwmBits);
        }
    }
}

// ============== Print Usage ==============

static void printUsage(const char* progName) {
    std::printf("Usage: %s [options]\n", progName);
    std::printf("\nOptions:\n");
    std::printf("  -f, --config <file>     Config file (default: epic.conf)\n");
    std::printf("  -c, --can <interface>   CAN interface (default: can0)\n");
    std::printf("  -g, --gps <device>      GPS serial device (default: /dev/ttyUSB0)\n");
    std::printf("  -i, --i2c <device>      I2C device for ADC (default: /dev/i2c-1)\n");
    std::printf("  -p, --gpio <chip>       GPIO chip (default: /dev/gpiochip4)\n");
    std::printf("  -m, --mode <mode>       I/O mode: inputs, balanced, outputs, custom\n");
    std::printf("  --generate-config       Generate default config file and exit\n");
    std::printf("  -h, --help              Show this help message\n");
    std::printf("\nI/O Modes:\n");
    std::printf("  inputs   - 16 digital inputs (no outputs or VSS)\n");
    std::printf("  balanced - 8 inputs + 4 outputs + 4 VSS (default)\n");
    std::printf("  outputs  - 4 inputs + 8 outputs + 4 VSS\n");
    std::printf("  custom   - Use gpio assignments from config file\n");
    std::printf("\nExample:\n");
    std::printf("  %s -c can0 -g /dev/ttyACM0 -m balanced\n", progName);
    std::printf("  %s -f /etc/epic.conf\n", progName);
}

// ============== Main ==============

int main(int argc, char* argv[]) {
    std::string configFile = "epic.conf";
    bool generateConfig = false;
    IOMode cmdLineMode = IOMode::BALANCED;
    bool modeSpecified = false;

    // Parse command line arguments
    static struct option longOptions[] = {
        {"config",          required_argument, nullptr, 'f'},
        {"can",             required_argument, nullptr, 'c'},
        {"gps",             required_argument, nullptr, 'g'},
        {"i2c",             required_argument, nullptr, 'i'},
        {"gpio",            required_argument, nullptr, 'p'},
        {"mode",            required_argument, nullptr, 'm'},
        {"generate-config", no_argument,       nullptr, 'G'},
        {"help",            no_argument,       nullptr, 'h'},
        {nullptr, 0, nullptr, 0}
    };

    int opt;
    while ((opt = getopt_long(argc, argv, "f:c:g:i:p:m:h", longOptions, nullptr)) != -1) {
        switch (opt) {
            case 'f': configFile = optarg; break;
            case 'c': config.canInterface = optarg; break;
            case 'g': config.gpsDevice = optarg; break;
            case 'i': config.i2cDevice = optarg; break;
            case 'p': config.gpioChip = optarg; break;
            case 'm': {
                std::string mode = optarg;
                modeSpecified = true;
                if (mode == "inputs" || mode == "inputs_only") {
                    cmdLineMode = IOMode::INPUTS_ONLY;
                } else if (mode == "balanced" || mode == "mixed") {
                    cmdLineMode = IOMode::BALANCED;
                } else if (mode == "outputs" || mode == "outputs_focus") {
                    cmdLineMode = IOMode::OUTPUTS_FOCUS;
                } else if (mode == "custom") {
                    cmdLineMode = IOMode::CUSTOM;
                } else {
                    std::fprintf(stderr, "Unknown mode: %s\n", optarg);
                    return 1;
                }
                break;
            }
            case 'G':
                generateConfig = true;
                break;
            case 'h':
                printUsage(argv[0]);
                return 0;
            default:
                printUsage(argv[0]);
                return 1;
        }
    }

    // Generate config and exit if requested
    if (generateConfig) {
        ConfigLoader::applyPreset(cmdLineMode, config);
        if (ConfigLoader::saveToFile(configFile, config)) {
            std::printf("Generated config file: %s\n", configFile.c_str());
            return 0;
        } else {
            std::fprintf(stderr, "Failed to generate config file\n");
            return 1;
        }
    }

    // Setup signal handler for graceful shutdown
    std::signal(SIGINT, signalHandler);
    std::signal(SIGTERM, signalHandler);

    std::printf("RPI_EPIC_CANBUS - epicEFI CAN I/O Expander\n");
    std::printf("==========================================\n\n");

    // Try to load config file
    if (!ConfigLoader::loadFromFile(configFile, config)) {
        std::printf("Config file not found, using defaults\n");
        if (modeSpecified) {
            ConfigLoader::applyPreset(cmdLineMode, config);
        } else {
            ConfigLoader::applyPreset(IOMode::BALANCED, config);
        }
    } else if (modeSpecified) {
        // Command line mode overrides config file
        ConfigLoader::applyPreset(cmdLineMode, config);
    }

    std::printf("\nConfiguration:\n");
    std::printf("  CAN interface:   %s\n", config.canInterface.c_str());
    std::printf("  GPS device:      %s\n", config.gpsDevice.c_str());
    std::printf("  I2C device:      %s\n", config.i2cDevice.c_str());
    std::printf("  GPIO chip:       %s\n", config.gpioChip.c_str());
    std::printf("  Digital inputs:  %d\n", config.numDigitalInputs);
    std::printf("  Digital outputs: %d\n", config.numDigitalOutputs);
    std::printf("  VSS inputs:      %d\n", config.numVssInputs);
    std::printf("\n");

    // Initialize interfaces
    CANInterface can;
    GPSInterface gps;
    ADCInterface adc;
    GPIOInterface gpio;

    // Initialize CAN bus
    if (!can.init(config.canInterface, 500000)) {
        std::fprintf(stderr, "Failed to initialize CAN interface\n");
        return 1;
    }

    // Set CAN filter to accept only Variable Response frames
    can.setFilter(CAN_ID_VAR_RESPONSE, 0x7FF);

    // Initialize GPS (optional - continue if fails)
    speed_t baudRate = B9600;
    if (config.gpsBaudRate == 115200) baudRate = B115200;
    else if (config.gpsBaudRate == 38400) baudRate = B38400;
    else if (config.gpsBaudRate == 19200) baudRate = B19200;

    if (!gps.init(config.gpsDevice, baudRate)) {
        std::fprintf(stderr, "Warning: GPS not available, continuing without GPS\n");
    }

    // Initialize ADC (optional - continue if fails)
    if (!adc.init(config.i2cDevice)) {
        std::fprintf(stderr, "Warning: ADC not available, analog inputs disabled\n");
    }

    // Initialize GPIO
    if (!gpio.init(config.gpioChip)) {
        std::fprintf(stderr, "Warning: GPIO not available, digital I/O disabled\n");
    } else {
        std::printf("Configuring GPIO pins:\n");
        gpio.configureDigitalInputs(config.getDigitalInputPins());
        gpio.configureDigitalOutputs(config.getDigitalOutputPins());
        gpio.configureVssInputs(config.getVssInputPins());
        gpio.configurePwmOutputs(config.getPwmOutputPins());
    }

    std::printf("\nStarting main loop (Ctrl+C to exit)...\n\n");

    // Initialize transmission state
    int numInputs = config.numDigitalInputs;
    int numVss = config.numVssInputs;

    for (int i = 0; i < 16; ++i) {
        analogTxState[i].hasChanged = true;
        analogTxState[i].state = TX_STATE_CHANGED;
    }
    digitalTxState.hasChanged = true;
    digitalTxState.state = TX_STATE_CHANGED;
    for (int i = 0; i < 4; ++i) {
        vssTxState[i].hasChanged = true;
        vssTxState[i].state = TX_STATE_CHANGED;
    }
    for (int i = 0; i < 8; ++i) {
        gpsTxState[i].hasChanged = true;
        gpsTxState[i].state = TX_STATE_CHANGED;
    }

    // Timing variables
    uint64_t lastSlowOutRequestMs = 0;
    uint64_t lastReadMs = 0;

    // Main loop
    while (running) {
        // Poll for CAN frames
        can.receiveFrames([&gpio](const struct can_frame& frame) {
            handleCanFrame(gpio, frame);
        });

        // Poll VSS inputs for edge detection
        if (gpio.isOpen() && numVss > 0) {
            gpio.pollVSS();
            calculateVSSRates(gpio, numVss);
        }

        uint64_t nowMs = millis();

        // Request slow output state from ECU periodically (only if we have outputs)
        if (config.numDigitalOutputs > 0 || config.numPwmOutputs > 0) {
            if (nowMs - lastSlowOutRequestMs >= SLOW_OUT_REQUEST_INTERVAL_MS) {
                lastSlowOutRequestMs = nowMs;
                can.sendVariableRequestFrame(VAR_HASH_OUT_SLOW);
            }
        }

        // Read GPS data
        if (gps.isOpen()) {
            gps.readData();
        }

        // Read inputs at fixed interval
        if (nowMs - lastReadMs >= TX_READ_INTERVAL_MS) {
            lastReadMs = nowMs;

            // Read analog inputs
            if (adc.isOpen()) {
                adc.readAllChannels(currentAnalogValues);
            }

            // Read digital inputs
            if (gpio.isOpen() && numInputs > 0) {
                currentDigitalBits = gpio.readDigitalInputs();
            }

            // Update change detection and state machines
            for (int i = 0; i < 16; ++i) {
                bool changed = hasAnalogChanged(i, currentAnalogValues[i]);
                updateTxState(&analogTxState[i], changed, nowMs);
            }

            if (numInputs > 0) {
                bool digitalChanged = hasDigitalChanged(currentDigitalBits);
                updateTxState(&digitalTxState, digitalChanged, nowMs);
            }

            for (int i = 0; i < numVss && i < 4; ++i) {
                bool changed = hasVSSChanged(i, currentVssValues[i]);
                updateTxState(&vssTxState[i], changed, nowMs);
            }
        }

        // Transmit analog inputs
        for (int i = 0; i < 16; ++i) {
            transmitIfNeeded(can, &analogTxState[i], VAR_HASH_ANALOG[i],
                            currentAnalogValues[i], nowMs);
        }

        // Transmit digital inputs (only if we have inputs configured)
        if (numInputs > 0) {
            transmitIfNeeded(can, &digitalTxState, VAR_HASH_D22_D37,
                            static_cast<float>(currentDigitalBits), nowMs);
        }

        // Transmit VSS inputs
        constexpr int32_t vssHashes[4] = {
            VAR_HASH_VSS_FRONT_LEFT,
            VAR_HASH_VSS_FRONT_RIGHT,
            VAR_HASH_VSS_REAR_LEFT,
            VAR_HASH_VSS_REAR_RIGHT
        };
        for (int i = 0; i < numVss && i < 4; ++i) {
            transmitIfNeeded(can, &vssTxState[i], vssHashes[i],
                            currentVssValues[i], nowMs);
        }

        // Transmit GPS data
        transmitGPSIfNeeded(can, gps);

        // Small sleep to prevent CPU spinning (500us)
        std::this_thread::sleep_for(std::chrono::microseconds(500));
    }

    std::printf("\nShutting down...\n");

    // Cleanup
    can.close();
    gps.close();
    adc.close();
    gpio.close();

    std::printf("Goodbye!\n");
    return 0;
}
