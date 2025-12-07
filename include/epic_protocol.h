/*
 * EPIC CAN Bus Protocol Constants
 * Defines the communication protocol between the I/O expander and epicEFI ECU
 */

#ifndef EPIC_PROTOCOL_H
#define EPIC_PROTOCOL_H

#include <cstdint>

// ECU and EPIC CAN configuration
constexpr uint8_t ECU_CAN_ID = 1;
constexpr uint32_t CAN_ID_VAR_REQUEST       = 0x700 + ECU_CAN_ID;
constexpr uint32_t CAN_ID_VAR_RESPONSE      = 0x720 + ECU_CAN_ID;
constexpr uint32_t CAN_ID_FUNCTION_REQUEST  = 0x740 + ECU_CAN_ID;
constexpr uint32_t CAN_ID_FUNCTION_RESPONSE = 0x760 + ECU_CAN_ID;
constexpr uint32_t CAN_ID_VARIABLE_SET      = 0x780 + ECU_CAN_ID;

// Timing parameters (milliseconds)
constexpr uint32_t SLOW_OUT_REQUEST_INTERVAL_MS = 25;
constexpr uint32_t TX_INTERVAL_FAST_MS = 25;   // Fast transmission for changed values
constexpr uint32_t TX_INTERVAL_SLOW_MS = 500;  // Slow transmission for stable values
constexpr uint32_t TX_READ_INTERVAL_MS = 10;   // Input reading interval
constexpr uint32_t VSS_CALC_INTERVAL_MS = 25;  // VSS rate calculation interval

// Change detection thresholds
constexpr float TX_ANALOG_THRESHOLD = 2.0f;    // ADC counts
constexpr float TX_VSS_THRESHOLD = 0.1f;       // Pulses per second
constexpr float GPS_FLOAT_THRESHOLD = 0.001f;  // GPS float comparison

// EPIC variable hashes for analog inputs A0-A15
constexpr int32_t VAR_HASH_ANALOG[16] = {
    595545759,   // A0
    595545760,   // A1
    595545761,   // A2
    595545762,   // A3
    595545763,   // A4
    595545764,   // A5
    595545765,   // A6
    595545766,   // A7
    595545767,   // A8
    595545768,   // A9
    -1821826352, // A10
    -1821826351, // A11
    -1821826350, // A12
    -1821826349, // A13
    -1821826348, // A14
    -1821826347  // A15
};

// Digital inputs D22-D37 bitfield hash
constexpr int32_t VAR_HASH_D22_D37 = 2138825443;

// Slow GPIO and PWM outputs bitfield hash
constexpr int32_t VAR_HASH_OUT_SLOW = 1430780106;

// GPS variable hashes
constexpr int32_t VAR_HASH_GPS_HMSD_PACKED   = 703958849;
constexpr int32_t VAR_HASH_GPS_MYQSAT_PACKED = -1519914092;
constexpr int32_t VAR_HASH_GPS_ACCURACY      = -1489698215;
constexpr int32_t VAR_HASH_GPS_ALTITUDE      = -2100224086;
constexpr int32_t VAR_HASH_GPS_COURSE        = 1842893663;
constexpr int32_t VAR_HASH_GPS_LATITUDE      = 1524934922;
constexpr int32_t VAR_HASH_GPS_LONGITUDE     = -809214087;
constexpr int32_t VAR_HASH_GPS_SPEED         = -1486968225;

// VSS variable hashes
constexpr int32_t VAR_HASH_VSS_FRONT_LEFT  = -1645222329;
constexpr int32_t VAR_HASH_VSS_FRONT_RIGHT = 1549498074;
constexpr int32_t VAR_HASH_VSS_REAR_LEFT   = 768443592;
constexpr int32_t VAR_HASH_VSS_REAR_RIGHT  = -403905157;

// Smart transmission state machine
enum TxState : uint8_t {
    TX_STATE_CHANGED = 0,  // Value changed, transmit quickly
    TX_STATE_STABLE = 1    // Value stable, transmit slowly
};

struct TxChannelState {
    float lastTransmittedValue = 0.0f;
    uint64_t lastTxTimeMs = 0;
    bool hasChanged = true;  // Force initial transmission
    TxState state = TX_STATE_CHANGED;
};

#endif // EPIC_PROTOCOL_H
