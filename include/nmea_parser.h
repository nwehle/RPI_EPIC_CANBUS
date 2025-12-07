/*
 * NMEA Parser for GPS modules
 * Parses GPRMC and GPGGA sentences
 * Ported from Arduino version for Raspberry Pi
 */

#ifndef NMEA_PARSER_H
#define NMEA_PARSER_H

#include <cstdint>
#include <cstddef>

// GPS data structure
struct GPSData {
    uint8_t hours = 0;
    uint8_t minutes = 0;
    uint8_t seconds = 0;
    uint8_t days = 0;
    uint8_t months = 0;
    uint8_t years = 0;    // 2-digit year from NMEA (0-99)
    uint8_t quality = 0;
    uint8_t satellites = 0;
    float accuracy = 0.0f;
    float altitude = 0.0f;
    float course = 0.0f;
    float latitude = 0.0f;
    float longitude = 0.0f;
    float speed = 0.0f;
    bool hasFix = false;
    bool dataValid = false;
};

// NMEA parser class
class NMEAParser {
public:
    static constexpr size_t SENTENCE_MAX_LEN = 82;

    NMEAParser();

    // Process incoming character, returns true if a complete sentence was parsed
    bool processChar(char c, GPSData& gpsData);

    // Get pointer to field in comma-separated NMEA sentence
    static const char* getField(const char* sentence, uint8_t fieldIndex);

    // Verify NMEA sentence checksum
    static bool verifyChecksum(const char* sentence);

private:
    char buffer_[SENTENCE_MAX_LEN + 1];
    size_t bufferIndex_;

    // Parse specific sentence types
    bool parseGPRMC(const char* sentence, GPSData& gpsData);
    bool parseGPGGA(const char* sentence, GPSData& gpsData);

    // Coordinate parsing helpers
    static void parseTime(const char* timeStr, uint8_t& hours, uint8_t& minutes, uint8_t& seconds);
    static void parseDate(const char* dateStr, uint8_t& days, uint8_t& months, uint8_t& years);
    static float parseLatitude(const char* latStr, char ns);
    static float parseLongitude(const char* lonStr, char ew);
    static uint8_t calculateChecksum(const char* sentence);
};

// Utility functions for GPS data packing
inline uint32_t packGPSHMSD(uint8_t hours, uint8_t minutes, uint8_t seconds, uint8_t days) {
    return static_cast<uint32_t>(hours) |
           (static_cast<uint32_t>(minutes) << 8) |
           (static_cast<uint32_t>(seconds) << 16) |
           (static_cast<uint32_t>(days) << 24);
}

inline uint32_t packGPSMYQSAT(uint8_t months, uint8_t years, uint8_t quality, uint8_t satellites) {
    return static_cast<uint32_t>(months) |
           (static_cast<uint32_t>(years) << 8) |
           (static_cast<uint32_t>(quality) << 16) |
           (static_cast<uint32_t>(satellites) << 24);
}

#endif // NMEA_PARSER_H
