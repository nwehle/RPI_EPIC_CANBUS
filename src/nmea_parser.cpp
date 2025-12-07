/*
 * NMEA Parser Implementation
 * Parses GPRMC and GPGGA sentences from GPS modules
 * Ported from Arduino version for Raspberry Pi
 */

#include "nmea_parser.h"
#include <cstring>
#include <cstdlib>
#include <cmath>

NMEAParser::NMEAParser() : bufferIndex_(0) {
    buffer_[0] = '\0';
}

void NMEAParser::parseTime(const char* timeStr, uint8_t& hours, uint8_t& minutes, uint8_t& seconds) {
    if (!timeStr || std::strlen(timeStr) < 6) {
        hours = 0;
        minutes = 0;
        seconds = 0;
        return;
    }

    char hh[3] = {timeStr[0], timeStr[1], '\0'};
    char mm[3] = {timeStr[2], timeStr[3], '\0'};
    char ss[3] = {timeStr[4], timeStr[5], '\0'};

    hours = static_cast<uint8_t>(std::atoi(hh));
    minutes = static_cast<uint8_t>(std::atoi(mm));
    seconds = static_cast<uint8_t>(std::atoi(ss));

    if (hours > 23) hours = 0;
    if (minutes > 59) minutes = 0;
    if (seconds > 59) seconds = 0;
}

void NMEAParser::parseDate(const char* dateStr, uint8_t& days, uint8_t& months, uint8_t& years) {
    if (!dateStr || std::strlen(dateStr) < 6) {
        days = 1;
        months = 1;
        years = 25;
        return;
    }

    char dd[3] = {dateStr[0], dateStr[1], '\0'};
    char mm[3] = {dateStr[2], dateStr[3], '\0'};
    char yy[3] = {dateStr[4], dateStr[5], '\0'};

    days = static_cast<uint8_t>(std::atoi(dd));
    months = static_cast<uint8_t>(std::atoi(mm));
    years = static_cast<uint8_t>(std::atoi(yy));

    if (days < 1 || days > 31) days = 1;
    if (months < 1 || months > 12) months = 1;
    if (years > 99) years = 25;
}

float NMEAParser::parseLatitude(const char* latStr, char ns) {
    if (!latStr || std::strlen(latStr) < 4) return 0.0f;

    char degStr[3] = {latStr[0], latStr[1], '\0'};
    int degrees = std::atoi(degStr);
    float minutes = std::atof(latStr + 2);

    float decimalDegrees = degrees + (minutes / 60.0f);
    if (ns == 'S' || ns == 's') {
        decimalDegrees = -decimalDegrees;
    }

    return decimalDegrees;
}

float NMEAParser::parseLongitude(const char* lonStr, char ew) {
    if (!lonStr || std::strlen(lonStr) < 5) return 0.0f;

    char degStr[4] = {lonStr[0], lonStr[1], lonStr[2], '\0'};
    int degrees = std::atoi(degStr);
    float minutes = std::atof(lonStr + 3);

    float decimalDegrees = degrees + (minutes / 60.0f);
    if (ew == 'W' || ew == 'w') {
        decimalDegrees = -decimalDegrees;
    }

    return decimalDegrees;
}

const char* NMEAParser::getField(const char* sentence, uint8_t fieldIndex) {
    if (!sentence) return nullptr;

    const char* p = sentence;
    uint8_t currentField = 0;

    if (*p == '$') p++;

    const char* fieldStart = p;

    while (*p) {
        if (*p == ',' || *p == '*') {
            if (currentField == fieldIndex) {
                if (fieldStart == p) return nullptr;
                return fieldStart;
            }
            currentField++;
            if (*p == ',') {
                p++;
                fieldStart = p;
            } else {
                break;
            }
        } else {
            p++;
        }
    }

    if (currentField == fieldIndex && fieldStart < p) {
        return fieldStart;
    }

    return nullptr;
}

uint8_t NMEAParser::calculateChecksum(const char* sentence) {
    if (!sentence || *sentence != '$') return 0;

    uint8_t checksum = 0;
    const char* p = sentence + 1;

    while (*p && *p != '*') {
        checksum ^= *p;
        p++;
    }

    return checksum;
}

bool NMEAParser::verifyChecksum(const char* sentence) {
    if (!sentence) return false;

    const char* star = std::strchr(sentence, '*');
    if (!star || std::strlen(star) < 3) return false;

    uint8_t calculated = calculateChecksum(sentence);
    uint8_t received = static_cast<uint8_t>(std::strtoul(star + 1, nullptr, 16));

    return (calculated == received);
}

// Helper to extract field into buffer
static void extractField(const char* fieldPtr, char* buf, size_t bufSize) {
    if (!fieldPtr || !buf || bufSize == 0) return;

    const char* end = std::strchr(fieldPtr, ',');
    size_t len;
    if (end) {
        len = end - fieldPtr;
    } else {
        end = std::strchr(fieldPtr, '*');
        if (end) {
            len = end - fieldPtr;
        } else {
            len = std::strlen(fieldPtr);
        }
    }

    if (len >= bufSize) len = bufSize - 1;
    std::strncpy(buf, fieldPtr, len);
    buf[len] = '\0';
}

bool NMEAParser::parseGPRMC(const char* sentence, GPSData& gpsData) {
    if (!sentence || std::strncmp(sentence, "$GPRMC", 6) != 0) return false;
    if (!verifyChecksum(sentence)) return false;

    const char* timeStr = getField(sentence, 1);
    const char* statusStr = getField(sentence, 2);
    const char* latStr = getField(sentence, 3);
    const char* nsStr = getField(sentence, 4);
    const char* lonStr = getField(sentence, 5);
    const char* ewStr = getField(sentence, 6);
    const char* speedStr = getField(sentence, 7);
    const char* courseStr = getField(sentence, 8);
    const char* dateStr = getField(sentence, 9);

    if (!timeStr || !statusStr || !dateStr) return false;

    char timeBuf[16] = {0};
    extractField(timeStr, timeBuf, sizeof(timeBuf));
    parseTime(timeBuf, gpsData.hours, gpsData.minutes, gpsData.seconds);

    char dateBuf[7] = {0};
    extractField(dateStr, dateBuf, sizeof(dateBuf));
    parseDate(dateBuf, gpsData.days, gpsData.months, gpsData.years);

    bool hasValidFix = (*statusStr == 'A' || *statusStr == 'a');
    gpsData.hasFix = hasValidFix;

    if (hasValidFix && latStr && nsStr && lonStr && ewStr) {
        char latBuf[16] = {0};
        extractField(latStr, latBuf, sizeof(latBuf));
        gpsData.latitude = parseLatitude(latBuf, *nsStr);

        char lonBuf[16] = {0};
        extractField(lonStr, lonBuf, sizeof(lonBuf));
        gpsData.longitude = parseLongitude(lonBuf, *ewStr);

        if (speedStr) {
            char speedBuf[16] = {0};
            extractField(speedStr, speedBuf, sizeof(speedBuf));
            float speedKnots = std::atof(speedBuf);
            gpsData.speed = speedKnots * 1.852f;
        }

        if (courseStr) {
            char courseBuf[16] = {0};
            extractField(courseStr, courseBuf, sizeof(courseBuf));
            gpsData.course = std::atof(courseBuf);
        }
    }

    gpsData.dataValid = true;
    return true;
}

bool NMEAParser::parseGPGGA(const char* sentence, GPSData& gpsData) {
    if (!sentence || std::strncmp(sentence, "$GPGGA", 6) != 0) return false;
    if (!verifyChecksum(sentence)) return false;

    const char* latStr = getField(sentence, 2);
    const char* nsStr = getField(sentence, 3);
    const char* lonStr = getField(sentence, 4);
    const char* ewStr = getField(sentence, 5);
    const char* qualityStr = getField(sentence, 6);
    const char* numSatsStr = getField(sentence, 7);
    const char* hdopStr = getField(sentence, 8);
    const char* altitudeStr = getField(sentence, 9);

    if (!qualityStr) return false;

    char qualityBuf[4] = {0};
    extractField(qualityStr, qualityBuf, sizeof(qualityBuf));
    gpsData.quality = static_cast<uint8_t>(std::atoi(qualityBuf));

    if (numSatsStr) {
        char satsBuf[4] = {0};
        extractField(numSatsStr, satsBuf, sizeof(satsBuf));
        gpsData.satellites = static_cast<uint8_t>(std::atoi(satsBuf));
    }

    gpsData.hasFix = (gpsData.quality > 0);

    if (gpsData.hasFix && latStr && nsStr && lonStr && ewStr &&
        (*nsStr == 'N' || *nsStr == 'S' || *nsStr == 'n' || *nsStr == 's') &&
        (*ewStr == 'E' || *ewStr == 'W' || *ewStr == 'e' || *ewStr == 'w')) {

        char latBuf[16] = {0};
        extractField(latStr, latBuf, sizeof(latBuf));
        gpsData.latitude = parseLatitude(latBuf, *nsStr);

        char lonBuf[16] = {0};
        extractField(lonStr, lonBuf, sizeof(lonBuf));
        gpsData.longitude = parseLongitude(lonBuf, *ewStr);
    }

    if (gpsData.hasFix && altitudeStr) {
        char altBuf[16] = {0};
        extractField(altitudeStr, altBuf, sizeof(altBuf));
        gpsData.altitude = std::atof(altBuf);
    }

    if (gpsData.hasFix && hdopStr) {
        char hdopBuf[16] = {0};
        extractField(hdopStr, hdopBuf, sizeof(hdopBuf));
        gpsData.accuracy = std::atof(hdopBuf);
    }

    gpsData.dataValid = true;
    return true;
}

bool NMEAParser::processChar(char c, GPSData& gpsData) {
    if (c == '$') {
        bufferIndex_ = 0;
        buffer_[0] = c;
        bufferIndex_ = 1;
        return false;
    } else if (c == '\r' || c == '\n') {
        if (bufferIndex_ > 0) {
            buffer_[bufferIndex_] = '\0';

            bool parsed = false;
            if (std::strncmp(buffer_, "$GPRMC", 6) == 0) {
                parsed = parseGPRMC(buffer_, gpsData);
            } else if (std::strncmp(buffer_, "$GPGGA", 6) == 0) {
                parsed = parseGPGGA(buffer_, gpsData);
            }

            bufferIndex_ = 0;
            return parsed;
        }
        bufferIndex_ = 0;
        return false;
    } else if (bufferIndex_ < SENTENCE_MAX_LEN) {
        buffer_[bufferIndex_++] = c;
        return false;
    } else {
        bufferIndex_ = 0;
        return false;
    }
}
