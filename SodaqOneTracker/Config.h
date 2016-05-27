/*
 * Based on: Config.h by Kees Bakker
 *
 */

#ifndef CONFIG_H_
#define CONFIG_H_

#include <stdint.h>
#include <Arduino.h>

struct ConfigParams
{
    uint16_t _header;

    uint16_t _defaultFixInterval;
    uint16_t _alternativeFixInterval;
    uint8_t _alternativeFixFromHours;
    uint8_t _alternativeFixFromMinutes;
    uint8_t _alternativeFixToHours;
    uint8_t _alternativeFixToMinutes;
    uint16_t _gpsFixTimeout;

    char _devAddr[8 + 1];
    char _appSKey[32 + 1];
    char _nwSKey[32 + 1];

    uint8_t _coordinateUploadCount;
    uint8_t _repeatCount;

    uint16_t _crc16;

public:
    void read();
    void commit(bool forced = false);
    void reset();

    bool execCommand(const char* line);

    uint16_t getDefaultFixInterval() const { return _defaultFixInterval; }
    uint16_t getAlternativeFixInterval() const { return _alternativeFixInterval; }
    uint8_t getAlternativeFixFromHours() const { return _alternativeFixFromHours; }
    uint8_t getAlternativeFixFromMinutes() const { return _alternativeFixFromMinutes; }
    uint32_t getAlternativeFixFrom() const { return _alternativeFixFromHours * 60 * 60 + _alternativeFixFromMinutes * 60; }
    uint8_t getAlternativeFixToHours() const { return _alternativeFixToHours; }
    uint8_t getAlternativeFixToMinutes() const { return _alternativeFixToMinutes; }
    uint32_t getAlternativeFixTo() const { return _alternativeFixToHours * 60 * 60 + _alternativeFixToMinutes * 60; }
    uint16_t getGpsFixTimeout() const { return _gpsFixTimeout; }

    const char* getDevAddr() const { return _devAddr; }
    const char* getAppSKey() const { return _appSKey; }
    const char* getNwSKey() const { return _nwSKey; }

    uint8_t getCoordinateUploadCount() const { return _coordinateUploadCount; }
    uint8_t getRepeatCount() const { return _repeatCount; }

    static void showConfig(Stream* stream);
    bool checkConfig(Stream& stream);
};

extern ConfigParams params;

#endif
