/*
 * Based on: Config.h by Kees Bakker
 *
 */

#ifndef CONFIG_H_
#define CONFIG_H_

#include <stdint.h>
#include <Arduino.h>

typedef void(*VoidCallbackMethodPtr)(void);

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

    uint8_t _isOtaaEnabled;
    
    char _devAddrOrEUI[16 + 1];
    char _appSKeyOrEUI[32 + 1];
    char _nwSKeyOrAppKey[32 + 1];

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

    uint8_t getIsOtaaEnabled() const { return _isOtaaEnabled; }
    
    const char* getDevAddrOrEUI() const { return _devAddrOrEUI; }
    const char* getAppSKeyOrEUI() const { return _appSKeyOrEUI; }
    const char* getNwSKeyOrAppKey() const { return _nwSKeyOrAppKey; }

    uint8_t getCoordinateUploadCount() const { return _coordinateUploadCount; }
    uint8_t getRepeatCount() const { return _repeatCount; }

    static void showConfig(Stream* stream);
    bool checkConfig(Stream& stream);
    void setConfigResetCallback(VoidCallbackMethodPtr callback);
};

extern ConfigParams params;

#endif
