/*
Copyright (c) 2016, SODAQ
All rights reserved.

Redistribution and use in source and binary forms, with or without
modification, are permitted provided that the following conditions are met:

1. Redistributions of source code must retain the above copyright notice,
this list of conditions and the following disclaimer.

2. Redistributions in binary form must reproduce the above copyright notice,
this list of conditions and the following disclaimer in the documentation
and/or other materials provided with the distribution.

3. Neither the name of the copyright holder nor the names of its contributors
may be used to endorse or promote products derived from this software without
specific prior written permission.

THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS"
AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO,
THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE
ARE DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT HOLDER OR CONTRIBUTORS BE
LIABLE FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR
CONSEQUENTIAL DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF
SUBSTITUTE GOODS OR SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS
INTERRUPTION) HOWEVER CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN
CONTRACT, STRICT LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE)
ARISING IN ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE
POSSIBILITY OF SUCH DAMAGE.
*/

#include "Config.h"
#include "Command.h"
#include "FlashStorage.h"

#define DEFAULT_HEADER 0xBEEF

ConfigParams params;

FlashStorage(flash, ConfigParams);
static bool needsCommit;
static VoidCallbackMethodPtr configResetCallback;

static uint16_t crc16ccitt(const uint8_t *buf, size_t len)
{
    uint16_t crc = 0;
    while (len--) {
        crc ^= (*buf++ << 8);
        for (uint8_t i = 0; i < 8; ++i) {
            if (crc & 0x8000) {
                crc = (crc << 1) ^ 0x1021;
            }
            else {
                crc = crc << 1;
            }
        }
    }
    return crc;
}

void ConfigParams::read()
{
    flash.read(this);

    // check header and CRC
    uint16_t calcCRC16 = crc16ccitt((uint8_t*)this, (uint32_t)&params._crc16 - (uint32_t)&params._header);
    if (_header != DEFAULT_HEADER || _crc16 != calcCRC16) {
        reset();
    }
}

void ConfigParams::reset()
{
    _defaultFixInterval = 15;
    _alternativeFixInterval = 0;
    _alternativeFixFromHours = 0;
    _alternativeFixFromMinutes = 0;
    _alternativeFixToHours = 0;
    _alternativeFixToMinutes = 0;
    _gpsFixTimeout = 120;

    memset(_devAddrOrEUI, 0x30, sizeof(_devAddrOrEUI) - 1);
    _devAddrOrEUI[sizeof(_devAddrOrEUI) - 1] = '\0';

    memset(_appSKeyOrEUI, 0x30, sizeof(_appSKeyOrEUI) - 1);
    _appSKeyOrEUI[sizeof(_appSKeyOrEUI) - 1] = '\0';

    memset(_nwSKeyOrAppKey, 0x30, sizeof(_nwSKeyOrAppKey) - 1);
    _nwSKeyOrAppKey[sizeof(_nwSKeyOrAppKey) - 1] = '\0';

    _coordinateUploadCount = 1;
    _repeatCount = 0;

    _isAdrOn = 1;
    _isAckOn = 0;
    _spreadingFactor = 7;
    _powerIndex = 1;
    _isGpsOn = 1;
    _gpsMinSatelliteCount = 4;
    _isDebugOn = 0;

    if (configResetCallback) {
        configResetCallback();
    }

    needsCommit = true;
}

/*
 * Write the configuration parameters to NVM / Dataflash
 */
void ConfigParams::commit(bool forced)
{
    if (!forced && !needsCommit) {
        return;
    }

    _header = DEFAULT_HEADER;
    _crc16 = crc16ccitt((uint8_t*)this, (uint32_t)&params._crc16 - (uint32_t)&params._header);

    flash.write(*this);

    needsCommit = false;
}

static const Command args[] = {
    { "GPS (OFF=0 / ON=1)        ", "gps=", Command::set_uint8, Command::show_uint8, &params._isGpsOn },
    { "Fix Interval (min)        ", "fi=", Command::set_uint16, Command::show_uint16, &params._defaultFixInterval },
    { "Alt. Fix Interval (min)   ", "afi=", Command::set_uint16, Command::show_uint16, &params._alternativeFixInterval },
    { "Alt. Fix From (HH)        ", "affh=", Command::set_uint8, Command::show_uint8, &params._alternativeFixFromHours },
    { "Alt. Fix From (MM)        ", "affm=", Command::set_uint8, Command::show_uint8, &params._alternativeFixFromMinutes },
    { "Alt. Fix To (HH)          ", "afth=", Command::set_uint8, Command::show_uint8, &params._alternativeFixToHours },
    { "Alt. Fix To (MM)          ", "aftm=", Command::set_uint8, Command::show_uint8, &params._alternativeFixToMinutes },
    { "GPS Fix Timeout (sec)     ", "gft=", Command::set_uint16, Command::show_uint16, &params._gpsFixTimeout },
    { "Minimum sat count         ", "sat=", Command::set_uint8, Command::show_uint8, &params._gpsMinSatelliteCount },

    { "OTAA Mode (OFF=0 / ON=1)  ", "otaa=", Command::set_uint8, Command::show_uint8, &params._isOtaaEnabled },
    { "Retry conn. (OFF=0 / ON=1)", "retry=", Command::set_uint8, Command::show_uint8, &params._shouldRetryConnectionOnSend },
    { "ADR (OFF=0 / ON=1)        ", "adr=", Command::set_uint8, Command::show_uint8, &params._isAdrOn },
    { "ACK (OFF=0 / ON=1)        ", "ack=", Command::set_uint8, Command::show_uint8, &params._isAckOn },
    { "Spreading Factor          ", "sf=", Command::set_uint8, Command::show_uint8, &params._spreadingFactor },
    { "Output Index              ", "pwr=", Command::set_uint8, Command::show_uint8, &params._powerIndex },
    { "DevAddr / DevEUI          ", "dev=", Command::set_string, Command::show_string, params._devAddrOrEUI, sizeof(params._devAddrOrEUI) },
    { "AppSKey / AppEUI          ", "app=", Command::set_string, Command::show_string, params._appSKeyOrEUI, sizeof(params._appSKeyOrEUI) },
    { "NWSKey / AppKey           ", "key=", Command::set_string, Command::show_string, params._nwSKeyOrAppKey, sizeof(params._nwSKeyOrAppKey) },

    { "Num Coords to Upload      ", "num=", Command::set_uint8, Command::show_uint8, &params._coordinateUploadCount },
    { "Repeat Count              ", "rep=", Command::set_uint8, Command::show_uint8, &params._repeatCount },
    { "Status LED (OFF=0 / ON=1) ", "led=", Command::set_uint8, Command::show_uint8, &params._isLedEnabled },
    { "Debug (OFF=0 / ON=1)      ", "dbg=", Command::set_uint8, Command::show_uint8, &params._isDebugOn }
};

void ConfigParams::showConfig(Stream* stream)
{
    stream->println();
    stream->println("Settings:");
    for (size_t i = 0; i < sizeof(args) / sizeof(args[0]); ++i) {
        const Command* a = &args[i];
        if (a->show_func) {
            a->show_func(a, stream);
        }
    }
}

/*
 * Execute a command from the commandline
 *
 * Return true if it was a valid command
 */
bool ConfigParams::execCommand(const char* line)
{
    bool done = Command::execCommand(args, sizeof(args) / sizeof(args[0]), line);
    if (done) {
        needsCommit = true;
    }

    return done;
}

/*
 * Check if all required config parameters are filled in
 */
bool ConfigParams::checkConfig(Stream& stream)
{
    bool fail = false;

    if (_alternativeFixFromHours > 23) {
        stream.println("\n\nERROR: \"Alt. Fix From (HH)\" must not be more than 23");

        fail = true;
    }

    if (_alternativeFixToHours > 23) {
        stream.println("\n\nERROR: \"Alt. Fix To (HH)\" must not be more than 23");

        fail = true;
    }

    if (_alternativeFixFromMinutes > 59) {
        stream.println("\n\nERROR: \"Alt. Fix From (MM)\" must not be more than 59");

        fail = true;
    }

    if (_alternativeFixToMinutes > 59) {
        stream.println("\n\nERROR: \"Alt. Fix To (MM)\" must not be more than 59");

        fail = true;
    }

    if (_alternativeFixToMinutes > 59) {
        stream.println("\n\nERROR: \"Alt. Fix To (MM)\" must not be more than 59");

        fail = true;
    }

    if (_coordinateUploadCount < 1 || _coordinateUploadCount > 4) {
        stream.println("\n\nERROR: \"Num Coords to Upload\" must be between 1 and 4");

        fail = true;
    }

    if (_isOtaaEnabled > 1) {
        stream.println("OTAA Mode must be either 0 or 1");
        fail = true;
    }

    if (_isAdrOn > 1) {
        stream.println("ADR must be either 0 or 1");
        fail = true;
    }

    if (_isAckOn > 1) {
        stream.println("ACK must be either 0 or 1");
        fail = true;
    }

    if (_isGpsOn > 1) {
        stream.println("GPS must be either 0 or 1");
        fail = true;
    }

    if (_isDebugOn > 1) {
        stream.println("Debug must be either 0 or 1");
        fail = true;
    }

    return !fail;
}

void ConfigParams::setConfigResetCallback(VoidCallbackMethodPtr callback)
{
    configResetCallback = callback;
}
