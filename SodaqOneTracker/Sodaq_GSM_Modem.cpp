/*
 * Copyright (c) 2013-2015 Kees Bakker.  All rights reserved.
 *
 * This file is part of GPRSbee.
 *
 * GPRSbee is free software: you can redistribute it and/or modify
 * it under the terms of the GNU Lesser General Public License as
 * published by the Free Software Foundation, either version 3 of
 * the License, or(at your option) any later version.
 *
 * GPRSbee is distributed in the hope that it will be useful,
 * but WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE. See the
 * GNU Lesser General Public License for more details.
 *
 * You should have received a copy of the GNU Lesser General Public
 * License along with GPRSbee.  If not, see
 * <http://www.gnu.org/licenses/>.
 */

#include "Sodaq_GSM_Modem.h"

#define DEBUG

#ifdef DEBUG
#define debugPrintLn(...) { if (!this->_disableDiag && this->_diagStream) this->_diagStream->println(__VA_ARGS__); }
#define debugPrint(...) { if (!this->_disableDiag && this->_diagStream) this->_diagStream->print(__VA_ARGS__); }
#warning "Debug mode is ON"
#else
#define debugPrintLn(...)
#define debugPrint(...)
#endif

#define CR "\r"
#define LF "\n"
#define CRLF "\r\n"

// TODO this needs to be set in the compiler directives. Find something else to do
#define SODAQ_GSM_TERMINATOR CRLF

#ifndef SODAQ_GSM_TERMINATOR
#warning "SODAQ_GSM_TERMINATOR is not set"
#define SODAQ_GSM_TERMINATOR CRLF
#endif

#define SODAQ_GSM_TERMINATOR_LEN (sizeof(SODAQ_GSM_TERMINATOR) - 1) // without the NULL terminator

#define SODAQ_GSM_MODEM_DEFAULT_INPUT_BUFFER_SIZE 250

// Constructor
Sodaq_GSM_Modem::Sodaq_GSM_Modem() :
    _modemStream(0),
    _diagStream(0),
    _disableDiag(false),
    _inputBufferSize(SODAQ_GSM_MODEM_DEFAULT_INPUT_BUFFER_SIZE),
    _inputBuffer(0),
    _apn(0),
    _apnUser(0),
    _apnPass(0),
    _pin(0),
    _onoff(0),
    _baudRateChangeCallbackPtr(0),
    _appendCommand(false),
    _lastRSSI(0),
    _CSQtime(0),
    _minRSSI(-93),      // -93 dBm
    _echoOff(false),
    _startOn(0),
    _tcpClosedHandler(0)
{
    this->_isBufferInitialized = false;
}

// Turns the modem on and returns true if successful.
bool Sodaq_GSM_Modem::on()
{
    _startOn = millis();

    if (!isOn()) {
        if (_onoff) {
            _onoff->on();
        }
    }

    // wait for power up
    bool timeout = true;
    for (uint8_t i = 0; i < 10; i++) {
        if (isAlive()) {
            timeout = false;
            break;
        }
    }

    if (timeout) {
        debugPrintLn("Error: No Reply from Modem");
        return false;
    }

    return isOn(); // this essentially means isOn() && isAlive()
}

// Turns the modem off and returns true if successful.
bool Sodaq_GSM_Modem::off()
{
    // No matter if it is on or off, turn it off.
    if (_onoff) {
        _onoff->off();
    }

    _echoOff = false;

    return !isOn();
}

// Returns true if the modem is on.
bool Sodaq_GSM_Modem::isOn() const
{
    if (_onoff) {
        return _onoff->isOn();
    }

    // No onoff. Let's assume it is on.
    return true;
}

void Sodaq_GSM_Modem::writeProlog()
{
    if (!_appendCommand) {
        debugPrint(">> ");
        _appendCommand = true;
    }
}

// Write a byte, as binary data
size_t Sodaq_GSM_Modem::writeByte(uint8_t value)
{
    return _modemStream->write(value);
}

size_t Sodaq_GSM_Modem::print(const String& buffer)
{
    writeProlog();
    debugPrint(buffer);

    return _modemStream->print(buffer);
}

size_t Sodaq_GSM_Modem::print(const char buffer[])
{
    writeProlog();
    debugPrint(buffer);

    return _modemStream->print(buffer);
}

size_t Sodaq_GSM_Modem::print(char value)
{
    writeProlog();
    debugPrint(value);

    return _modemStream->print(value);
};

size_t Sodaq_GSM_Modem::print(unsigned char value, int base)
{
    writeProlog();
    debugPrint(value, base);

    return _modemStream->print(value, base);
};

size_t Sodaq_GSM_Modem::print(int value, int base)
{
    writeProlog();
    debugPrint(value, base);

    return _modemStream->print(value, base);
};

size_t Sodaq_GSM_Modem::print(unsigned int value, int base)
{
    writeProlog();
    debugPrint(value, base);

    return _modemStream->print(value, base);
};

size_t Sodaq_GSM_Modem::print(long value, int base)
{
    writeProlog();
    debugPrint(value, base);

    return _modemStream->print(value, base);
};

size_t Sodaq_GSM_Modem::print(unsigned long value, int base)
{
    writeProlog();
    debugPrint(value, base);

    return _modemStream->print(value, base);
};

size_t Sodaq_GSM_Modem::println(const __FlashStringHelper *ifsh)
{
    size_t n = print(ifsh);
    n += println();
    return n;
}

size_t Sodaq_GSM_Modem::println(const String &s)
{
    size_t n = print(s);
    n += println();
    return n;
}

size_t Sodaq_GSM_Modem::println(const char c[])
{
    size_t n = print(c);
    n += println();
    return n;
}

size_t Sodaq_GSM_Modem::println(char c)
{
    size_t n = print(c);
    n += println();
    return n;
}

size_t Sodaq_GSM_Modem::println(unsigned char b, int base)
{
    size_t i = print(b, base);
    return i + println();
}

size_t Sodaq_GSM_Modem::println(int num, int base)
{
    size_t i = print(num, base);
    return i + println();
}

size_t Sodaq_GSM_Modem::println(unsigned int num, int base)
{
    size_t i = print(num, base);
    return i + println();
}

size_t Sodaq_GSM_Modem::println(long num, int base)
{
    size_t i = print(num, base);
    return i + println();
}

size_t Sodaq_GSM_Modem::println(unsigned long num, int base)
{
    size_t i = print(num, base);
    return i + println();
}

size_t Sodaq_GSM_Modem::println(double num, int digits)
{
    writeProlog();
    debugPrint(num, digits);

    return _modemStream->println(num, digits);
}

size_t Sodaq_GSM_Modem::println(const Printable& x)
{
    size_t i = print(x);
    return i + println();
}

size_t Sodaq_GSM_Modem::println(void)
{
    debugPrintLn();
    size_t i = print('\r');
    _appendCommand = false;
    return i;
}

// Initializes the input buffer and makes sure it is only initialized once.
// Safe to call multiple times.
void Sodaq_GSM_Modem::initBuffer()
{
    debugPrintLn("[initBuffer]");

    // make sure the buffers are only initialized once
    if (!_isBufferInitialized) {
        this->_inputBuffer = static_cast<char*>(malloc(this->_inputBufferSize));

        _isBufferInitialized = true;
    }
}

// Sets the modem stream.
void Sodaq_GSM_Modem::setModemStream(Stream& stream)
{
    this->_modemStream = &stream;
}

void Sodaq_GSM_Modem::setApn(const char * apn, const char * user, const char * pass)
{
    if (apn) {
        if (!_apn || strcmp(_apn, apn) != 0) {
            size_t len = strlen(apn);
            _apn = static_cast<char*>(realloc(_apn, len + 1));
            strcpy(_apn, apn);
        }
    } else {
        // Should we release the memory?
    }
    if (user) {
        setApnUser(user);
    }
    if (pass) {
        setApnPass(pass);
    }
}

void Sodaq_GSM_Modem::setApnUser(const char * user)
{
    if (user) {
        if (!_apnUser || strcmp(_apnUser, user) != 0) {
            size_t len = strlen(user);
            _apnUser = static_cast<char*>(realloc(_apnUser, len + 1));
            strcpy(_apnUser, user);
        }
    }
}

void Sodaq_GSM_Modem::setApnPass(const char * pass)
{
    if (pass) {
        if (!_apnPass || strcmp(_apnPass, pass) != 0) {
            size_t len = strlen(pass);
            _apnPass = static_cast<char*>(realloc(_apnPass, len + 1));
            strcpy(_apnPass, pass);
        }
    }
}

void Sodaq_GSM_Modem::setPin(const char * pin)
{
    size_t len = strlen(pin);
    _pin = static_cast<char*>(realloc(_pin, len + 1));
    strcpy(_pin, pin);
}

// Returns a character from the modem stream if read within _timeout ms or -1 otherwise.
int Sodaq_GSM_Modem::timedRead(uint32_t timeout) const
{
    int c;
    uint32_t _startMillis = millis();

    do {
        c = _modemStream->read();
        if (c >= 0) {
            return c;
        }
    } while (millis() - _startMillis < timeout);

    return -1; // -1 indicates timeout
}

// Fills the given "buffer" with characters read from the modem stream up to "length"
// maximum characters and until the "terminator" character is found or a character read
// times out (whichever happens first).
// The buffer does not contain the "terminator" character or a null terminator explicitly.
// Returns the number of characters written to the buffer, not including null terminator.
size_t Sodaq_GSM_Modem::readBytesUntil(char terminator, char* buffer, size_t length, uint32_t timeout)
{
    if (length < 1) {
        return 0;
    }

    size_t index = 0;

    while (index < length) {
        int c = timedRead(timeout);

        if (c < 0 || c == terminator) {
            break;
        }

        *buffer++ = static_cast<char>(c);
        index++;
    }
    if (index < length) {
        *buffer = '\0';
    }

    // TODO distinguise timeout from empty string?
    // TODO return error for overflow?
    return index; // return number of characters, not including null terminator
}

// Fills the given "buffer" with up to "length" characters read from the modem stream.
// It stops when a character read times out or "length" characters have been read.
// Returns the number of characters written to the buffer.
size_t Sodaq_GSM_Modem::readBytes(uint8_t* buffer, size_t length, uint32_t timeout)
{
    size_t count = 0;

    while (count < length) {
        int c = timedRead(timeout);

        if (c < 0) {
            break;
        }

        *buffer++ = static_cast<uint8_t>(c);
        count++;
    }

    // TODO distinguise timeout from empty string?
    // TODO return error for overflow?
    return count;
}

// Reads a line (up to the SODAQ_GSM_TERMINATOR) from the modem stream into the "buffer".
// The buffer is terminated with null.
// Returns the number of bytes read, not including the null terminator.
size_t Sodaq_GSM_Modem::readLn(char* buffer, size_t size, uint32_t timeout)
{
    // Use size-1 to leave room for a string terminator
    size_t len = readBytesUntil(SODAQ_GSM_TERMINATOR[SODAQ_GSM_TERMINATOR_LEN - 1], buffer, size - 1, timeout);

    // check if the terminator is more than 1 characters, then check if the first character of it exists
    // in the calculated position and terminate the string there
    if ((SODAQ_GSM_TERMINATOR_LEN > 1) && (buffer[len - (SODAQ_GSM_TERMINATOR_LEN - 1)] == SODAQ_GSM_TERMINATOR[0])) {
        len -= SODAQ_GSM_TERMINATOR_LEN - 1;
    }

    // terminate string, there should always be room for it (see size-1 above)
    buffer[len] = '\0';

    return len;
}
