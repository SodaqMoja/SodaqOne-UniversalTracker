/*
* Copyright (c) 2015 SODAQ. All rights reserved.
*
* This file is part of Sodaq_RN2483.
*
* Sodaq_RN2483 is free software: you can redistribute it and/or modify
* it under the terms of the GNU Lesser General Public License as
* published by the Free Software Foundation, either version 3 of
* the License, or(at your option) any later version.
*
* Sodaq_RN2483 is distributed in the hope that it will be useful,
* but WITHOUT ANY WARRANTY; without even the implied warranty of
* MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE. See the
* GNU Lesser General Public License for more details.
*
* You should have received a copy of the GNU Lesser General Public
* License along with Sodaq_RN2483.  If not, see
* <http://www.gnu.org/licenses/>.
*/

#ifndef _Sodaq_RN2483_h
#define _Sodaq_RN2483_h

#include <Arduino.h>

#include <stdint.h>
#include <Stream.h>
#include "Switchable_Device.h"

/**

 Notes:

 - uint16_t is preferred over size_t because long is never needed by the
   size of the packets or the buffers of this application.
   (Kees Bakker does not agree with this. size_t is not the same as long.
    On AVR a size_t is uint16_t. On SAMD we don't care too much about the
    data size, a long is fine.)
 - Currently, only one received packet is supported. Every time a packet is
   received, the previous one is overwritten.
 - Also multiple responses from the server (with Frame Pending Bit set) are
   not supported.
 - The port of the received packet is not returned.

 */

 //#define USE_DYNAMIC_BUFFER

#define DEFAULT_INPUT_BUFFER_SIZE 64
#define DEFAULT_RECEIVED_PAYLOAD_BUFFER_SIZE 32
#define DEFAULT_TIMEOUT 120
#define RECEIVE_TIMEOUT 60000
#define DEFAULT_FSB 2
#define DEFAULT_PWR_IDX_868 1
#define DEFAULT_PWR_IDX_915 5
#define DEFAULT_SF_868 7
#define DEFAULT_SF_915 7

#if defined(ARDUINO_ARCH_AVR)
typedef HardwareSerial SerialType;
#define ENABLE_SLEEP
#elif defined(ARDUINO_ARCH_SAM) || defined(ARDUINO_ARCH_SAMD)
typedef Uart SerialType;
#define ENABLE_SLEEP
#else
typedef Stream SerialType;
#endif

// Available error codes.
enum MacTransmitErrorCodes
{
    NoError = 0,
    NoResponse = 1,
    Timeout = 2,
    PayloadSizeError = 3,
    InternalError = 4,
    Busy = 5,
    NetworkFatalError = 6,
    NotConnected = 7,
    NoAcknowledgment = 8,
};

// Provides a simple, abstracted interface to Microchip's RN2483 LoRaWAN module.
// Implements SwitchableDevice for turning the device On/Off on supported boards.
//
// It is strongly suggested to use the static instance that is included with the library (LoRaBee)
// and not to create a new instance.
class Sodaq_RN2483 : public SwitchableDevice
{
public:
    // Creates a new Sodaq_RN2483 instance.
    Sodaq_RN2483();

    // Returns the correct baudrate for the serial port that connects to the device.
    uint32_t getDefaultBaudRate() { return 57600; };

    // Takes care of the initialization tasks common to both initOTA() and initABP().
    // If hardware reset is available, the module is re-set, otherwise it is woken up if possible.
    // Returns true if the module replies to a device reset command.
    bool init(SerialType& stream, int8_t resetPin = -1);

    // Initializes the device and connects to the network using Over-The-Air Activation.
    // Returns true on successful connection.
    bool initOTA(SerialType& stream, const uint8_t devEUI[8], const uint8_t appEUI[8], const uint8_t appKey[16], bool adr = true, int8_t resetPin = -1);

    // Initializes the device and connects to the network using Activation By Personalization.
    // Returns true on successful connection.
    bool initABP(SerialType& stream, const uint8_t devAddr[4], const uint8_t appSKey[16], const uint8_t nwkSKey[16], bool adr = true, int8_t resetPin = -1);

    // Sets the optional "Diagnostics and Debug" stream.
    void setDiag(Stream& stream) { diagStream = &stream; };

    // Performs a hardware reset (using the reset pin -if available).
    void hardwareReset();

    // Sends the given payload without acknowledgement.
    // Returns 0 (NoError) when transmission is successful or one of the MacTransmitErrorCodes otherwise.
    uint8_t send(uint8_t port, const uint8_t* payload, uint8_t size);

    // Sends the given payload with acknowledgement.
    // Returns 0 (NoError) when transmission is successful or one of the MacTransmitErrorCodes otherwise.
    uint8_t sendReqAck(uint8_t port, const uint8_t* payload, uint8_t size, uint8_t maxRetries);

    // Copies the latest received packet (optionally starting from the "payloadStartPosition"
    // position of the payload) into the given "buffer", up to "size" number of bytes.
    // Returns the number of bytes written or 0 if no packet is received since last transmission.
    uint16_t receive(uint8_t* buffer, uint16_t size, uint16_t payloadStartPosition = 0);

    // Gets the preprogrammed EUI node address from the module.
    // Returns the number of bytes written or 0 in case of error.
    uint8_t getHWEUI(uint8_t* buffer, uint8_t size);

    // Enables all the channels that belong to the given Frequency Sub-Band (FSB)
    // and disables the rest.
    // fsb is [1, 8] or 0 to enable all channels.
    // Returns true if all channels were set successfully.
    bool setFsbChannels(uint8_t fsb);

    // Sets the spreading factor.
    // In reality it sets the datarate of the module according to the
    // LoraWAN specs mapping for 868MHz and 915MHz, 
    // using the given spreading factor parameter.
    bool setSpreadingFactor(uint8_t spreadingFactor);

    // Sets the power index (868MHz: 1 to 5 / 915MHz: 5, 7, 8, 9 or 10)
    // Returns true if successful.
    bool setPowerIndex(uint8_t powerIndex);

    // Sends the command together with the given paramValue (optional)
    // to the device and awaits for the response.
    // Returns true on success.
    // NOTE: command should include a trailing space if paramValue is set
    bool sendCommand(const char* command, const uint8_t* paramValue, uint16_t size);
    bool sendCommand(const char* command, uint8_t paramValue);
    bool sendCommand(const char* command, const char* paramValue = NULL);

    // Sends the given mac command together with the given paramValue
    // to the device and awaits for the response.
    // Returns true on success.
    // NOTE: paramName should include a trailing space
    bool setMacParam(const char* paramName, const uint8_t* paramValue, uint16_t size);
    bool setMacParam(const char* paramName, uint8_t paramValue);
    bool setMacParam(const char* paramName, const char* paramValue);

#ifdef ENABLE_SLEEP
    // Wakes up the module from sleep (if supported).
    void wakeUp();

    // Puts the module to sleep (if supported).
    void sleep();
#endif

#ifdef USE_DYNAMIC_BUFFER
    // Sets the size of the input buffer.
    // Needs to be called before initOTA()/initABP().
    void setInputBufferSize(uint16_t value) { this->inputBufferSize = value; };

    // Sets the size of the "Received Payload" buffer.
    // Needs to be called before initOTA()/initABP().
    void setReceivedPayloadBufferSize(uint16_t value) { this->receivedPayloadBufferSize = value; };
#endif

    // Provides a quick test of several methods as a pseudo-unit test.
    void runTestSequence(SerialType& loraStream, Stream& debugStream);

private:
    // The stream that communicates with the device.
    SerialType* loraStream;

    // The (optional) stream to show debug information.
    Stream* diagStream;

    // The size of the input buffer. Equals DEFAULT_INPUT_BUFFER_SIZE
    // by default or (optionally) a user-defined value when using USE_DYNAMIC_BUFFER.
    uint16_t inputBufferSize;

    // The size of the received payload buffer. Equals DEFAULT_RECEIVED_PAYLOAD_BUFFER_SIZE
    // by default or (optionally) a user-defined value when using USE_DYNAMIC_BUFFER.
    uint16_t receivedPayloadBufferSize;

    // Flag used to make sure the received payload buffer is
    // current with the latest transmission.
    bool packetReceived;

    // Used to distinguise between RN2483 and RN2903. 
    // Currently only being set during reset().
    bool isRN2903;

#ifdef USE_DYNAMIC_BUFFER
    // Flag to make sure the buffers are not allocated more than once.
    bool isBufferInitialized;

    char* inputBuffer;
    char* receivedPayloadBuffer;
#else
    char inputBuffer[DEFAULT_INPUT_BUFFER_SIZE];
    char receivedPayloadBuffer[DEFAULT_RECEIVED_PAYLOAD_BUFFER_SIZE];
#endif

    // Used for resetting the module on init.
    int8_t resetPin;

    // Enables hardware-resetting the module.
    void enableHardwareReset(uint8_t resetPin) { this->resetPin = resetPin; };

    // Returns true if the hardware reset pin is set.
    bool isHardwareResetEnabled() { return resetPin >= 0; };

    // Reads a line from the device stream into the "buffer" starting at the "start" position of the buffer.
    // Returns the number of bytes read.
    uint16_t readLn(char* buffer, uint16_t size, uint16_t start = 0);

    // Reads a line from the device stream into the input buffer.
    // Returns the number of bytes read.
    uint16_t readLn() { return readLn(this->inputBuffer, this->inputBufferSize); };

    // Waits for the given string. Returns true if the string is received before a timeout.
    // Returns false if a timeout occurs or if another string is received.
    bool expectString(const char* str, uint16_t timeout = DEFAULT_TIMEOUT);
    bool expectOK();

    // Sends a reset command to the module and waits for the success response (or timeout).
    // Returns true on success.
    bool resetDevice();

    // Sends a join network command to the device and waits for the response (or timeout).
    // Returns true on success.
    bool joinNetwork(const char* type);

    // Returns the enum that is mapped to the given "error" message.
    uint8_t lookupMacTransmitError(const char* error);

    // Sends a a payload and blocks until there is a response back, or the receive windows have closed,
    // or the hard timeout has passed.
    uint8_t macTransmit(const char* type, uint8_t port, const uint8_t* payload, uint8_t size);

    // Parses the input buffer and copies the received payload into the "received payload" buffer
    // when a "mac rx" message has been received. It is called internally by macTransmit().
    // Returns 0 (NoError) or otherwise one of the MacTransmitErrorCodes.
    uint8_t onMacRX();
};

extern Sodaq_RN2483 LoRaBee;

#endif // Sodaq_RN2483
