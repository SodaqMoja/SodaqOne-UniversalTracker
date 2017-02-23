/*
 * Copyright (c) 2015-2016 Kees Bakker.  All rights reserved.
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

#ifndef _SODAQ_GSM_MODEM_h
#define _SODAQ_GSM_MODEM_h

#include <Arduino.h>
#include <stdint.h>
#include <Stream.h>
#include "Sodaq_OnOffBee.h"

// Network registration status.
enum NetworkRegistrationStatuses {
    UnknownNetworkRegistrationStatus = 0,
    Denied,
    NoNetworkRegistrationStatus,
    Home,
    Roaming,
};

// Network Technology.
enum NetworkTechnologies {
    UnknownNetworkTechnology = 0,
    GSM,
    EDGE,
    UTRAN,
    HSDPA,
    HSUPA,
    HSDPAHSUPA,
    LTE,
};

// SIM status.
enum SimStatuses {
    SimStatusUnknown = 0,
    SimMissing,
    SimNeedsPin,
    SimReady,
};

// TCP/UDP Protocol.
enum Protocols {
    TCP = 0,
    UDP,
};

// HTTP request type.
enum HttpRequestTypes {
    POST = 0,
    GET,
    HEAD,
    DELETE,
    PUT,
    HttpRequestTypesMAX = PUT,
};

// FTP mode.
enum FtpModes {
    ActiveMode = 0,
    PassiveMode,
};

// Response type (returned by readResponse() and parser methods).
enum ResponseTypes {
    ResponseNotFound = 0,
    ResponseOK = 1,
    ResponseError = 2,
    ResponsePrompt = 3,
    ResponseTimeout = 4,
    ResponseEmpty = 5,
};

// IP type
typedef uint32_t IP_t;

// callback for changing the baudrate of the modem stream.
typedef void (*BaudRateChangeCallbackPtr)(uint32_t newBaudrate);

#define DEFAULT_READ_MS 5000 // Used in readResponse()

#define NO_IP_ADDRESS ((IP_t)0)

#define IP_FORMAT "%d.%d.%d.%d"

#define IP_TO_TUPLE(x) (uint8_t)(((x) >> 24) & 0xFF), \
                       (uint8_t)(((x) >> 16) & 0xFF), \
                       (uint8_t)(((x) >> 8) & 0xFF), \
                       (uint8_t)(((x) >> 0) & 0xFF)

#define TUPLE_TO_IP(o1, o2, o3, o4) ((((IP_t)o1) << 24) | (((IP_t)o2) << 16) | \
                                     (((IP_t)o3) << 8) | (((IP_t)o4) << 0))

#define SOCKET_FAIL -1

class Sodaq_GSM_Modem {
public:
    // Constructor
    Sodaq_GSM_Modem();
    virtual ~Sodaq_GSM_Modem() {}

    // Sets the onoff instance
    void setOnOff(Sodaq_OnOffBee & onoff) { _onoff = &onoff; }

    // Turns the modem on and returns true if successful.
    bool on();

    // Turns the modem off and returns true if successful.
    bool off();

    // Sets the optional "Diagnostics and Debug" stream.
    void setDiag(Stream &stream) { _diagStream = &stream; }
    void setDiag(Stream *stream) { _diagStream = stream; }

    // Sets the size of the input buffer.
    // Needs to be called before init().
    void setInputBufferSize(size_t value) { this->_inputBufferSize = value; };

    // Store APN and user and password
    void setApn(const char *apn, const char *user = NULL, const char *pass = NULL);
    void setApnUser(const char *user);
    void setApnPass(const char *pass);

    // Store PIN
    void setPin(const char *pin);

    // Returns the default baud rate of the modem. 
    // To be used when initializing the modem stream for the first time.
    virtual uint32_t getDefaultBaudrate() = 0;

    // Enables the change of the baud rate to a higher speed when the modem is ready to do so.
    // Needs a callback in the main application to re-initialize the stream.
    void enableBaudrateChange(BaudRateChangeCallbackPtr callback) { _baudRateChangeCallbackPtr = callback; };

    // Sends the apn, apn username and apn password to the modem.
    virtual bool sendAPN(const char* apn, const char* username, const char* password) = 0;

    // Turns on and initializes the modem, then connects to the network and activates the data connection.
    virtual bool connect() = 0;

    // Disconnects the modem from the network.
    virtual bool disconnect() = 0;

    // Returns true if the modem is connected to the network and has an activated data connection.
    virtual bool isConnected() = 0;

    void setMinRSSI(int rssi) { _minRSSI = rssi; }
    void setMinCSQ(int csq) { _minRSSI = convertCSQ2RSSI(csq); }
    int8_t getMinRSSI() const { return _minRSSI; }
    uint8_t getCSQtime() const { return _CSQtime; }
    virtual int8_t convertCSQ2RSSI(uint8_t csq) const = 0;
    virtual uint8_t convertRSSI2CSQ(int8_t rssi) const = 0;

    int8_t getLastRSSI() const { return _lastRSSI; }

    // Returns the current status of the network.
    virtual NetworkRegistrationStatuses getNetworkStatus() = 0;

    // Returns the network technology the modem is currently registered to.
    virtual NetworkTechnologies getNetworkTechnology() = 0;

    // Gets the Operator Name.
    // Returns true if successful.
    virtual bool getOperatorName(char* buffer, size_t size) = 0;

    // Select the Best Operator.
    // Returns true if successful.
    virtual bool selectBestOperator(Stream & verbose_stream) = 0;

    // Select the an Operator (and measure RSSI).
    // Returns true if successful.
    virtual bool selectOperatorWithRSSI(const String & oper_long, const String & oper_num,
            int8_t & lastRSSI, Stream & verbose_stream) = 0;

    // Gets Mobile Directory Number.
    // Returns true if successful.
    virtual bool getMobileDirectoryNumber(char* buffer, size_t size) = 0;

    // Gets International Mobile Equipment Identity.
    // Should be provided with a buffer of at least 16 bytes.
    // Returns true if successful.
    virtual bool getIMEI(char* buffer, size_t size) = 0;

    // Gets Integrated Circuit Card ID.
    // Should be provided with a buffer of at least 21 bytes.
    // Returns true if successful.
    virtual bool getCCID(char* buffer, size_t size) = 0;

    // Gets the International Mobile Station Identity.
    // Should be provided with a buffer of at least 16 bytes.
    // Returns true if successful.
    virtual bool getIMSI(char* buffer, size_t size) = 0;

    // Returns the current SIM status.
    virtual SimStatuses getSimStatus() = 0;

    // Returns the local IP Address.
    virtual IP_t getLocalIP() = 0;

    // Returns the IP of the given host (nslookup).
    virtual IP_t getHostIP(const char* host) = 0;

    // Returns the sent and received counters
    virtual bool getSessionCounters(uint32_t* sent_count, uint32_t* recv_count) = 0;
    //virtual bool getTotalCounters(uint32_t* sent_count, uint32_t* recv_count) = 0;

    // ==== Sockets

    // Creates a new socket for the given protocol, optionally bound to the given localPort.
    // Returns the index of the socket created or -1 in case of error.
    virtual int createSocket(Protocols protocol, uint16_t localPort = 0) = 0;
    
    // Requests a connection to the given host and port, on the given socket.
    // Returns true if successful.
    virtual bool connectSocket(uint8_t socket, const char* host, uint16_t port) = 0;
    
    // Sends the given buffer through the given socket.
    // Returns true if successful.
    virtual bool socketSend(uint8_t socket, const uint8_t* buffer, size_t size) = 0;

    // Reads data from the given socket into the given buffer.
    // Returns the number of bytes written to the buffer.
    virtual size_t socketReceive(uint8_t socket, uint8_t* buffer, size_t size) = 0;

    // Returns the number of bytes pending in the read buffer of the given socket .
    virtual size_t socketBytesPending(uint8_t socket) = 0;

    // Closes the given socket.
    // Returns true if successful.
    virtual bool closeSocket(uint8_t socket) = 0;

    // ==== TCP

    // Open a TCP connection
    // This is merely a convenience wrapper which can use socket functions.
    virtual bool openTCP(const char *apn, const char *apnuser, const char *apnpwd,
            const char *server, int port, bool transMode=false) = 0;

    // Close the TCP connection
    // This is merely a convenience wrapper which can use socket functions.
    virtual void closeTCP(bool switchOff=true) = 0;

    // Send data via TCP
    // This is merely a convenience wrapper which can use socket functions.
    virtual bool sendDataTCP(const uint8_t *data, size_t data_len) = 0;

    // Receive data via TCP
    // This is merely a convenience wrapper which can use socket functions.
    virtual bool receiveDataTCP(uint8_t *data, size_t data_len, uint16_t timeout=4000) = 0;

    // Set a handler to be called when URC
    void setTCPClosedHandler(void (*handler)(void)) { _tcpClosedHandler = handler; }

    // ==== HTTP

    // Creates an HTTP request using the (optional) given buffer and 
    // (optionally) returns the received data.
    // endpoint should include the initial "/".
    virtual size_t httpRequest(const char* server, uint16_t port, const char* endpoint,
            HttpRequestTypes requestType = GET,
            char* responseBuffer = NULL, size_t responseSize = 0,
            const char* sendBuffer = NULL, size_t sendSize = 0) = 0;

    // ==== FTP

    // Opens an FTP connection.
    virtual bool openFtpConnection(const char* server, const char* username, const char* password, FtpModes ftpMode) = 0;
    
    // Closes the FTP connection.
    virtual bool closeFtpConnection() = 0;

    // Opens an FTP file for sending or receiving.
    // filename should be limited to 256 characters (excl. null terminator)
    // path should be limited to 512 characters (excl. null temrinator)
    virtual bool openFtpFile(const char* filename, const char* path = NULL) = 0;

    // Sends the given "buffer" to the (already) open FTP file.
    // Returns true if successful.
    // Fails immediatelly if there is no open FTP file.
    virtual bool ftpSend(const char* buffer) = 0;
    virtual bool ftpSend(const uint8_t* buffer, size_t size) = 0;

    // Fills the given "buffer" from the (already) open FTP file.
    // Returns true if successful.
    // Fails immediatelly if there is no open FTP file.
    virtual int ftpReceive(char* buffer, size_t size) = 0;

    // Closes the open FTP file.
    // Returns true if successful.
    // Fails immediatelly if there is no open FTP file.
    virtual bool closeFtpFile() = 0;

    // ==== SMS
    
    // Gets an SMS list according to the given filter and puts the indexes in the "indexList".
    // Returns the number of indexes written to the list or -1 in case of error.
    virtual int getSmsList(const char* statusFilter = "ALL", int* indexList = NULL, size_t size = 0) = 0;
    
    // Reads an SMS from the given index and writes it to the given buffer.
    // Returns true if successful.
    virtual bool readSms(uint8_t index, char* phoneNumber, char* buffer, size_t size) = 0;
    
    // Deletes the SMS at the given index.
    virtual bool deleteSms(uint8_t index) = 0;

    // Sends a text-mode SMS.
    // Expects a null-terminated buffer.
    // Returns true if successful.
    virtual bool sendSms(const char* phoneNumber, const char* buffer) = 0;

    // MQTT (using this class as a transport)
    virtual bool openMQTT(const char * server, uint16_t port = 1883) = 0;
    virtual bool closeMQTT(bool switchOff=true) = 0;
    virtual bool sendMQTTPacket(uint8_t * pckt, size_t len) = 0;
    virtual size_t receiveMQTTPacket(uint8_t * pckt, size_t size, uint32_t timeout = 20000) = 0;
    virtual size_t availableMQTTPacket() = 0;
    virtual bool isAliveMQTT() = 0;

protected:
    // The stream that communicates with the device.
    Stream* _modemStream;

    // The (optional) stream to show debug information.
    Stream* _diagStream;
    bool _disableDiag;

    // The size of the input buffer. Equals SODAQ_GSM_MODEM_DEFAULT_INPUT_BUFFER_SIZE
    // by default or (optionally) a user-defined value when using USE_DYNAMIC_BUFFER.
    size_t _inputBufferSize;

    // Flag to make sure the buffers are not allocated more than once.
    bool _isBufferInitialized;

    // The buffer used when reading from the modem. The space is allocated during init() via initBuffer().
    char* _inputBuffer;

    char * _apn;
    char * _apnUser;
    char * _apnPass;

    char * _pin;

    // The on-off pin power controller object.
    Sodaq_OnOffBee* _onoff;

    // The callback for requesting baudrate change of the modem stream.
    BaudRateChangeCallbackPtr _baudRateChangeCallbackPtr;

    // This flag keeps track if the next write is the continuation of the current command
    // A Carriage Return will reset this flag.
    bool _appendCommand;

    // This is the value of the most recent CSQ
    // Notice that CSQ is somewhat standard. SIM800/SIM900 and Ublox
    // compute to comparable numbers. With minor deviations.
    // For example SIM800
    //   1              -111 dBm
    //   2...30         -110... -54 dBm
    // For example UBlox
    //   1              -111 dBm
    //   2..30          -109 to -53 dBm
    int8_t _lastRSSI;   // 0 not known or not detectable

    // This is the number of second it took when CSQ was record last
    uint8_t _CSQtime;

    // This is the minimum required RSSI to continue making the connection
    // Use convertCSQ2RSSI if you have a CSQ value
    int _minRSSI;

    // Keep track if ATE0 was sent
    bool _echoOff;

    // Keep track when connect started. Use this to record various status changes.
    uint32_t _startOn;

    // A call-back function to be called when the TCP is closed by the remote
    // Usually this comes in via URC's
    void (*_tcpClosedHandler)(void);

    // Initializes the input buffer and makes sure it is only initialized once.
    // Safe to call multiple times.
    void initBuffer();

    // Returns true if the modem is ON (and replies to "AT" commands without timing out)
    virtual bool isAlive() = 0;

    // Returns true if the modem is on.
    bool isOn() const;

    virtual void switchEchoOff() = 0;

    // Sets the modem stream.
    void setModemStream(Stream& stream);

    // Returns a character from the modem stream if read within _timeout ms or -1 otherwise.
    int timedRead(uint32_t timeout = 1000) const;

    // Fills the given "buffer" with characters read from the modem stream up to "length"
    // maximum characters and until the "terminator" character is found or a character read
    // times out (whichever happens first).
    // The buffer does not contain the "terminator" character or a null terminator explicitly.
    // Returns the number of characters written to the buffer, not including null terminator.
    size_t readBytesUntil(char terminator, char* buffer, size_t length, uint32_t timeout = 1000);

    // Fills the given "buffer" with up to "length" characters read from the modem stream.
    // It stops when a character read times out or "length" characters have been read.
    // Returns the number of characters written to the buffer.
    size_t readBytes(uint8_t* buffer, size_t length, uint32_t timeout = 1000);

    // Reads a line from the modem stream into the "buffer". The line terminator is not
    // written into the buffer. The buffer is terminated with null.
    // Returns the number of bytes read, not including the null terminator.
    size_t readLn(char* buffer, size_t size, uint32_t timeout = 1000);

    // Reads a line from the modem stream into the input buffer.
    // Returns the number of bytes read.
    size_t readLn() { return readLn(_inputBuffer, _inputBufferSize); };

    // Write a byte
    size_t writeByte(uint8_t value);

    // Write the command prolog (just for debugging
    void writeProlog();

    size_t print(const __FlashStringHelper *);
    size_t print(const String &);
    size_t print(const char[]);
    size_t print(char);
    size_t print(unsigned char, int = DEC);
    size_t print(int, int = DEC);
    size_t print(unsigned int, int = DEC);
    size_t print(long, int = DEC);
    size_t print(unsigned long, int = DEC);
    size_t print(double, int = 2);
    size_t print(const Printable&);

    size_t println(const __FlashStringHelper *);
    size_t println(const String &s);
    size_t println(const char[]);
    size_t println(char);
    size_t println(unsigned char, int = DEC);
    size_t println(int, int = DEC);
    size_t println(unsigned int, int = DEC);
    size_t println(long, int = DEC);
    size_t println(unsigned long, int = DEC);
    size_t println(double, int = 2);
    size_t println(const Printable&);
    size_t println(void);

    virtual ResponseTypes readResponse(char* buffer, size_t size, size_t* outSize, uint32_t timeout = DEFAULT_READ_MS) = 0;
};

#endif
