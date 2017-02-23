#ifndef _SODAQ_3GBEE_h
#define _SODAQ_3GBEE_h

#include <Arduino.h>
#include <stdint.h>
#include <stddef.h>
#include <Stream.h>
#include "Sodaq_GSM_Modem.h"

#define SOCKET_COUNT 7

enum TriBoolStates
{
    TriBoolFalse,
    TriBoolTrue,
    TriBoolUndefined
};

typedef TriBoolStates tribool_t;

// Packet Switch Data (PSD) authorization type.
enum PSDAuthType_e {
    PAT_TryAll = -1,                // This is not a UBlox number. Just our own.
    PAT_None = 0,
    PAT_PAP = 1,
    PAT_CHAP = 2,
    PAT_AutoSelect = 3
};

typedef ResponseTypes (*CallbackMethodPtr)(ResponseTypes& response, const char* buffer, size_t size,
        void* parameter, void* parameter2);

class Sodaq_3Gbee: public Sodaq_GSM_Modem {
public:
    Sodaq_3Gbee();
    ~Sodaq_3Gbee() {}

    // Returns true if the modem replies to "AT" commands without timing out.
    bool isAlive();

    // Returns the default baud rate of the modem. 
    // To be used when initializing the modem stream for the first time.
    uint32_t getDefaultBaudrate() { return 9600; };

    // Initializes the modem instance. Sets the modem stream and the on-off power pins.
    void init(Stream& stream, int8_t vcc33Pin, int8_t onoffPin, int8_t statusPin);
    void init_wdt(Stream& stream, int8_t onoffPin);

    // Set authentication of PSD profile (via AT+UPSD=<profile>,6,<num>)
    void setPSDAuth(PSDAuthType_e authType) { _psdAuthType = authType; }
    PSDAuthType_e numToPSDAuthType(int8_t i);

    // Turns on and initializes the modem, then connects to the network and activates the data connection.
    bool connect();

    // Turns on and initializes the modem.
    bool connectSimple();

    // Disconnects the modem from the network.
    bool disconnect();

    // Returns true if the modem is connected to the network and has an activated data connection.
    bool isConnected();

    // Returns the current status of the network.
    NetworkRegistrationStatuses getNetworkStatus();

    // Returns the network technology the modem is currently registered to.
    NetworkTechnologies getNetworkTechnology();

    // Gets the Received Signal Strength Indication in dBm and Bit Error Rate.
    // Returns true if successful.
    bool getRSSIAndBER(int8_t* rssi, uint8_t* ber);
    int8_t convertCSQ2RSSI(uint8_t csq) const;
    uint8_t convertRSSI2CSQ(int8_t rssi) const;

    // Gets the Operator Name.
    // Returns true if successful.
    bool getOperatorName(char* buffer, size_t size);

    // Select the Best Operator.
    // Returns true if successful.
    bool selectBestOperator(Stream & verbose_stream);

    // Select the an Operator (and measure RSSI).
    // Returns true if successful.
    bool selectOperatorWithRSSI(const String & oper_long, const String & oper_num, int8_t & lastRSSI, Stream & verbose_stream);

    // Gets Mobile Directory Number.
    // Returns true if successful.
    bool getMobileDirectoryNumber(char* buffer, size_t size);

    // Gets International Mobile Equipment Identity.
    // Should be provided with a buffer of at least 16 bytes.
    // Returns true if successful.
    bool getIMEI(char* buffer, size_t size);

    // Gets Integrated Circuit Card ID.
    // Should be provided with a buffer of at least 21 bytes.
    // Returns true if successful.
    bool getCCID(char* buffer, size_t size);

    // Gets the International Mobile Station Identity.
    // Should be provided with a buffer of at least 16 bytes.
    // Returns true if successful.
    bool getIMSI(char* buffer, size_t size);

    // Returns the current SIM status.
    SimStatuses getSimStatus();

    // Returns the local IP Address.
    IP_t getLocalIP();

    // Returns the IP of the given host (nslookup).
    IP_t getHostIP(const char* host);

    // Returns the sent and received counters
    bool getSessionCounters(uint32_t* sentCnt, uint32_t* recvCnt);
    //bool getTotalCounters(uint32_t* sentCnt, uint32_t* recvCnt);

    // ==== Sockets

    // Creates a new socket for the given protocol, optionally bound to the given localPort.
    // Returns the index of the socket created or -1 in case of error.
    int createSocket(Protocols protocol, uint16_t localPort = 0);

    // Requests a connection to the given host and port, on the given socket.
    // Returns true if successful.
    bool connectSocket(uint8_t socket, const char* host, uint16_t port);

    // Sends the given buffer through the given socket.
    // Returns true if successful.
    bool socketSend(uint8_t socket, const uint8_t* buffer, size_t size);

    // Sends the given string through the given socket.
    // Returns true if successful.
    bool socketSend(uint8_t socket, const char* str);

    // Reads data from the given socket into the given buffer.
    // Returns the number of bytes written to the buffer.
    // NOTE: if the modem hasn't reported available data, it blocks for up to 10 seconds waiting.
    size_t socketReceive(uint8_t socket, uint8_t* buffer, size_t size);

    // Returns the number of bytes pending in the read buffer of the given socket .
    size_t socketBytesPending(uint8_t socket);

    // Closes the given socket.
    // Returns true if successful.
    bool closeSocket(uint8_t socket);

    // Blocks waiting for the given socket to be reported closed.
    // This method should be called only after closeSocket() or when the remote is expected to close the socket.
    // Times out after 60 seconds.
    // TODO Figure out what a good timeout value is. 60 seconds is very long.
    void waitForSocketClose(uint8_t socket, uint32_t timeout=60000);

    // Make sure output is acknowledged by the server when doing socketSend
    void setFlushEverySend(bool x = true) { _flushEverySend = x; }

    // ==== TCP

    // Open a TCP connection
    // These are merely a convenience wrapper which can use socket functions.
    bool openTCP(const char *apn, const char *apnuser, const char *apnpwd,
            const char *server, int port, bool transMode=false);
    bool openTCP(const char *server, int port, bool transMode=false);

    // Close the TCP connection
    // This is merely a convenience wrapper which can use socket functions.
    void closeTCP(bool switchOff=true);

    // Send data via TCP
    // This is merely a convenience wrapper which can use socket functions.
    bool sendDataTCP(const uint8_t *data, size_t data_len);

    // Receive data via TCP
    // This is merely a convenience wrapper which can use socket functions.
    bool receiveDataTCP(uint8_t *data, size_t data_len, uint16_t timeout=4000);

    // ==== HTTP

    // Creates an HTTP request using the (optional) given buffer and 
    // (optionally) returns the received data.
    // endpoint should include the initial "/".
    size_t httpRequest(const char* server, uint16_t port, const char* endpoint, HttpRequestTypes requestType = GET, char* responseBuffer = NULL, size_t responseSize = 0, const char* sendBuffer = NULL, size_t sendSize = 0);

    // ==== Ftp

    // Opens an FTP connection.
    bool openFtpConnection(const char* server, const char* username, const char* password, FtpModes ftpMode);
    
    // Closes the FTP connection.
    bool closeFtpConnection();

    // Opens an FTP file for sending or receiving.
    // filename should be limited to 256 characters (excl. null terminator)
    // path should be limited to 512 characters (excl. null temrinator)
    bool openFtpFile(const char* filename, const char* path = NULL);
    
    // Sends the given "buffer" to the (already) open FTP file.
    // Returns true if successful.
    // Fails immediatelly if there is no open FTP file.
    bool ftpSend(const char* buffer);
    bool ftpSend(const uint8_t* buffer, size_t size);

    // Fills the given "buffer" from the (already) open FTP file.
    // Returns true if successful.
    // Fails immediatelly if there is no open FTP file.
    int ftpReceive(char* buffer, size_t size);
    
    // Closes the open FTP file.
    // Returns true if successful.
    // Fails immediatelly if there is no open FTP file.
    bool closeFtpFile();
    
    // ==== Sms

    // Gets an SMS list according to the given filter and puts the indexes in the "indexList".
    // Returns the number of indexes written to the list or -1 in case of error.
    int getSmsList(const char* statusFilter, int* indexList, size_t size);

    // Reads an SMS from the given index and writes it to the given buffer.
    // Returns true if successful.
    bool readSms(uint8_t index, char* phoneNumber, char* buffer, size_t size);

    // Deletes the SMS at the given index.
    bool deleteSms(uint8_t index);

    // Sends a text-mode SMS.
    // Expects a null-terminated buffer.
    // Returns true if successful.
    bool sendSms(const char* phoneNumber, const char* buffer);

    // MQTT (using this class as a transport)
    bool openMQTT(const char * server, uint16_t port = 1883);
    bool closeMQTT(bool switchOff=true);
    bool sendMQTTPacket(uint8_t * pckt, size_t len);
    size_t receiveMQTTPacket(uint8_t * pckt, size_t size, uint32_t timeout = 20000);
    size_t availableMQTTPacket();
    bool isAliveMQTT();

    size_t readFile(const char* filename, uint8_t* buffer, size_t size);

    bool writeFile(const char* filename, const uint8_t* buffer, size_t size);
    
    bool deleteFile(const char* filename);

    void clearCachedHostIp() { _host_ip = NO_IP_ADDRESS; }

    // Getters of diagnostic values
    uint32_t getTimeToSocketConnect() { return _timeToSocketConnect; }
    uint32_t getTimeToSocketClose() { return _timeToSocketClose; }

    // Selecting the best network
    bool deregisterNetwork(uint32_t timeout);
    bool getOperators(String & listOfOperators);
    bool getNthOperator(const String & listOfOperators, size_t nth, String & oper_long, String & oper_num, size_t & status);
    bool selectOperator(const String & oper_long, uint32_t timeout);
    bool selectOperatorNum(const String & oper_num, uint32_t timeout);

protected:
    // Sets the apn, apn username and apn password to the modem.
    bool sendAPN(const char* apn, const char* username, const char* password);

    // override
    ResponseTypes readResponse(char* buffer, size_t size, size_t* outSize, uint32_t timeout = DEFAULT_READ_MS)
    {
        return readResponse(_inputBuffer, _inputBufferSize, NULL, NULL, NULL, outSize, timeout);
    };

    ResponseTypes readResponse(char* buffer, size_t size,
            CallbackMethodPtr parserMethod, void* callbackParameter, void* callbackParameter2 = NULL,
            size_t* outSize = NULL, uint32_t timeout = DEFAULT_READ_MS);
    
    ResponseTypes readResponse(size_t* outSize = NULL, uint32_t timeout = DEFAULT_READ_MS)
    {
        return readResponse(_inputBuffer, _inputBufferSize, NULL, NULL, NULL, outSize, timeout);
    };

    ResponseTypes readResponse(CallbackMethodPtr parserMethod, void* callbackParameter,
        void* callbackParameter2 = NULL, size_t* outSize = NULL, uint32_t timeout = DEFAULT_READ_MS)
    {
        return readResponse(_inputBuffer, _inputBufferSize,
                parserMethod, callbackParameter, callbackParameter2,
                outSize, timeout);
    };

    template<typename T1, typename T2>
    ResponseTypes readResponse(ResponseTypes(*parserMethod)(ResponseTypes& response, const char* parseBuffer, size_t size, T1* parameter, T2* parameter2),
        T1* callbackParameter, T2* callbackParameter2,
        size_t* outSize = NULL, uint32_t timeout = DEFAULT_READ_MS)
    {
        return readResponse(_inputBuffer, _inputBufferSize, (CallbackMethodPtr)parserMethod, 
            (void*)callbackParameter, (void*)callbackParameter2, outSize, timeout);
    };

private:
    PSDAuthType_e _psdAuthType;

    uint16_t _socketPendingBytes[SOCKET_COUNT]; // TODO add getter
    tribool_t _httpRequestSuccessBit[HttpRequestTypesMAX];
    bool _socketClosedBit[SOCKET_COUNT];
    uint8_t ftpCommandURC[2];
    char ftpFilename[256 + 1]; // always null terminated
    uint8_t ftpDirectoryChangeCounter; // counts how many nested directories were changed, to revert on close
    int _openTCPsocket;

    uint32_t _timeToSocketConnect;
    uint32_t _timeToSocketClose;

    bool _foundUUPSDD;

    IP_t _host_ip;
    char _host_name[20];        // an arbitrary size, see getHostIP() for details

    bool _flushEverySend;

    bool tryAuthAndActivate(PSDAuthType_e authType);

    static bool startsWith(const char* pre, const char* str);
    static size_t ipToString(IP_t ip, char* buffer, size_t size);
    static bool isValidIPv4(const char* str);
    bool setSimPin(const char* simPin);

    void switchEchoOff();
    bool doInitialCommands();
    bool doSIMcheck();
    bool enableAutoRegistration(uint32_t timeout = 4L * 60 * 1000);
    bool waitForSignalQuality(uint32_t timeout = 60L * 1000);

    bool setBinaryMode();
    bool setHexMode();

    // Wait until no more un-acknowledged data in output
    // Return true if no more data, false if error, or timeout
    bool waitForSocketOutput(uint8_t socket, uint32_t timeout=10000);

    // returns true if URC returns 1, false in case URC returns 0 or in case of timeout
    bool waitForFtpCommandResult(uint8_t ftpCommandIndex, uint32_t timeout=10000);
    bool changeFtpDirectory(const char* directory);
    void resetFtpDirectoryIfNeeded();

    bool waitForDeactivatedNetwork(uint32_t timeout);

    void cleanupTempFiles();

    static int _httpRequestTypeToModemIndex(HttpRequestTypes requestType);
    static int _httpModemIndexToRequestType(uint8_t modemIndex);

    // ==== Parser Methods
    static ResponseTypes _cpinParser(ResponseTypes& response, const char* buffer, size_t size, SimStatuses* simStatusResult, uint8_t* dummy);
    static ResponseTypes _udnsrnParser(ResponseTypes& response, const char* buffer, size_t size, IP_t* ipResult, uint8_t* dummy);
    static ResponseTypes _upsndParser(ResponseTypes& response, const char* buffer, size_t size, IP_t* ipResult, uint8_t* dummy);
    static ResponseTypes _upsndParser(ResponseTypes& response, const char* buffer, size_t size, uint8_t* thirdParam, uint8_t* dummy);
    static ResponseTypes _usocrParser(ResponseTypes& response, const char* buffer, size_t size, uint8_t* socket, uint8_t* dummy);
    static ResponseTypes _usordParser(ResponseTypes& response, const char* buffer, size_t size, char* resultBuffer, uint8_t* dummy);
    static ResponseTypes _usoctlParser(ResponseTypes& response, const char* buffer, size_t size, uint16_t* result, uint8_t* dummy);
    static ResponseTypes _copsParser(ResponseTypes& response, const char* buffer, size_t size, char* operatorNameBuffer, size_t* operatorNameBufferSize);
    static ResponseTypes _copsParser(ResponseTypes& response, const char* buffer, size_t size, int* networkTechnology, uint8_t* dummy);
    static ResponseTypes _csqParser(ResponseTypes& response, const char* buffer, size_t size, int* rssi, int* ber);
    static ResponseTypes _cnumParser(ResponseTypes& response, const char* buffer, size_t size, char* numberBuffer, size_t* numberBufferSize);
    static ResponseTypes _nakedStringParser(ResponseTypes& response, const char* buffer, size_t size, char* stringBuffer, size_t* stringBufferSize);
    static ResponseTypes _ccidParser(ResponseTypes& response, const char* buffer, size_t size, char* ccidBuffer, size_t* ccidBufferSize);
    static ResponseTypes _cregParser(ResponseTypes& response, const char* buffer, size_t size, int* networkStatus, uint8_t* dummy);
    static ResponseTypes _ulstfileParser(ResponseTypes& response, const char* buffer, size_t size, uint32_t* filesize, uint8_t* dummy);
    static ResponseTypes _cmgrParser(ResponseTypes& response, const char* buffer, size_t size, char* phoneNumber, char* smsBuffer);
    static ResponseTypes _cmglParser(ResponseTypes& response, const char* buffer, size_t size, int* indexList, size_t* indexListSize);
    static ResponseTypes _ugcntrdParser(ResponseTypes& response, const char* buffer, size_t size, uint32_t* sentCnt, uint32_t* recvCnt);

    String getValueAt(String data, char separator, int index);
};

extern Sodaq_3Gbee sodaq_3gbee;

#endif
