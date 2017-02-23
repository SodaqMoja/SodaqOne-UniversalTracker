#include <Arduino.h>
#include "Sodaq_wdt.h"

#include "Sodaq_3Gbee.h"

#define DEBUG

#define STR_AT "AT"
#define STR_RESPONSE_OK "OK"
#define STR_RESPONSE_ERROR "ERROR"
#define STR_RESPONSE_CME_ERROR "+CME ERROR:"
#define STR_RESPONSE_CMS_ERROR "+CMS ERROR:"
#define STR_RESPONSE_SOCKET_PROMPT "@"
#define STR_RESPONSE_SMS_PROMPT ">"
#define STR_RESPONSE_FILE_PROMPT ">"

#define DEBUG_STR_ERROR "[ERROR]: "

#define NIBBLE_TO_HEX_CHAR(i) ((i <= 9) ? ('0' + i) : ('A' - 10 + i))
#define HIGH_NIBBLE(i) ((i >> 4) & 0x0F)
#define LOW_NIBBLE(i) (i & 0x0F)

#define HEX_CHAR_TO_NIBBLE(c) ((c >= 'A') ? (c - 'A' + 0x0A) : (c - '0'))
#define HEX_PAIR_TO_BYTE(h, l) ((HEX_CHAR_TO_NIBBLE(h) << 4) + HEX_CHAR_TO_NIBBLE(l))

#define STR_HELPER(x) #x
#define STR(x) STR_HELPER(x)

#define ARRAY_SIZE(a) (sizeof(a)/sizeof(a[0]))

#ifdef DEBUG
#define debugPrintLn(...) { if (!this->_disableDiag && this->_diagStream) this->_diagStream->println(__VA_ARGS__); }
#define debugPrint(...) { if (!this->_disableDiag && this->_diagStream) this->_diagStream->print(__VA_ARGS__); }
#warning "Debug mode is ON"
#else
#define debugPrintLn(...)
#define debugPrint(...)
#endif

#define BLOCK_TIMEOUT -1
#define DEFAULT_PROFILE "0"
#define HIGH_BAUDRATE 57600
#define MAX_SOCKET_BUFFER 512
#define DEFAULT_HTTP_SEND_TMP_FILENAME "http_tmp_put_0"
#define DEFAULT_HTTP_RECEIVE_TMP_FILENAME "http_last_response_0"
#define DEFAULT_FTP_TMP_FILENAME "ftp_tmp_file"
#define CTRL_Z '\x1A'

#define NOW (uint32_t)millis()

static inline bool is_timedout(uint32_t from, uint32_t nr_ms) __attribute__((always_inline));
static inline bool is_timedout(uint32_t from, uint32_t nr_ms)
{
    return (millis() - from) > nr_ms;
}


// A specialized class to switch on/off the 3Gbee module
// The VCC3.3 pin is switched by the Autonomo BEE_VCC pin
// The DTR pin is the actual ON/OFF pin, it is A13 on Autonomo, D20 on Tatu
//
// This can be used for WDT too, but statusPin is not available.
class Sodaq_3GbeeOnOff : public Sodaq_OnOffBee
{
public:
    Sodaq_3GbeeOnOff();
    void init(int vcc33Pin, int onoffPin, int statusPin = -1);
    void on();
    void off();
    bool isOn();
private:
    int8_t _vcc33Pin;
    int8_t _onoffPin;
    int8_t _statusPin;
    bool _onoff_status;
};

static Sodaq_3GbeeOnOff sodaq_3gbee_onoff;

Sodaq_3Gbee sodaq_3gbee;

Sodaq_3Gbee::Sodaq_3Gbee()
{
    _psdAuthType = PAT_None;
    ftpDirectoryChangeCounter = 0;
    _openTCPsocket = -1;
    _timeToSocketConnect = 0;
    _timeToSocketClose = 0;
    _host_ip = NO_IP_ADDRESS;
    _host_name[0] = 0;
    _echoOff = false;
    _flushEverySend = false;
    _foundUUPSDD = false;
}

bool Sodaq_3Gbee::startsWith(const char* pre, const char* str)
{
    return (strncmp(pre, str, strlen(pre)) == 0);
}

/*!
 * Read the next response from the modem
 *
 * Notice that we're collecting URC's here. And in the process we could
 * be updating:
 *     _socketPendingBytes[] if +UUSORD: is seen
 *     _socketClosedBit[] if +UUSOCL: is seen
 *     _httpRequestSuccessBit[] if +UUHTTPCR: is seen
 *     ftpCommandURC[] if +UUFTPCR: is seen
 */
ResponseTypes Sodaq_3Gbee::readResponse(char* buffer, size_t size,
        CallbackMethodPtr parserMethod, void* callbackParameter, void* callbackParameter2,
        size_t* outSize, uint32_t timeout)
{
    ResponseTypes response = ResponseNotFound;
    uint32_t from = NOW;

    do {
        // 250ms,  how many bytes at which baudrate?
        int count = readLn(buffer, size, 250);
        sodaq_wdt_reset();

        if (count > 0) {
            if (outSize) {
                *outSize = count;
            }
            if (_disableDiag && strncmp(buffer, "OK", 2) != 0) {
                _disableDiag = false;
            }

            debugPrint("[rdResp]: ");
            debugPrintLn(buffer);

            // handle unsolicited codes

            int param1, param2;
            if (sscanf(buffer, "+UUSORD: %d,%d", &param1, &param2) == 2) {
                uint16_t socket_nr = param1;
                uint16_t nr_bytes = param2;
                debugPrint("Unsolicited: Socket ");
                debugPrint(socket_nr);
                debugPrint(": ");
                debugPrint(param2);
                debugPrintLn(" bytes pending");
                if (socket_nr < ARRAY_SIZE(_socketPendingBytes)) {
                    _socketPendingBytes[socket_nr] = nr_bytes;
                }
                continue;
            }
            else if (sscanf(buffer, "+UUSOCL: %d", &param1) == 1) {
                uint16_t socket_nr = param1;
                if (socket_nr < ARRAY_SIZE(_socketPendingBytes)) {
                    debugPrint("Unsolicited: Socket ");
                    debugPrint(socket_nr);
                    debugPrint(": ");
                    debugPrintLn("closed by remote");

                    _socketClosedBit[socket_nr] = true;
                    if (socket_nr == _openTCPsocket) {
                        _openTCPsocket = -1;
                        // Report this other software layers
                        if (_tcpClosedHandler) {
                            _tcpClosedHandler();
                        }
                    }
                }
                continue;
            }
            else if (sscanf(buffer, "+UUHTTPCR: 0, %d, %d", &param1, &param2) == 2) {
                int requestType = _httpModemIndexToRequestType(static_cast<uint8_t>(param1));
                if (requestType >= 0) {
                    debugPrint("HTTP Result for request type ");
                    debugPrint(requestType);
                    debugPrint(": ");
                    debugPrintLn(param2);

                    if (param2 == 0) {
                        _httpRequestSuccessBit[requestType] = TriBoolFalse;
                    }
                    else if (param2 == 1) {
                        _httpRequestSuccessBit[requestType] = TriBoolTrue;
                    }
                } else {
                    // Unknown type
                }
                continue;
            }
            else if (sscanf(buffer, "+UUFTPCR: %d, %d", &param1, &param2) == 2) {
                debugPrint("FTP Result for command ");
                debugPrint(param1);
                debugPrint(": ");
                debugPrintLn(param2);

                ftpCommandURC[0] = static_cast<uint8_t>(param1);
                ftpCommandURC[1] = static_cast<uint8_t>(param2);
                continue;
            }
            else if (sscanf(buffer, "+UUPSDD: %d", &param1) == 1) {
                debugPrint("UUPSDD profile: ");
                debugPrintLn(param1);
                // Ignore profile
                _foundUUPSDD = true;
                continue;
            }

            // ignore the Network Selection Control +PACSP URC
            if (startsWith("+PACSP", buffer)) {
              continue;
            }

            if (startsWith(STR_AT, buffer)) {
                continue; // skip echoed back command
            }

            _disableDiag = false;
            if (startsWith(STR_RESPONSE_OK, buffer)) {
                return ResponseOK;
            }

            if (startsWith(STR_RESPONSE_ERROR, buffer) ||
                    startsWith(STR_RESPONSE_CME_ERROR, buffer) ||
                    startsWith(STR_RESPONSE_CMS_ERROR, buffer)) {
                return ResponseError;
            }

            if (startsWith(STR_RESPONSE_SOCKET_PROMPT, buffer) ||
                    startsWith(STR_RESPONSE_SMS_PROMPT, buffer) ||
                    startsWith(STR_RESPONSE_FILE_PROMPT, buffer)) {
                return ResponsePrompt;
            }

            if (parserMethod) {
                ResponseTypes parserResponse = parserMethod(response, buffer, count, callbackParameter, callbackParameter2);
                if (parserResponse != ResponseEmpty) {
                    return parserResponse;
                } else {
                    // ?
                    // ResponseEmpty indicates that the parser was satisfied
                    // Continue until "OK", "ERROR", or whatever else.
                }
                // Prevent calling the parser again.
                // This could happen if the input line is too long. It will be split
                // and the next readLn will return the next part.
                parserMethod = 0;
            }

            // at this point, the parserMethod has ran and there is no override response from it,
            // so if there is some other response recorded, return that
            // (otherwise continue iterations until timeout)
            if (response != ResponseNotFound) {
                debugPrintLn("** response != ResponseNotFound");
                return response;
            }
        }

        delay(10);      // TODO Why do we need this delay?
    } while (!is_timedout(from, timeout));

    if (outSize) {
        *outSize = 0;
    }

    debugPrintLn("[rdResp]: timed out");
    return ResponseTimeout;
}

bool Sodaq_3Gbee::setSimPin(const char* simPin)
{
    print("AT+CPIN=\"");
    print(simPin);
    println("\"");

    return (readResponse() == ResponseOK);
}

bool Sodaq_3Gbee::setBinaryMode()
{
    println("AT+UDCONF=1,0");

    return (readResponse() == ResponseOK);
}

bool Sodaq_3Gbee::setHexMode()
{
    println("AT+UDCONF=1,1");

    return (readResponse() == ResponseOK);
}

// Returns true if the modem is connected to the network and has an activated data connection.
bool Sodaq_3Gbee::isConnected()
{
    uint8_t value = 0;

    println("AT+UPSND=" DEFAULT_PROFILE ",8");
    if (readResponse<uint8_t, uint8_t>(_upsndParser, &value, NULL) == ResponseOK) {
        return (value == 1);
    }

    return false;
}

bool Sodaq_3Gbee::waitForFtpCommandResult(uint8_t ftpCommandIndex, uint32_t timeout)
{
    uint32_t start = millis();

    ftpCommandURC[0] = 0xFF; // set to unused value to clear (0 is used)
    ftpCommandURC[1] = 0;
    while (ftpCommandURC[0] != ftpCommandIndex && !is_timedout(start, timeout)) {
        isAlive();
        delay(5);
    }

    return (ftpCommandURC[0] == ftpCommandIndex) && (ftpCommandURC[1] == 1);
}

bool Sodaq_3Gbee::changeFtpDirectory(const char* directory)
{
    print("AT+UFTPC=8,\"");
    print(directory);
    println("\"");

    if ((readResponse() != ResponseOK) || (!waitForFtpCommandResult(8))) {
        return false;
    }

    if (strcmp("..", directory)) {
        ftpDirectoryChangeCounter--;
    }
    else {
        ftpDirectoryChangeCounter++;
    }

    return true;
}

void Sodaq_3Gbee::resetFtpDirectoryIfNeeded()
{
    uint8_t maxTries = 2 * ftpDirectoryChangeCounter;
    while (ftpDirectoryChangeCounter > 0 && maxTries > 0) {
        changeFtpDirectory("..");
        maxTries--;
    }
}

void Sodaq_3Gbee::cleanupTempFiles()
{
    deleteFile(DEFAULT_FTP_TMP_FILENAME);
    deleteFile(DEFAULT_HTTP_RECEIVE_TMP_FILENAME);
    deleteFile(DEFAULT_HTTP_SEND_TMP_FILENAME);
}

// Returns true if the modem replies to "AT" commands without timing out.
bool Sodaq_3Gbee::isAlive()
{
    _disableDiag = true;
    println(STR_AT);

    return (readResponse(NULL, 450) == ResponseOK);
}

// Sets the apn, apn username and apn password to the modem.
bool Sodaq_3Gbee::sendAPN(const char* apn, const char* username, const char* password)
{
    print("AT+UPSD=" DEFAULT_PROFILE ",1,\"");
    print(apn);
    println("\"");

    if (readResponse() != ResponseOK) {
        return false;
    }

    if (username && *username) {
        print("AT+UPSD=" DEFAULT_PROFILE ",2,\"");
        print(username);
        println("\"");

        if (readResponse() != ResponseOK) {
            return false;
        }

        if (password && *password) {
            print("AT+UPSD=" DEFAULT_PROFILE ",3,\"");
            print(password);
            println("\"");

            if (readResponse() != ResponseOK) {
                return false;
            }
        }
    }

    return true;
}

// Initializes the modem instance. Sets the modem stream and the on-off power pins.
void Sodaq_3Gbee::init(Stream& stream, int8_t vcc33Pin, int8_t onoffPin, int8_t statusPin)
{
    debugPrintLn("[init] started.");

    initBuffer(); // safe to call multiple times

    setModemStream(stream);

    sodaq_3gbee_onoff.init(vcc33Pin, onoffPin, statusPin);
    _onoff = &sodaq_3gbee_onoff;
}

// Initializes the modem instance. Sets the modem stream and the on-off power pins.
void Sodaq_3Gbee::init_wdt(Stream& stream, int8_t onoffPin)
{
    debugPrintLn("[init_wdt] started.");

    initBuffer(); // safe to call multiple times

    setModemStream(stream);

    sodaq_3gbee_onoff.init(-1, onoffPin);
    _onoff = &sodaq_3gbee_onoff;
}

void Sodaq_3Gbee::switchEchoOff()
{
    if (!_echoOff) {
        // Suppress echoing
        println("AT E0");
        readResponse();
        _echoOff = true;
    }
}

bool Sodaq_3Gbee::doInitialCommands()
{
    // verbose error messages
    println("AT+CMEE=2");
    if (readResponse() != ResponseOK) {
        return false;
    }

    // Switch to hex. Somehow this is also needed for AT+COPS=0
    if (!setHexMode()) {
        return false;
    }

    // enable network identification LED
    println("AT+UGPIOC=16,2");
    if (readResponse() != ResponseOK) {
        return false;
    }

    // SIM check
    if (!doSIMcheck()) {
        return false;
    }

    return true;
}

bool Sodaq_3Gbee::doSIMcheck()
{
    const uint8_t retry_count = 10;
    for (uint8_t i = 0; i < retry_count; i++) {
        if (i > 0) {
            sodaq_wdt_safe_delay(250);
        }

        SimStatuses simStatus = getSimStatus();
        if (simStatus == SimNeedsPin) {
            if (_pin == 0 || *_pin == '\0' || !setSimPin(_pin)) {
                debugPrintLn(DEBUG_STR_ERROR "SIM needs a PIN but none was provided, or setting it failed!");
                return false;
            }
        }
        else if (simStatus == SimReady) {
            return true;
        }
    }
    return false;
}

// Enable auto network registration
// Do AT+COPS=0 and wait for OK
bool Sodaq_3Gbee::enableAutoRegistration(uint32_t timeout)
{
    uint32_t start = millis();
    char operator_name[30];
    if (getOperatorName(operator_name, sizeof(operator_name))) {
        // We should have a name
        return true;
    }

    // The default timeout is 4 minutes. This may seem high, but ...
    // In some situations (new SIM card) it can take a long time
    // the get the registration. Subsequent registrations should
    // go much quicker.
    uint32_t delay_count = 500;
    while (!is_timedout(start, timeout)) {
        sodaq_wdt_safe_delay(delay_count);
        // Next time wait a little longer
        delay_count += 1000;

        println("AT+COPS=0");
        if (readResponse(NULL, 40000) == ResponseOK) {
            // TODO Fix this delay
            sodaq_wdt_safe_delay(1000);
            // We should have a name by now
            if (getOperatorName(operator_name, sizeof(operator_name))) {
                return true;
            }
        }
    }
    return false;
}

bool Sodaq_3Gbee::waitForSignalQuality(uint32_t timeout)
{
    uint32_t start = millis();
    const int8_t minRSSI = getMinRSSI();
    int8_t rssi;
    uint8_t ber;

    uint32_t delay_count = 500;
    while (!is_timedout(start, timeout)) {
        if (getRSSIAndBER(&rssi, &ber)) {
            if (rssi != 0 && rssi >= minRSSI) {
                _lastRSSI = rssi;
                _CSQtime = (int32_t)(millis() - start) / 1000;
                return true;
            }
        }
        sodaq_wdt_safe_delay(delay_count);
        // Next time wait a little longer, but not longer than 5 seconds
        if (delay_count < 5000) {
            delay_count += 1000;
        }
    }
    return false;
}

bool Sodaq_3Gbee::tryAuthAndActivate(PSDAuthType_e authType)
{
    // Set Authentication
    print("AT+UPSD=" DEFAULT_PROFILE ",6,");
    println(authType);
    if (readResponse() != ResponseOK) {
        return false;
    }

    // Activate using default profile
    println("AT+UPSDA=" DEFAULT_PROFILE ",3");
    if (readResponse(NULL, 200000) != ResponseOK) {
        return false;
    }

    return true;
}

PSDAuthType_e Sodaq_3Gbee::numToPSDAuthType(int8_t i)
{
    if (i >= PAT_TryAll && i <= PAT_AutoSelect) {
        return (PSDAuthType_e)i;
    }
    return PAT_TryAll;
}

// Turns on and initializes the modem, then connects to the network and activates the data connection.
bool Sodaq_3Gbee::connect()
{
    if (!connectSimple()) {
        return false;
    }

    // enable auto network registration
    if (!enableAutoRegistration()) {
        return false;
    }

    // NOTE: Default +URAT = 1,2 i.e. prefer UMTS, fallback to GPRS

    // cleanup tmp files
    //cleanupTempFiles();

    // set SMS to text mode
    println("AT+CMGF=1");
    if (readResponse() != ResponseOK) {
        return false;
    }

    // TODO check GPRS attach? (AT+CGATT=1 should be OK)

    // check if connected and disconnect
    if (isConnected()) {
        if (!disconnect()) {
            debugPrintLn(DEBUG_STR_ERROR "Modem seems to be already connected and failed to disconnect!");
            return false;
        }
    }

    if (!sendAPN(_apn, _apnUser, _apnPass)) {
        return false;
    }

    // DHCP
    println("AT+UPSD=" DEFAULT_PROFILE ",7,\"0.0.0.0\"");
    if (readResponse() != ResponseOK) {
        return false;
    }

    if (_psdAuthType != PAT_TryAll) {
        if (!tryAuthAndActivate(_psdAuthType)) {
            return false;
        }
    } else {
        // go through all authentication methods until one succeeds
        if (tryAuthAndActivate(PAT_None)) {
        } else if (tryAuthAndActivate(PAT_PAP)) {
        } else if (tryAuthAndActivate(PAT_CHAP)) {
        } else if (tryAuthAndActivate(PAT_AutoSelect)) {
        } else {
            return false;
        }
    }

    // If we got this far we succeeded
    return true;
}

bool Sodaq_3Gbee::connectSimple()
{
    if (!on()) {
        debugPrintLn("Modem failed to turn on");
        return false;
    }

    switchEchoOff();

    // switch off the +UMWI URCs
    // should we move this to switchEchoOff()
    // or some other location?
    println("AT+UMWI=0");
    readResponse();

    // if supported by target application, change the baudrate
    if (_baudRateChangeCallbackPtr) {
        println("AT+IPR=" STR(HIGH_BAUDRATE));
        if (readResponse() != ResponseOK) {
            return false;
        }

        _baudRateChangeCallbackPtr(HIGH_BAUDRATE);
        sodaq_wdt_safe_delay(1000); // wait for everything to be stable again
    }

    if (!doInitialCommands()) {
        return false;
    }

    if (!waitForSignalQuality()) {
        return false;
    }

    return true;
}

// Disconnects the modem from the network.
bool Sodaq_3Gbee::disconnect()
{
    // TODO also turn off the modem?
    println("AT+UPSDA=" DEFAULT_PROFILE ",4");

    return (readResponse(NULL, 40000) == ResponseOK);
}

ResponseTypes Sodaq_3Gbee::_cregParser(ResponseTypes& response, const char* buffer, size_t size,
        int* networkStatus, uint8_t* dummy)
{
    if (!networkStatus) {
        return ResponseError;
    }

    if (sscanf(buffer, "+CREG: %*d,%d", networkStatus) == 1) {
        return ResponseEmpty;
    }

    return ResponseError;
}

// Returns the current status of the network registration.
/*
 * Meaning of <status>
 *  0: not registered, the MT is not currently searching a new operator to register to
 *  1: registered, home network
 *  2: not registered, but the MT is currently searching a new operator to register to
 *  3: registration denied
 *  4: unknown (e.g. out of GERAN/UTRAN/E-UTRAN coverage)
 *  5: registered, roaming
 *  6: registered for "SMS only", home network (applicable only when <AcTStatus> indicates E-UTRAN)
 *  7: registered for "SMS only", roaming (applicable only when <AcTStatus> indicates E-UTRAN)
 *  9: registered for "CSFB not preferred", home network (applicable only when <AcTStatus> indicates E-UTRAN)
 * 10: registered for "CSFB not preferred", roaming (applicable only when <AcTStatus> indicates E-UTRAN)
 */
NetworkRegistrationStatuses Sodaq_3Gbee::getNetworkStatus()
{
    println("AT+CREG?"); // TODO ? +CGREG

    int networkStatus;
    if (readResponse<int, uint8_t>(_cregParser, &networkStatus, NULL) == ResponseOK) {
        switch (networkStatus) {
            case 0: return NoNetworkRegistrationStatus;
            case 1: return Home;
            case 2: return NoNetworkRegistrationStatus;
            case 3: return Denied;
            case 4: return UnknownNetworkRegistrationStatus;
            case 5: return Roaming;
            default: return Denied; // rest of statuses are actually registered, but they do not support data
        }
    }

    return UnknownNetworkRegistrationStatus;
}

// Returns the network technology the modem is currently registered to.
NetworkTechnologies Sodaq_3Gbee::getNetworkTechnology()
{
    println("AT+COPS?");

    int networkTechnology;
    if (readResponse<int, uint8_t>(_copsParser, &networkTechnology, NULL) == ResponseOK) {
        switch (networkTechnology) {
            case 0: return GSM;
            case 1: return GSM;
            case 2: return UTRAN;
            case 3: return EDGE;
            case 4: return HSDPA;
            case 5: return HSUPA;
            case 6: return HSDPAHSUPA;
            case 7: return LTE;
        }
    }

    return UnknownNetworkTechnology;
}

ResponseTypes Sodaq_3Gbee::_csqParser(ResponseTypes& response, const char* buffer, size_t size,
        int* rssi, int* ber)
{
    if (!rssi || !ber) {
        return ResponseError;
    }

    if (sscanf(buffer, "+CSQ: %d,%d", rssi, ber) == 2) {
        return ResponseEmpty;
    }

    return ResponseError;
}

// Gets the Received Signal Strength Indication in dBm and Bit Error Rate.
// Returns true if successful.
bool Sodaq_3Gbee::getRSSIAndBER(int8_t* rssi, uint8_t* ber)
{
    static char berValues[] = { 49, 43, 37, 25, 19, 13, 7, 0 }; // 3GPP TS 45.008 [20] subclause 8.2.4

    println("AT+CSQ");

    int csqRaw = 0;
    int berRaw = 0;

    if (readResponse<int, int>(_csqParser, &csqRaw, &berRaw) == ResponseOK) {
        *rssi = ((csqRaw == 99) ? 0 : convertCSQ2RSSI(csqRaw));
        *ber = ((berRaw == 99 || static_cast<size_t>(berRaw) >= sizeof(berValues)) ? 0 : berValues[berRaw]);

        return true;
    }

    return false;
}

/*
 * The range is the following:
 *   0: -113 dBm or less
 *   1: -111 dBm
 *   2..30: from -109 to -53 dBm with 2 dBm steps
 *   31: -51 dBm or greater
 *   99: not known or not detectable or currently not available
 */
int8_t Sodaq_3Gbee::convertCSQ2RSSI(uint8_t csq) const
{
    return -113 + 2 * csq;
}

uint8_t Sodaq_3Gbee::convertRSSI2CSQ(int8_t rssi) const
{
    return (rssi + 113) / 2;
}

// Parse the result from AT+COPS? and when we want to see the operator name.
// The manual says to expect:
//   +COPS: <mode>[,<format>,<oper>[,<AcT>]]
//   OK
ResponseTypes Sodaq_3Gbee::_copsParser(ResponseTypes& response, const char* buffer, size_t size,
        char* operatorNameBuffer, size_t* operatorNameBufferSize)
{
    if (!operatorNameBuffer || !operatorNameBufferSize) {
        return ResponseError;
    }

    // TODO maybe limit length of string in format? needs converting int to string though
    if (sscanf(buffer, "+COPS: %*d,%*d,\"%[^\"]\",%*d", operatorNameBuffer) == 1) {
        return ResponseEmpty;
    }
    if (sscanf(buffer, "+COPS: %*d,%*d,\"%[^\"]\"", operatorNameBuffer) == 1) {
        return ResponseEmpty;
    }
    int dummy;
    if (sscanf(buffer, "+COPS: %d", &dummy) == 1) {
        return ResponseEmpty;
    }

    return ResponseError;
}

ResponseTypes Sodaq_3Gbee::_copsParser(ResponseTypes& response, const char* buffer, size_t size,
        int* networkTechnology, uint8_t* dummy)
{
    if (!networkTechnology) {
        return ResponseError;
    }

    if (sscanf(buffer, "+COPS: %*d,%*d,\"%*[^\"]\",%d", networkTechnology) == 1) {
        return ResponseEmpty;
    }

    return ResponseError;
}

// Gets the Operator Name.
// Returns true if successful.
bool Sodaq_3Gbee::getOperatorName(char* buffer, size_t size)
{
    if (size > 0) {
        buffer[0] = 0;
    }

    println("AT+COPS?");

    if (readResponse<char, size_t>(_copsParser, buffer, &size) != ResponseOK) {
        return false;
    }
    // Make sure we have at least something
    return (buffer[0] != '\0');
}

bool Sodaq_3Gbee::selectBestOperator(Stream & verbose_stream)
{
    String listOfOperators;
    String oper_long;
    String oper_num;
    size_t oper_status;
    uint8_t highest_csq = 0;
    size_t highest_csq_ix = 0;
    int8_t lastRSSI;
    uint8_t lastCSQ;

    // Get list of operators
    uint32_t delay_count = 500;
    bool found_list = false;
    for (size_t ix = 0; ix < 5; ++ix) {
        if (getOperators(listOfOperators)) {
            found_list = true;
            break;
        }
        sodaq_wdt_safe_delay(delay_count);
        delay_count += 500;
    }
    if (!found_list) {
        verbose_stream.println("ERROR: Unable to get a list of operators");
        return false;
    }

    // Select each and keep track of highest CSQ.

    verbose_stream.println(String("List of available operators: ") + listOfOperators);
    size_t nr_valid_opers = 0;
    for (size_t ix = 0; ; ++ix) {
        if (getNthOperator(listOfOperators, ix, oper_long, oper_num, oper_status)) {
            verbose_stream.println("=======================");
            verbose_stream.println(String("  operator: ") + oper_long);
            verbose_stream.println(String("    number: ") + oper_num);
            //debugPrintLn(String("  status: ") + oper_status);
            if (oper_status == 1 || oper_status == 2) {      // 1: available, 2: current, 3: forbidden
                nr_valid_opers++;
                highest_csq_ix = ix;
            }
        } else {
            // No more operators in the list
            break;
        }
    }
    verbose_stream.println("=======================");
    verbose_stream.println(String("Number of available operators: ") + nr_valid_opers);
    if (nr_valid_opers == 0) {
        return false;
    }

    // Only do a check of all available if we have at least 2 or more
    if (nr_valid_opers > 1) {
        for (size_t ix = 0; ; ++ix) {
            if (getNthOperator(listOfOperators, ix, oper_long, oper_num, oper_status)) {
                verbose_stream.println();
                verbose_stream.println("=======================");
                verbose_stream.println(String("  operator: ") + oper_long);
                verbose_stream.println(String("    number: ") + oper_num);
                //debugPrintLn(String("  status: ") + oper_status);
                if (oper_status == 1 || oper_status == 2) {      // 1: available, 2: current, 3: forbidden
                    if (selectOperatorWithRSSI(oper_long, oper_num, lastRSSI, verbose_stream)) {
                        lastCSQ = convertRSSI2CSQ(lastRSSI);
                        verbose_stream.println(String("  RSSI: ") + lastRSSI + "dBm (CSQ: " + lastCSQ + ")");
                        if (lastCSQ > highest_csq) {
                            highest_csq = lastCSQ;
                            highest_csq_ix = ix;
                        }
                    }
                }
            } else {
                // No more operators in the list
                break;
            }
        }
    }

    // Select the operator with highest CSQ, or the only available operator
    verbose_stream.println();
    if (!getNthOperator(listOfOperators, highest_csq_ix, oper_long, oper_num, oper_status)) {
        // Shouldn't happen
        return false;
    }
    verbose_stream.println("=======================");
    verbose_stream.println(String("Selecting best operator: \"") + oper_long + '"');
    verbose_stream.println(String("                 number: ") + oper_num);

    if (!(oper_status == 1 || oper_status == 2)) {      // 1: available, 2: current, 3: forbidden
        // Shouldn't happen
        return false;
    }

    if (!selectOperatorWithRSSI(oper_long, oper_num, lastRSSI, verbose_stream)) {
        // Shouldn't happen, error message already given
        return false;
    }

    // No need to disconnect. We're switching off in a moment.

    return true;
}

bool Sodaq_3Gbee::selectOperatorWithRSSI(const String & oper_long, const String & oper_num,
        int8_t & lastRSSI, Stream & verbose_stream)
{
    String use_name;
    if (oper_num != "" && selectOperatorNum(oper_num, 60000)) {
        // OK
        use_name = oper_num;
    } else if (oper_long != "" && selectOperator(oper_long, 60000)) {
        // OK
        use_name = oper_long;
    } else {
        verbose_stream.println("ERROR: Failed to select operator");
        return false;
    }

    // This could return operator name long, short or number
    char op_name[30];
    if (!getOperatorName(op_name, sizeof(op_name))) {
        verbose_stream.println("ERROR: Failed to get selected operator");
        return false;
    }

    // Is it what we expected?
    if (use_name != String(op_name)) {
        verbose_stream.println(String("WARNING: Selected ") + use_name + ", but got " + op_name);
        return false;
    }

    // Disconnect, just in case it is connected
    disconnect();

    // Connect and measure CSQ/RSSI
    if (!connect()) {
        verbose_stream.println("ERROR: Failed to connect to operator");
        return false;
    }

    IP_t local_ip = getLocalIP();
    if (local_ip == NO_IP_ADDRESS) {
        verbose_stream.println("ERROR: Failed to get IP address");
        return false;
    }

    char ip_txt[20];
    snprintf(ip_txt, sizeof(ip_txt), "%d.%d.%d.%d", IP_TO_TUPLE(local_ip));
    verbose_stream.println(String("  IP addr: ") + ip_txt);
#if 0
    // Ping some server
#endif
    lastRSSI = getLastRSSI();

    return true;
}

bool Sodaq_3Gbee::getOperators(String & listOfOperators)
{
    //
    //size_t Sodaq_3Gbee::httpRequest(const char* url, uint16_t port,
    //    const char* endpoint, HttpRequestTypes requestType,
    //          char* responseBuffer, size_t responseSize,
    //          const char* sendBuffer, size_t sendSize)
    //
    //
    debugPrintLn("checking networks - begin");

    // ???? Maybe not needed to de-register to get the operators list
    if (false) {
        deregisterNetwork(10000);
    }

    // Get list of operators
    // +COPS: [(<stat>, long <oper>, short <oper>, numeric <oper>[,<AcT>])
    //         [,(<stat>, long <oper>, short <oper>, numeric <oper>[,<AcT>]),[...]]],
    //         (list of supported <mode>s),
    //         (list of supported<format>s)
    // On other words, a list of operators, plus list of modes, plus list of formats.
    // For example: +COPS: (1,"vodafone NL","voda NL","20404"),
    //                     (3,"NL KPN","NL KPN","20408"),
    //                     (3,"T-Mobile NL","TMO NL","20416"),,
    //                     (0-6),
    //                     (0-2)
    println("AT+COPS=?");

    char buffer[250];
    size_t buf_size = sizeof(buffer);
    buffer[0] = '\0';
    ResponseTypes retval;
    retval = readResponse<char, size_t>(_nakedStringParser, buffer, &buf_size, NULL, 120000);
    if (retval == ResponseOK) {
        listOfOperators = buffer;
        if (listOfOperators.startsWith("+COPS: ")) {
            listOfOperators = listOfOperators.substring(7);
        }
    }
    return retval == ResponseOK;
}

bool Sodaq_3Gbee::getNthOperator(const String & listOfOperators, size_t nth, String & oper_long, String & oper_num, size_t & status)
{
    bool retval = false;
    size_t cur_ix = 0;
    int lpar_ix = -1;
    int rpar_ix = -1;
    for (size_t ix = 0; ix <= nth; ++ix) {
        rpar_ix = -1;
        lpar_ix = listOfOperators.indexOf('(', cur_ix);
        //debugPrintLn(String(" lpar_ix ") + lpar_ix);
        if (lpar_ix < 0) {
            // No more "(" found
            break;
        }
        if ((size_t)lpar_ix != cur_ix) {
            // This is probably the empty field in the network list.
            break;
        }
        rpar_ix = listOfOperators.indexOf(')', lpar_ix);
        //debugPrintLn(String(" rpar_ix ") + rpar_ix);
        if (rpar_ix < 0) {
            // No more ")" found
            break;
        }
        cur_ix = rpar_ix + 1;
        if (cur_ix < listOfOperators.length() && listOfOperators.charAt(cur_ix) == ',') {
            // Skip comma separator
            ++cur_ix;
        }
    }
    if (lpar_ix >= 0 && rpar_ix >= 0) {
        // Found a match
        String tmp = listOfOperators.substring(lpar_ix + 1, rpar_ix);
        status = getValueAt(tmp, ',', 0).toInt();
        // Long format
        String tmp2 = getValueAt(tmp, ',', 1);
        // Strip quotes
        if (tmp2.length() >= 2 && tmp2.charAt(0) == '"' && tmp2.charAt(tmp2.length() - 1) == '"') {
            oper_long = tmp2.substring(1, tmp2.length() - 1);
        } else {
            oper_long = tmp2;
        }
        // Numeric format
        tmp2 = getValueAt(tmp, ',', 3);
        // Strip quotes
        if (tmp2.length() >= 2 && tmp2.charAt(0) == '"' && tmp2.charAt(tmp2.length() - 1) == '"') {
            oper_num = tmp2.substring(1, tmp2.length() - 1);
        } else {
            oper_num = tmp2;
        }
        retval = true;
    }

    return retval;
}

bool Sodaq_3Gbee::selectOperator(const String & oper_long, uint32_t timeout)
{
    bool retval = false;

    deregisterNetwork(10000);

    // Manual select operator, using long format
    println(String("AT+COPS=1,0,\"") + oper_long + '"');
    if (readResponse(NULL, 60000) == ResponseOK) {
        // Now wait for URC +CREG
        // ???? How do we do that?
        uint32_t start = millis();
        while (!is_timedout(start, timeout)) {
            NetworkRegistrationStatuses status;
            status = getNetworkStatus();
            if (status == Home || status == Roaming) {
                retval = true;
                break;
            }
        }
    }

    return retval;
}

bool Sodaq_3Gbee::selectOperatorNum(const String & oper_num, uint32_t timeout)
{
    bool retval = false;

    deregisterNetwork(10000);

    // Manual select operator, using numeric format
    println(String("AT+COPS=1,2,\"") + oper_num + '"');
    if (readResponse(NULL, 60000) == ResponseOK) {
        // Now wait for URC +CREG
        // ???? How do we do that?
        uint32_t start = millis();
        while (!is_timedout(start, timeout)) {
            NetworkRegistrationStatuses status;
            status = getNetworkStatus();
            if (status == Home || status == Roaming) {
                retval = true;
                break;
            }
        }
    }

    return retval;
}

bool Sodaq_3Gbee::waitForDeactivatedNetwork(uint32_t timeout)
{
    // This loop relies on readResponse being called via isAlive()
    uint32_t start = millis();
    uint32_t delay_count = 50;
    _foundUUPSDD = false;
    while (!_foundUUPSDD && !is_timedout(start, timeout)) {
        isAlive();
        sodaq_wdt_safe_delay(delay_count);
        delay_count += 250;
    }

    return _foundUUPSDD;
}

bool Sodaq_3Gbee::deregisterNetwork(uint32_t timeout)
{
    bool retval = false;
    println("AT+COPS=2");
    if (readResponse() == ResponseOK) {
        debugPrintLn("OK, deregister from network");
        // ?? Expect +UUPSDD: 0
        retval = waitForDeactivatedNetwork(timeout);
    }
    return retval;
}

String Sodaq_3Gbee::getValueAt(String data, char separator, int index)
{
    int found = 0;
    int strIndex[] = { 0, -1 };
    int maxIndex = data.length() - 1;
    for (int i = 0; i <= maxIndex && found <= index; i++) {
        if (data.charAt(i) == separator || i == maxIndex) {
            found++;
            strIndex[0] = strIndex[1] + 1;
            strIndex[1] = (i == maxIndex) ? i + 1 : i;
        }
    }
    return found > index ? data.substring(strIndex[0], strIndex[1]) : "";
}

ResponseTypes Sodaq_3Gbee::_cnumParser(ResponseTypes& response, const char* buffer, size_t size,
        char* numberBuffer, size_t* numberBufferSize)
{
    if (!numberBuffer || !numberBufferSize) {
        return ResponseError;
    }

    // TODO test with a sim that has a number
    // TODO limit?
    if (sscanf(buffer, "+CNUM: \"%*[^\"]\",\"%[^\"]\",%*d", numberBuffer) == 1) { // TODO is it "My Number"?
        return ResponseEmpty;
    }

    return ResponseError;

}

// Gets Mobile Directory Number.
// Returns true if successful.
bool Sodaq_3Gbee::getMobileDirectoryNumber(char* buffer, size_t size)
{
    if (size < 7 + 1) {
        return false;
    }

    if (size > 0) {
        buffer[0] = 0;
    }

    println("AT+CNUM");

    return (readResponse<char, size_t>(_cnumParser, buffer, &size) == ResponseOK);
}

ResponseTypes Sodaq_3Gbee::_nakedStringParser(ResponseTypes& response, const char* buffer,
        size_t size, char* stringBuffer, size_t* stringBufferSize)
{
    if (!stringBuffer || !stringBufferSize) {
        return ResponseError;
    }

    stringBuffer[0] = 0;
    if (*stringBufferSize > 0)
    {
        strncat(stringBuffer, buffer, *stringBufferSize - 1);

        return ResponseEmpty;
    }

    return ResponseError;
}

// Gets International Mobile Equipment Identity.
// Should be provided with a buffer of at least 16 bytes.
// Returns true if successful.
bool Sodaq_3Gbee::getIMEI(char* buffer, size_t size)
{
    if (size < 15 + 1) {
        return false;
    }

    switchEchoOff();
    doInitialCommands();

    if (size > 0) {
        buffer[0] = 0;
    }

    println("AT+CGSN");

    return (readResponse<char, size_t>(_nakedStringParser, buffer, &size) == ResponseOK);
}

ResponseTypes Sodaq_3Gbee::_ccidParser(ResponseTypes& response, const char* buffer, size_t size,
        char* ccidBuffer, size_t* ccidBufferSize)
{
    if (!ccidBuffer || !ccidBufferSize) {
        return ResponseError;
    }

    // TODO limit?
    if (sscanf(buffer, "+CCID: %s", ccidBuffer) == 1) {
        return ResponseEmpty;
    }

    return ResponseError;
}

// Gets Integrated Circuit Card ID.
// Should be provided with a buffer of at least 21 bytes.
// Returns true if successful.
bool Sodaq_3Gbee::getCCID(char* buffer, size_t size)
{
    if (size < 20 + 1) {
        return false;
    }

    switchEchoOff();
    doInitialCommands();

    if (size > 0) {
        buffer[0] = 0;
    }

    println("AT+CCID");

    return (readResponse<char, size_t>(_ccidParser, buffer, &size) == ResponseOK);
}

// Gets the International Mobile Station Identity.
// Should be provided with a buffer of at least 16 bytes.
// Returns true if successful.
bool Sodaq_3Gbee::getIMSI(char* buffer, size_t size)
{
    if (size < 15 + 1) {
        return false;
    }

    switchEchoOff();
    doInitialCommands();

    if (size > 0) {
        buffer[0] = 0;
    }

    println("AT+CIMI");

    return (readResponse<char, size_t>(_nakedStringParser, buffer, &size) == ResponseOK);
}

ResponseTypes Sodaq_3Gbee::_cpinParser(ResponseTypes& response, const char* buffer, size_t size,
        SimStatuses* parameter, uint8_t* dummy)
{
    if (!parameter) {
        return ResponseError;
    }

    char status[16];
    if (sscanf(buffer, "+CPIN: %" STR(sizeof(status)-1) "s", status) == 1) {
        if (startsWith("READY", status)) {
            *parameter = SimReady;
        }
        else {
            *parameter = SimNeedsPin;
        }

        return ResponseEmpty;
    }

    return ResponseError;
}

// Returns the current SIM status.
SimStatuses Sodaq_3Gbee::getSimStatus()
{
    SimStatuses simStatus;

    println("AT+CPIN?");
    if (readResponse<SimStatuses, uint8_t>(_cpinParser, &simStatus, NULL) == ResponseOK) {
        return simStatus;
    }

    return SimMissing;
}

// Returns the local IP Address.
IP_t Sodaq_3Gbee::getLocalIP()
{
    IP_t ip = NO_IP_ADDRESS;

    println("AT+UPSND=" DEFAULT_PROFILE ",0");
    if (readResponse<IP_t, uint8_t>(_upsndParser, &ip, NULL) == ResponseOK) {
        return ip;
    }

    return NO_IP_ADDRESS;
}

ResponseTypes Sodaq_3Gbee::_udnsrnParser(ResponseTypes& response, const char* buffer, size_t size,
        IP_t* ipResult, uint8_t* dummy)
{
    if (!ipResult) {
        return ResponseError;
    }

    int o1, o2, o3, o4;

    if (sscanf(buffer, "+UDNSRN: \"" IP_FORMAT "\"", &o1, &o2, &o3, &o4) == 4) {
        *ipResult = TUPLE_TO_IP(o1, o2, o3, o4);

        return ResponseEmpty;
    }

    return ResponseError;
}

ResponseTypes Sodaq_3Gbee::_upsndParser(ResponseTypes& response, const char* buffer, size_t size,
        IP_t* ipResult, uint8_t* dummy)
{
    if (!ipResult) {
        return ResponseError;
    }

    int o1, o2, o3, o4;

    if (sscanf(buffer, "+UPSND: " DEFAULT_PROFILE ", 0, \"" IP_FORMAT "\"", &o1, &o2, &o3, &o4) == 4) {
        *ipResult = TUPLE_TO_IP(o1, o2, o3, o4);

        return ResponseEmpty;
    }

    return ResponseError;
}

ResponseTypes Sodaq_3Gbee::_upsndParser(ResponseTypes& response, const char* buffer, size_t size,
        uint8_t* thirdParam, uint8_t* dummy)
{
    if (!thirdParam) {
        return ResponseError;
    }

    int value;

    if (sscanf(buffer, "+UPSND: %*d,%*d,%d", &value) == 1) {
        *thirdParam = value;

        return ResponseEmpty;
    }

    return ResponseError;
}

ResponseTypes Sodaq_3Gbee::_usocrParser(ResponseTypes& response, const char* buffer, size_t size,
        uint8_t* socket, uint8_t* dummy)
{
    if (!socket) {
        return ResponseError;
    }

    int value;

    if (sscanf(buffer, "+USOCR: %d", &value) == 1) {
        *socket = value;

        return ResponseEmpty;
    }

    return ResponseError;
}

// Returns the IP of the given host (nslookup).
IP_t Sodaq_3Gbee::getHostIP(const char* host)
{
    if (_host_ip != NO_IP_ADDRESS && strcmp(host, _host_name) == 0) {
        return _host_ip;
    }

    IP_t ip = NO_IP_ADDRESS;

    print("AT+UDNSRN=0,\"");
    print(host);
    println("\"");
    if (readResponse<IP_t, uint8_t>(_udnsrnParser, &ip, NULL, NULL, 70000) == ResponseOK) {
        if (ip != NO_IP_ADDRESS) {
            // Try to cache it
            // Only if it fits
            if (strlen(host) < sizeof(_host_name) - 1) {
                memset(_host_name, 0, sizeof(_host_name));
                strncpy(_host_name, host, sizeof(_host_name) - 1);
                _host_ip = ip;
            }
        }
        return ip;
    }

    return NO_IP_ADDRESS;
}

bool Sodaq_3Gbee::getSessionCounters(uint32_t* sentCnt, uint32_t* recvCnt)
{
    println("AT+UGCNTRD");

    if (readResponse<uint32_t, uint32_t>(_ugcntrdParser, sentCnt, recvCnt) == ResponseOK) {
        return true;
    }

    return false;
}

/*
 * +UGCNTRD: <cid>,<sent_sess_bytes>,<received_sess_bytes>,<sent_total_bytes>,<received_total_bytes>
 */
ResponseTypes Sodaq_3Gbee::_ugcntrdParser(ResponseTypes& response, const char* buffer, size_t size,
        uint32_t* sentCnt, uint32_t* recvCnt)
{
    if (!sentCnt || !recvCnt) {
        return ResponseError;
    }

    long int value1;
    long int value2;

    if (sscanf(buffer, "+UGCNTRD: %*d,%*d,%*d,%ld,%ld", &value1, &value2) == 2) {
        *sentCnt = value1;
        *recvCnt = value2;

        return ResponseEmpty;
    }

    return ResponseError;
}

// Creates a new socket for the given protocol, optionally bound to the given localPort.
// Returns the index of the socket created or -1 in case of error.
int Sodaq_3Gbee::createSocket(Protocols protocol, uint16_t localPort)
{
    uint8_t protocolIndex;
    if (protocol == TCP) {
        protocolIndex = 6;
    }
    else if (protocol == UDP) {
        protocolIndex = 17;
    }
    else {
        return SOCKET_FAIL;
    }

    print("AT+USOCR=");
    if (localPort > 0) {
        print(protocolIndex);
        print(",");
        println(localPort);
    }
    else {
        println(protocolIndex);
    }

    uint8_t socket;
    if (readResponse<uint8_t, uint8_t>(_usocrParser, &socket, NULL) == ResponseOK) {
        return socket;
    }

    return SOCKET_FAIL;
}

size_t Sodaq_3Gbee::ipToString(IP_t ip, char* buffer, size_t size)
{
    return snprintf(buffer, size, IP_FORMAT, IP_TO_TUPLE(ip));
}

bool Sodaq_3Gbee::isValidIPv4(const char* str)
{
    uint8_t segs = 0; // Segment count
    uint8_t chcnt = 0; // Character count within segment
    uint8_t accum = 0; // Accumulator for segment

    if (!str) {
        return false;
    }

    // Process every character in string
    while (*str != '\0') {
        // Segment changeover
        if (*str == '.') {
            // Must have some digits in segment
            if (chcnt == 0) {
                return false;
            }

            // Limit number of segments
            if (++segs == 4) {
                return false;
            }

            // Reset segment values and restart loop
            chcnt = accum = 0;
            str++;
            continue;
        }

        // Check numeric
        if ((*str < '0') || (*str > '9')) {
            return false;
        }

        // Accumulate and check segment
        if ((accum = accum * 10 + *str - '0') > 255) {
            return false;
        }

        // Advance other segment specific stuff and continue loop
        chcnt++;
        str++;
    }

    // Check enough segments and enough characters in last segment
    if (segs != 3) {
        return false;
    }

    if (chcnt == 0) {
        return false;
    }

    // Address OK

    return true;
}

// Requests a connection to the given host and port, on the given socket.
// Returns true if successful.
bool Sodaq_3Gbee::connectSocket(uint8_t socket, const char* host, uint16_t port)
{
    bool usePassedHost;
    char ipBuffer[16];

    if (isValidIPv4(host)) {
        usePassedHost = true;
    }
    else {
        usePassedHost = false;
        if (ipToString(getHostIP(host), ipBuffer, sizeof(ipBuffer)) <= 0) {
            return false;
        }
    }

    _socketClosedBit[socket] = false;
    print("AT+USOCO=");
    print(socket);
    print(",\"");
    print(usePassedHost ? host : ipBuffer);
    print("\",");
    println(port);

    bool retval = (readResponse(NULL, 30000) == ResponseOK);
    _timeToSocketConnect = millis() - _startOn;
    return retval;
}

// Sends the given buffer through the given socket.
// Returns true if successful.
bool Sodaq_3Gbee::socketSend(uint8_t socket, const uint8_t* buffer, size_t size)
{
    // TODO see if we should keep an array of sockets so that the UDP-specific
    // commands can be used instead, without first initializing the UDP socket
    // OR maybe query the socket type? AT+USOCTL=0,0  => +USOCTL:0,0,6 (socket #0 is TCP)

    // TODO +USOCTL=1 check last error, (11: queue full)

    print("AT+USOWR=");
    print(socket);
    print(",");
    println(size);

    // Wait for prompt. See writeFile
    if (readResponse() == ResponsePrompt) {
        // After the @ prompt reception, wait for a minimum of 50 ms before sending data.
        delay(51);

        for (size_t i = 0; i < size; ++i) {
            writeByte(buffer[i]);
        }
    }

    bool status = (readResponse(NULL, 10000) == ResponseOK);
    if (_flushEverySend && status) {
        // We want to be sure it is sent
        status = waitForSocketOutput(socket);
    }
    return status;
}

// Sends the given buffer through the given socket.
// Returns true if successful.
bool Sodaq_3Gbee::socketSend(uint8_t socket, const char* str)
{
    return socketSend(socket, (uint8_t *)str, strlen(str));
}

ResponseTypes Sodaq_3Gbee::_usordParser(ResponseTypes& response, const char* buffer, size_t size,
        char* resultBuffer, uint8_t* dummy)
{
    if (!resultBuffer) {
        return ResponseError;
    }

    int socket, count;
    if ((sscanf(buffer, "+USORD: %d,%d,", &socket, &count) == 2)
        && (buffer[size - count*2 - 2] == '\"')
        && (buffer[size - 1] == '\"')
        && (count < MAX_SOCKET_BUFFER)) {
        memcpy(resultBuffer, &buffer[size - 1 - count*2], count*2);
        resultBuffer[count*2] = 0;

        return ResponseEmpty;
    }

    return ResponseError;
}

// Reads data from the given socket into the given buffer.
// Returns the number of bytes written to the buffer.
// NOTE: if the modem hasn't reported available data, it blocks for up to 10 seconds waiting.
size_t Sodaq_3Gbee::socketReceive(uint8_t socket, uint8_t* buffer, size_t size)
{
    if (socket >= ARRAY_SIZE(_socketPendingBytes)) {
        return 0;
    }

    // if there are no data available yet, block for some seconds while checking
    uint32_t start = millis();
    uint32_t delay_count = 50;
    while (_socketPendingBytes[socket] == 0 && !is_timedout(start, 10000)) {
        isAlive();
        sodaq_wdt_safe_delay(delay_count);
        if (delay_count < 2000) {
            delay_count += 250;
        }
    }

    size_t pending = _socketPendingBytes[socket];
    size_t count = (pending > size) ? size : pending;

    if (pending == 0) {
        return 0;
    }

    // bound the count, as the socket bytes are in hex string (so 2 * bytes)
    if (count > MAX_SOCKET_BUFFER/2) {
        count = MAX_SOCKET_BUFFER/2;
    }

    print("AT+USORD=");
    print(socket);
    print(",");
    println(count);

    char resultBuffer[MAX_SOCKET_BUFFER];
    if (readResponse<char, uint8_t>(_usordParser, resultBuffer, NULL) == ResponseOK) {
        // stop at the first string termination char, or if output buffer is over, or if payload buffer is over
        size_t outputIndex = 0;
        size_t inputIndex = 0;

        while (outputIndex < count
            && resultBuffer[inputIndex] != 0
            && resultBuffer[inputIndex + 1] != 0) {
            buffer[outputIndex] = HEX_PAIR_TO_BYTE(resultBuffer[inputIndex], resultBuffer[inputIndex + 1]);

            inputIndex += 2;
            outputIndex++;
        }

        _socketPendingBytes[socket] -= count;
        return count;
    }

    return 0;
}

size_t Sodaq_3Gbee::socketBytesPending(uint8_t socket)
{
    return _socketPendingBytes[socket];
}

// Closes the given socket.
// Returns true if successful.
bool Sodaq_3Gbee::closeSocket(uint8_t socket)
{
    // Wait until there are no more unacknowledged output data
    waitForSocketOutput(socket);

    print("AT+USOCL=");
    println(socket);

    bool retval = (readResponse(NULL, 20000) == ResponseOK);
    _timeToSocketClose = millis() - _startOn;
    return retval;
}

// Blocks waiting for the given socket to be reported closed.
// This method should be called only after closeSocket() or when the remote is expected to close the socket.
// Times out after 60 seconds.
void Sodaq_3Gbee::waitForSocketClose(uint8_t socket, uint32_t timeout)
{
    debugPrint("[waitForSocketClose]: ");
    debugPrintLn(socket);

    uint32_t start = millis();
    while (isAlive() && (!_socketClosedBit[socket]) && (!is_timedout(start, timeout))) {
        delay(5);
    }
}

// Read result from AT+USOCTL=<socket>,11
// +USOCTL: 0,11,2
ResponseTypes Sodaq_3Gbee::_usoctlParser(ResponseTypes& response, const char* buffer, size_t size,
        uint16_t* result, uint8_t* dummy)
{
    if (!result) {
        return ResponseError;
    }

    int value;
    // +USOCTL: 0,11,2
    if (sscanf(buffer, "+USOCTL: %*d,%*d,%d", &value) == 1) {
        *result = value;

        return ResponseEmpty;
    }

    return ResponseError;
}

// Wait until no more unacknowledged socket output
bool Sodaq_3Gbee::waitForSocketOutput(uint8_t socket, uint32_t timeout)
{
    //debugPrint("[waitForSocketOutput]: ");
    //debugPrintLn(socket);

    bool retval = false;        // Assume the worst, sorry
    uint32_t start = millis();

    while (isAlive() && (!is_timedout(start, timeout))) {
        print("AT+USOCTL=");
        print(socket);
        println(",11");
        // parse +USOCTL: 0,11,0
        uint16_t value;
        if (readResponse<uint16_t, uint8_t>(_usoctlParser, &value, NULL) == ResponseOK) {
            if (value == 0) {
                retval = true;
                break;
            }
        } else {
            // Every other response is considered an error, so quit
            break;
        }
        sodaq_wdt_safe_delay(300);
    }
    //debugPrintLn("[waitForSocketOutput]: end");
    return retval;
}

// ==== TCP

bool Sodaq_3Gbee::openTCP(const char *apn, const char *apnuser, const char *apnpwd,
            const char *server, int port, bool transMode)
{
    // TODO Verify this
    bool retval = false;
    if (on()) {
        setApn(apn, apnuser, apnpwd);
        if (connect()) {
            // IP_t ip = getHostIP(server);
            _openTCPsocket = createSocket(TCP);
            // TODO Use ip instead of hostname
            if (_openTCPsocket >= 0 && connectSocket(_openTCPsocket, server, port)) {
                retval = true;
            }
        } else {
            // The connect failed
            off();
        }
    }
    return retval;
}

bool Sodaq_3Gbee::openTCP(const char *server, int port, bool transMode)
{
    // TODO Verify this
    bool retval = false;
    if (on()) {
        if (connect()) {
            // IP_t ip = getHostIP(server);
            _openTCPsocket = createSocket(TCP);
            // TODO Use ip instead of hostname
            if (_openTCPsocket >= 0 && connectSocket(_openTCPsocket, server, port)) {
                retval = true;
            }
        } else {
            // The connect failed
            off();
        }
    }
    return retval;
}

void Sodaq_3Gbee::closeTCP(bool switchOff)
{
    // TODO Verify this
    if (_openTCPsocket >= 0) {
        closeSocket(_openTCPsocket);
        //waitForSocketClose(_openTCPsocket);
        _openTCPsocket = -1;
    }
    if (switchOff) {
        off();
    }
}

bool Sodaq_3Gbee::sendDataTCP(const uint8_t *data, size_t data_len)
{
    // TODO Verify this
    bool retval = false;
    if (_openTCPsocket >= 0) {
        if (socketSend(_openTCPsocket, data, data_len)) {
            retval = true;
        }
    }
    return retval;
}

bool Sodaq_3Gbee::receiveDataTCP(uint8_t *data, size_t data_len, uint16_t timeout)
{
    // TODO Verify this
    bool retval = false;
    if (_openTCPsocket >= 0) {
        size_t nrbytes = socketReceive(_openTCPsocket, data, data_len);
        if (nrbytes == data_len) {
            retval = true;
        }
    }
    return retval;
}

// ==== HTTP

// Creates an HTTP request using the (optional) given buffer and
// (optionally) returns the received data.
// endpoint should include the initial "/".
size_t Sodaq_3Gbee::httpRequest(const char* server, uint16_t port,
        const char* endpoint, HttpRequestTypes requestType,
        char* responseBuffer, size_t responseSize,
        const char* sendBuffer, size_t sendSize)
{
    // TODO maybe return error <0 ?

    // reset http profile 0
    println("AT+UHTTP=0");
    if (readResponse() != ResponseOK) {
        return 0;
    }

    deleteFile(DEFAULT_HTTP_RECEIVE_TMP_FILENAME); // cleanup the file first (if exists)

    if (requestType > HttpRequestTypesMAX) {
        debugPrintLn(DEBUG_STR_ERROR "Unknown request type!");
        return 0;
    }

    // set server host name
    print("AT+UHTTP=0,");
    print(isValidIPv4(server) ? "0,\"" : "1,\"");
    print(server);
    println("\"");
    if (readResponse() != ResponseOK) {
        return 0;
    }

    // set port
    if (port != 80) {
        print("AT+UHTTP=0,5,");
        println(port);

        if (readResponse() != ResponseOK) {
            return 0;
        }
    }

    // before starting the actual http request, create any files needed in the fs of the modem
    // that way there is a chance to abort sending the http req command in case of an fs error
    if (requestType == PUT || requestType == POST) {
        if (!sendBuffer || sendSize == 0) {
            debugPrintLn(DEBUG_STR_ERROR "There is no sendBuffer or sendSize set!");
            return 0;
        }

        deleteFile(DEFAULT_HTTP_SEND_TMP_FILENAME); // cleanup the file first (if exists)

        if (!writeFile(DEFAULT_HTTP_SEND_TMP_FILENAME, (uint8_t*)sendBuffer, sendSize)) {
            debugPrintLn(DEBUG_STR_ERROR "Could not create the http tmp file!");
            return 0;
        }
    }

    // reset the success bit before calling a new request
    _httpRequestSuccessBit[requestType] = TriBoolUndefined;

    print("AT+UHTTPC=0,");
    print(_httpRequestTypeToModemIndex(requestType));
    print(",\"");
    print(endpoint);
    print("\",\"\""); // empty filename = default = "http_last_response_0" (DEFAULT_HTTP_RECEIVE_TMP_FILENAME)

    // NOTE: a file that includes the buffer to send has been created already
    if (requestType == PUT) {
        print(",\"" DEFAULT_HTTP_SEND_TMP_FILENAME "\""); // param1: file from filesystem to send
    }
    else if (requestType == POST) {
        print(",\"" DEFAULT_HTTP_SEND_TMP_FILENAME "\""); // param1: file from filesystem to send
        print(",1"); // param2: content type, 1=text/plain
        // TODO consider making the content type a parameter
    } else {
        // GET, etc
    }
    println("");

    if (readResponse() != ResponseOK) {
        return 0;
    }

    // check for success while checking URCs
    // This loop relies on readResponse being called via isAlive()
    uint32_t start = millis();
    uint32_t delay_count = 50;
    while ((_httpRequestSuccessBit[requestType] == TriBoolUndefined) && !is_timedout(start, 30000)) {
        isAlive();
        sodaq_wdt_safe_delay(delay_count);
        delay_count += 250;
    }

    if (_httpRequestSuccessBit[requestType] == TriBoolTrue) {
        if (responseBuffer && responseSize > 0) {
            return readFile(DEFAULT_HTTP_RECEIVE_TMP_FILENAME, (uint8_t*)responseBuffer, responseSize);
        }
    }
    else if (_httpRequestSuccessBit[requestType] == TriBoolFalse) {
        debugPrintLn(DEBUG_STR_ERROR "An error occurred with the http request!");
        return 0;
    }
    else {
        debugPrintLn(DEBUG_STR_ERROR "Timed out waiting for a response for the http request!");
        return 0;
    }

    return 0;
}

ResponseTypes Sodaq_3Gbee::_ulstfileParser(ResponseTypes& response, const char* buffer, size_t size,
        uint32_t* filesize, uint8_t* dummy)
{
    if (!filesize) {
        return ResponseError;
    }

    if (sscanf(buffer, "+ULSTFILE: %lu", filesize) == 1) {
        return ResponseEmpty;
    }

    return ResponseError;
}

// maps the given requestType to the index the modem recognizes, -1 if error
int Sodaq_3Gbee::_httpRequestTypeToModemIndex(HttpRequestTypes requestType)
{
    static uint8_t mapping[] = {
        4,      // 0 POST
        1,      // 1 GET
        0,      // 2 HEAD
        2,      // 3 DELETE
        3,      // 4 PUT
    };

    return (requestType < sizeof(mapping)) ? mapping[requestType] : -1;
}

// HttpRequestTypes if successful, -1 if not
int Sodaq_3Gbee::_httpModemIndexToRequestType(uint8_t modemIndex)
{
    static uint8_t mapping[] = {
        HEAD,   // 0
        GET,    // 1
        DELETE, // 2
        PUT,    // 3
        POST,   // 4
    };

    return (modemIndex < sizeof(mapping)) ? mapping[modemIndex] : -1;
}

// no string termination
size_t Sodaq_3Gbee::readFile(const char* filename, uint8_t* buffer, size_t size)
{
    // TODO escape filename characters { '"', ',', }

    //sanity check
    if (!buffer || size == 0) {
        return 0;
    }

    // first, make sure the buffer is sufficient
    print("AT+ULSTFILE=2,\"");
    print(filename);
    println("\"");

    uint32_t filesize = 0;

    if ((readResponse<uint32_t, uint8_t>(_ulstfileParser, &filesize, NULL) != ResponseOK) || (filesize > size)) {
        debugPrintLn(DEBUG_STR_ERROR "The buffer is not big enough to store the file or the file was not found!");

        return 0;
    }

    print("AT+URDFILE=\"");
    print(filename);
    println("\"");

    // override normal parsing process and explicitly read characters here
    // to be able to also read terminator characters within files
    char checkChar = 0;
    size_t len = 0;

    // reply identifier
    len = readBytesUntil(' ', _inputBuffer, _inputBufferSize);
    if (len == 0 || strstr(_inputBuffer, "+URDFILE:") == NULL) {
        debugPrintLn(DEBUG_STR_ERROR "+URDFILE literal is missing!");
        goto error;
    }

    // filename
    len = readBytesUntil(',', _inputBuffer, _inputBufferSize);
    // TODO check filename after removing quotes and escaping chars
    //if (len == 0 || strstr(_inputBuffer, filename)) {
    //    debugPrintLn(DEBUG_STR_ERROR "Filename reported back is not correct!");
    //    return 0;
    //}

    // filesize
    len = readBytesUntil(',', _inputBuffer, _inputBufferSize);
    filesize = 0; // reset the var before reading from reply string
    if (sscanf(_inputBuffer, "%lu", &filesize) != 1) {
        debugPrintLn(DEBUG_STR_ERROR "Could not parse the file size!");
        goto error;
    }
    if (filesize == 0 || filesize > size) {
        debugPrintLn(DEBUG_STR_ERROR "Size error!");
        goto error;
    }

    // opening quote character
    checkChar = timedRead();
    if (checkChar != '"') {
        debugPrintLn(DEBUG_STR_ERROR "Missing starting character (quote)!");
        goto error;
    }

    // actual file buffer, written directly to the provided result buffer
    len = readBytes(buffer, filesize);
    if (len != filesize) {
        debugPrintLn(DEBUG_STR_ERROR "File size error!");
        goto error;
    }

    // closing quote character
    checkChar = timedRead();
    if (checkChar != '"') {
        debugPrintLn(DEBUG_STR_ERROR "Missing termination character (quote)!");
        goto error;
    }

    // read final OK response from modem and return the filesize
    if (readResponse() == ResponseOK) {
        return filesize;
    }

error:
    return 0;
}

bool Sodaq_3Gbee::writeFile(const char* filename, const uint8_t* buffer, size_t size)
{
    // TODO escape filename characters
    print("AT+UDWNFILE=\"");
    print(filename);
    print("\",");
    println(size);

    if (readResponse() == ResponsePrompt) {
        for (size_t i = 0; i < size; i++) {
            print(buffer[i]);
        }

        return (readResponse() == ResponseOK);
    }

    return false;
}

bool Sodaq_3Gbee::deleteFile(const char* filename)
{
    // TODO escape filename characters
    print("AT+UDELFILE=\"");
    print(filename);
    println("\"");

    return (readResponse() == ResponseOK);
}

// Opens an FTP connection.
bool Sodaq_3Gbee::openFtpConnection(const char* server, const char* username, const char* password, FtpModes ftpMode)
{
    ftpDirectoryChangeCounter = 0;

    // set server
    print("AT+UFTP=");
    print(isValidIPv4(server) ? "0,\"" : "1,\"");
    print(server);
    println("\"");

    if (readResponse() != ResponseOK) {
        return false;
    }

    // set username
    print("AT+UFTP=2,\"");
    print(username);
    println("\"");

    if (readResponse() != ResponseOK) {
        return false;
    }

    // set password
    print("AT+UFTP=3,\"");
    print(password);
    println("\"");

    if (readResponse() != ResponseOK) {
        return false;
    }

    // set passive / active
    print("AT+UFTP=6,");
    println(ftpMode == ActiveMode ? 0 : 1);

    if (readResponse() != ResponseOK) {
        return false;
    }

    // connect
    println("AT+UFTPC=1");

    if ((readResponse() != ResponseOK) || (!waitForFtpCommandResult(1))) {
        return false;
    }

    return true;
}

// Closes the FTP connection.
bool Sodaq_3Gbee::closeFtpConnection()
{
    ftpDirectoryChangeCounter = 0;

    println("AT+UFTPC=0");

    if ((readResponse() != ResponseOK) || (!waitForFtpCommandResult(0))) {
        return false;
    }

    return true;
}

// Opens an FTP file for sending or receiving.
// filename should be limited to 256 characters (excl. null terminator)
// path should be limited to 512 characters (excl. null temrinator)
bool Sodaq_3Gbee::openFtpFile(const char* filename, const char* path)
{
    // keep the filename for subsequent calls to send or receive data
    strncpy(ftpFilename, filename, sizeof(ftpFilename)-1);
    ftpFilename[sizeof(ftpFilename) - 1] = 0; // always null terminated, even when given filename has length > sizeof(ftpFilename)-1

    resetFtpDirectoryIfNeeded();
    if (path) {
        char pathBuffer[512 + 1];
        strncpy(pathBuffer, path, sizeof(pathBuffer)-1);
        pathBuffer[sizeof(pathBuffer) - 1] = '\0'; // secure from overflow

        char* pathPointer = strtok(pathBuffer, "/");
        while (pathPointer != NULL)
        {
            if (!changeFtpDirectory(pathPointer)) {
                // TODO do something here?
            }
            pathPointer = strtok(NULL, "/");
        }
    }

    return true;
}

// Sends the given "buffer" to the (already) open FTP file.
// Returns true if successful.
// Fails immediately if there is no open FTP file.
bool Sodaq_3Gbee::ftpSend(const char* buffer)
{
    return ftpSend((uint8_t*)buffer, strlen(buffer));
}

// Sends the given "buffer" to the (already) open FTP file.
// Returns true if successful.
// Fails immediately if there is no open FTP file.
bool Sodaq_3Gbee::ftpSend(const uint8_t* buffer, size_t size)
{
    // quick sanity check
    if (ftpFilename[0] == '\0') {
        return false;
    }

    deleteFile(DEFAULT_FTP_TMP_FILENAME); // cleanup

    if (!writeFile(DEFAULT_FTP_TMP_FILENAME, buffer, size)) {
        deleteFile(DEFAULT_FTP_TMP_FILENAME);
        return false;
    }

    print("AT+UFTPC=5,\"" DEFAULT_FTP_TMP_FILENAME "\",\"");
    print(ftpFilename);
    println("\"");

    if ((readResponse() != ResponseOK) || (!waitForFtpCommandResult(5))) {
        return false;
    }

    return true;
}

// Fills the given "buffer" from the (already) open FTP file.
// Returns true if successful.
// Fails immediately if there is no open FTP file.
int Sodaq_3Gbee::ftpReceive(char* buffer, size_t size)
{
    // quick sanity check
    if (ftpFilename[0] == '\0') {
        return 0;
    }

    deleteFile(DEFAULT_FTP_TMP_FILENAME); // cleanup

    print("AT+UFTPC=4,\"");
    print(ftpFilename);
    println("\",\"" DEFAULT_FTP_TMP_FILENAME "\"");

    if ((readResponse() != ResponseOK) || (!waitForFtpCommandResult(4))) {
        return 0;
    }

    return readFile(DEFAULT_FTP_TMP_FILENAME, (uint8_t*)buffer, size);
}

// Closes the open FTP file.
// Returns true if successful.
// Fails immediatelly if there is no open FTP file.
bool Sodaq_3Gbee::closeFtpFile()
{
    resetFtpDirectoryIfNeeded();
    ftpFilename[0] = '\0'; // invalidate the filename
    return true;
}

ResponseTypes Sodaq_3Gbee::_cmglParser(ResponseTypes& response, const char* buffer, size_t size,
        int* indexList, size_t* indexListSize)
{
    if (!indexList || !indexListSize || *indexListSize <= 0) {
        return ResponseError;
    }

    int index;
    if (sscanf(buffer, "+CMGL: %d,", &index) == 1) {
        *(indexList++) = index;
        (*indexListSize)--;

        return ResponseEmpty;
    }

    return ResponseError;
}

// TODO test
// Gets an SMS list according to the given filter and puts the indexes in the "indexList".
// Returns the number of indexes written to the list or -1 in case of error.
int Sodaq_3Gbee::getSmsList(const char* statusFilter, int* indexList, size_t size)
{
    print("AT+CMGL=\"");
    print(statusFilter);
    println("\"");

    size_t sizeParam = size;
    if (readResponse<int, size_t>(_cmglParser, indexList, &sizeParam) == ResponseOK) {
        return size - sizeParam; // the parser method subtracts from the total size when adding an index in the list
    }

    return -1;
}

ResponseTypes Sodaq_3Gbee::_cmgrParser(ResponseTypes& response, const char* buffer, size_t size,
        char* phoneNumber, char* smsBuffer)
{
    if (!phoneNumber || !smsBuffer) {
        return ResponseError;
    }

    if (sscanf(buffer, "+CMGR: \"%*[^\"]\",\"%[^\"]", phoneNumber) == 1) {
        return ResponseEmpty;
    }
    else if ((buffer[size - 2] == '\r') && (buffer[size - 1] == '\n')) {
        memcpy(smsBuffer, buffer, size - 2);
        smsBuffer[size - 2] = '\0';

        return ResponseEmpty;
    }

    return ResponseError;
}

// TODO test
// Reads an SMS from the given index and writes it to the given buffer.
// Returns true if successful.
bool Sodaq_3Gbee::readSms(uint8_t index, char* phoneNumber, char* buffer, size_t size)
{
    print("AT+CMGR=");
    println(index);

    return (readResponse<char, char>(_cmgrParser, phoneNumber, buffer) == ResponseOK);
}

// TODO test
// Deletes the SMS at the given index.
bool Sodaq_3Gbee::deleteSms(uint8_t index)
{
    print("AT+CMGD=");
    println(index);

    return (readResponse() == ResponseOK);
}

// TODO test
// Sends a text-mode SMS.
// Expects a null-terminated buffer.
// Returns true if successful.
bool Sodaq_3Gbee::sendSms(const char* phoneNumber, const char* buffer)
{
    print("AT+CMGS=\"");
    print(phoneNumber);
    println("\"");

    if (readResponse() == ResponsePrompt) {
        for (size_t i = 0; i < strlen(buffer); i++) {
            print(buffer[i]);
        }

        print(CTRL_Z);

        return (readResponse() == ResponseOK);
    }

    return false;
}

////////////////////////////////////////////////////////////////////////////////
////////////////////    MQTT               /////////////////////////////////////
////////////////////////////////////////////////////////////////////////////////

bool Sodaq_3Gbee::openMQTT(const char * server, uint16_t port)
{
    return openTCP(_apn, _apnUser, _apnPass, server, port);
}

bool Sodaq_3Gbee::closeMQTT(bool switchOff)
{
    closeTCP(switchOff);
    return true;        // Always succeed
}

bool Sodaq_3Gbee::sendMQTTPacket(uint8_t * pckt, size_t len)
{
    return sendDataTCP(pckt, len);
}

/*
 * Read the MQTT packet
 *
 * \returns The number of bytes in the packet. This can be larger than what is stored in the buffer.
 * Return 0 if it failed to read the packet.
 */
size_t Sodaq_3Gbee::receiveMQTTPacket(uint8_t * pckt, size_t size, uint32_t timeout)
{
    // TODO While loop until packet received, or timed out
    isAlive();
    if (_openTCPsocket < 0) {
        return 0;
    }
    size_t retval = 0;
    uint32_t start = millis();
    while (!is_timedout(start, timeout)) {
        if (_openTCPsocket < 0) {
            break;
        }
        size_t nrbytes = socketBytesPending(_openTCPsocket);
        if (nrbytes > 0) {
            size_t read_len = nrbytes;
            if (nrbytes > size) {
                read_len = size;
                // TODO What do we do with the remaining bytes after we have read this amount?
            }
            if (receiveDataTCP(pckt, read_len)) {
                retval = nrbytes;
                break;
            }
        }
    }
    return retval;
}

/*
 * Is the MQTT connection alive?
 */
bool Sodaq_3Gbee::isAliveMQTT()
{
    return _openTCPsocket >= 0;
}

/*
 * Read the MQTT packet
 *
 * \returns The number of bytes available in a received packet.
 * Return 0 if no packet is available.
 */
size_t Sodaq_3Gbee::availableMQTTPacket()
{
    isAlive();
    if (_openTCPsocket < 0) {
        return 0;
    }
    return socketBytesPending(_openTCPsocket);
}

////////////////////////////////////////////////////////////////////////////////
////////////////////    Sodaq_3GbeeOnOff       /////////////////////////////////
////////////////////////////////////////////////////////////////////////////////

Sodaq_3GbeeOnOff::Sodaq_3GbeeOnOff()
{
    _vcc33Pin = -1;
    _onoffPin = -1;
    _statusPin = -1;
    _onoff_status = false;
}

// Initializes the instance
void Sodaq_3GbeeOnOff::init(int vcc33Pin, int onoffPin, int statusPin)
{
    if (vcc33Pin >= 0) {
      _vcc33Pin = vcc33Pin;
      // First write the output value, and only then set the output mode.
      digitalWrite(_vcc33Pin, LOW);
      pinMode(_vcc33Pin, OUTPUT);
    }

    if (onoffPin >= 0) {
      _onoffPin = onoffPin;
      // First write the output value, and only then set the output mode.
      digitalWrite(_onoffPin, LOW);
      pinMode(_onoffPin, OUTPUT);
    }

    if (statusPin >= 0) {
      _statusPin = statusPin;
      pinMode(_statusPin, INPUT);
    }
}

void Sodaq_3GbeeOnOff::on()
{
    // First VCC 3.3 HIGH
    if (_vcc33Pin >= 0) {
        digitalWrite(_vcc33Pin, HIGH);
        // Wait a little
        // TODO Figure out if this is really needed
        delay(2);
    }

    if (_onoffPin >= 0) {
        digitalWrite(_onoffPin, HIGH);
    }
    _onoff_status = true;
}

void Sodaq_3GbeeOnOff::off()
{
    if (_vcc33Pin >= 0) {
        digitalWrite(_vcc33Pin, LOW);
    }

    // The GPRSbee is switched off immediately
    if (_onoffPin >= 0) {
        digitalWrite(_onoffPin, LOW);
    }

    // Should be instant
    // Let's wait a little, but not too long
    delay(50);
    _onoff_status = false;
}

bool Sodaq_3GbeeOnOff::isOn()
{
    if (_statusPin >= 0) {
        bool status = digitalRead(_statusPin);
        return status;
    } else {
#if defined(ARDUINO_ARCH_AVR)
      // Use the onoff pin, which is close to useless
      bool status = digitalRead(_onoffPin);
      return status;
#elif defined(ARDUINO_ARCH_SAMD)
      // There is no status pin. On SAMD we cannot read back the onoff pin.
      // So, our own status is all we have.
      return _onoff_status;
#endif
    }

    // No status pin. Let's assume it is on.
    return true;
}
