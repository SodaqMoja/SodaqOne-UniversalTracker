#include <Arduino.h>
#include <Wire.h>
#include "RTCTimer.h"
#include "RTCZero.h"
#include "Sodaq_RN2483.h"
#include "Sodaq_wdt.h"
#include "Config.h"
#include "BootMenu.h"
#include "ublox.h"
#include "MyTime.h"
#include "ReportDataRecord.h"
#include "GpsFixDataRecord.h"
#include "OverTheAirConfigDataRecord.h"
#include "GpsFixLiFoRingBuffer.h"
#include "LSM303.h"

//#define DEBUG

#define PROJECT_NAME "SodaqOne Universal Tracker"
#define VERSION "1.4"
#define STARTUP_DELAY 5000

#define GPS_TIME_VALIDITY 0b00000011 // date and time (but not fully resolved)
#define GPS_FIX_FLAGS 0b00000001 // just gnssFixOK

#define MAX_RTC_EPOCH_OFFSET 25

#define ADC_AREF 3.3f
#define BATVOLT_R1 2.0f
#define BATVOLT_R2 2.0f
#define BATVOLT_PIN A5

#define DEBUG_STREAM SerialUSB
#define CONSOLE_STREAM SerialUSB
#define LORA_STREAM Serial1

#define NIBBLE_TO_HEX_CHAR(i) ((i <= 9) ? ('0' + i) : ('A' - 10 + i))
#define HIGH_NIBBLE(i) ((i >> 4) & 0x0F)
#define LOW_NIBBLE(i) (i & 0x0F)

// version of "hex to bin" macro that supports both lower and upper case
#define HEX_CHAR_TO_NIBBLE(c) ((c >= 'a') ? (c - 'a' + 0x0A) : ((c >= 'A') ? (c - 'A' + 0x0A) : (c - '0')))
#define HEX_PAIR_TO_BYTE(h, l) ((HEX_CHAR_TO_NIBBLE(h) << 4) + HEX_CHAR_TO_NIBBLE(l))

#define consolePrint(x) CONSOLE_STREAM.print(x)
#define consolePrintln(x) CONSOLE_STREAM.println(x)

#ifdef DEBUG
#define debugPrint(x) DEBUG_STREAM.print(x)
#define debugPrintln(x) DEBUG_STREAM.println(x)
#else
#define debugPrint(x)
#define debugPrintln(x)
#endif


enum LedColor {
    NONE = 0,
    RED,
    GREEN,
    BLUE
};


RTCZero rtc;
RTCTimer timer;
UBlox ublox;
Time time;
LSM303 lsm303;

ReportDataRecord pendingReportDataRecord;
bool isPendingReportDataRecordNew; // this is set to true only when pendingReportDataRecord is written by the delegate

volatile bool minuteFlag;

static uint8_t lastResetCause;
static bool isLoraInitialized;
static bool isRtcInitialized;
static bool isDeviceInitialized;
static int64_t rtcEpochDelta; // set in setNow() and used in getGpsFixAndTransmit() for correcting time in loop

static uint8_t receiveBuffer[16];
static uint8_t receiveBufferSize;
static uint8_t sendBuffer[51];
static uint8_t sendBufferSize;
static uint8_t loraHWEui[8];
static bool isLoraHWEuiInitialized;


void setup();
void loop();

uint32_t getNow();
void setNow(uint32_t now);
void handleBootUpCommands();
void initRtc();
void rtcAlarmHandler();
void initRtcTimer();
void resetRtcTimerEvents();
void initSleep();
bool initLora();
void systemSleep();
void runDefaultFixEvent(uint32_t now);
void runAlternativeFixEvent(uint32_t now);
void runLoraModuleSleepExtendEvent(uint32_t now);
void setLedColor(LedColor color);
void setGpsActive(bool on);
void setLoraActive(bool on);
void setLsm303Active(bool on);
bool convertAndCheckHexArray(uint8_t* result, const char* hex, size_t resultSize);
bool isAlternativeFixEventApplicable();
bool isCurrentTimeOfDayWithin(uint32_t daySecondsFrom, uint32_t daySecondsTo);
void delegateNavPvt(NavigationPositionVelocityTimeSolution* NavPvt);
bool getGpsFixAndTransmit();
uint8_t getBatteryVoltage();
int8_t getBoardTemperature();
void updateSendBuffer();
void transmit();
void updateConfigOverTheAir();
void resetPin(uint8_t pin);
void resetAllDigitalPins();
void setDevAddrOrEUItoHWEUI(void);
void onConfigReset(void);

static void printCpuResetCause(Stream& stream);
static void printBootUpMessage(Stream& stream);


void setup()
{
    // Allow power to remain on
    pinMode(ENABLE_PIN_IO, OUTPUT);
    digitalWrite(ENABLE_PIN_IO, HIGH);

    lastResetCause = PM->RCAUSE.reg;
    sodaq_wdt_enable();
    sodaq_wdt_reset();

    SerialUSB.begin(115200);
    setLedColor(RED);
    sodaq_wdt_safe_delay(STARTUP_DELAY);
    printBootUpMessage(SerialUSB);

    gpsFixLiFoRingBuffer_init();
    initSleep();
    initRtc();

    Wire.begin();
    ublox.enable(); // turn power on early for faster initial fix

    // init params
    params.setConfigResetCallback(onConfigReset);
    params.read();

    // disable the watchdog only for the boot menu
    sodaq_wdt_disable();
    handleBootUpCommands();
    sodaq_wdt_enable();

    isLoraInitialized = initLora();
    initRtcTimer();

    isDeviceInitialized = true;

    consolePrintln("** Boot-up completed successfully!");
    sodaq_wdt_reset();

    // disable the USB if the app is not in debug mode
#ifndef DEBUG
    consolePrintln("The USB is going to be disabled now.");
    SerialUSB.flush();
    sodaq_wdt_safe_delay(3000);
    SerialUSB.end();
    USBDevice.detach();
    USB->DEVICE.CTRLA.reg &= ~USB_CTRLA_ENABLE; // Disable USB
#endif

    if (getGpsFixAndTransmit()) {
        setLedColor(GREEN);
        sodaq_wdt_safe_delay(800);
    }
}

void loop()
{
    // Reset watchdog
    sodaq_wdt_reset();
    sodaq_wdt_flag = false;

    if (minuteFlag) {

        if (params.getIsLedEnabled()) {
            setLedColor(BLUE);
        }

        timer.update(); // handle scheduled events

        minuteFlag = false;
    }

    systemSleep();
}

/**
 * Initializes the CPU sleep mode.
 */
void initSleep()
{
    // Set the sleep mode
    SCB->SCR |= SCB_SCR_SLEEPDEEP_Msk;
}

/**
 * Returns the battery voltage minus 3 volts.
 */
uint8_t getBatteryVoltage()
{
    uint16_t voltage = (uint16_t)((ADC_AREF / 1.023) * (BATVOLT_R1 + BATVOLT_R2) / BATVOLT_R2 * (float)analogRead(BATVOLT_PIN));
    voltage = (voltage - 3000) / 10;

    return (voltage > 255 ? 255 : (uint8_t)voltage);
}

/**
 * Returns the board temperature.
*/
int8_t getBoardTemperature()
{
    setLsm303Active(true);

    uint8_t tempL = lsm303.readReg(LSM303::TEMP_OUT_L);
    uint8_t tempH = lsm303.readReg(LSM303::TEMP_OUT_H);

    int16_t temp = (int16_t)(((uint16_t)tempH << 8) | tempL);

    setLsm303Active(false);

    return temp;
}

/**
 * Updates the "sendBuffer" using the current "pendingReportDataRecord" and its "sendBufferSize".
 */
void updateSendBuffer()
{
    // copy the pendingReportDataRecord into the sendBuffer
    memcpy(sendBuffer, pendingReportDataRecord.getBuffer(), pendingReportDataRecord.getSize());
    sendBufferSize = pendingReportDataRecord.getSize();

    // copy the previous coordinates if applicable (-1 because one coordinate is already in the report record)
    GpsFixDataRecord record;
    for (uint8_t i = 0; i < params.getCoordinateUploadCount() - 1; i++) {
        record.init();

        // (skip first record because it is in the report record already)
        if (!gpsFixLiFoRingBuffer_peek(1 + i, &record)) {
            break;
        }

        if (!record.isValid()) {
            break;
        }

        record.updatePreviousFixValue(pendingReportDataRecord.getTimestamp());
        memcpy(&sendBuffer[sendBufferSize - 1], record.getBuffer(), record.getSize());
        sendBufferSize += record.getSize();
    }
}

/**
 * Sends the current sendBuffer through lora (if enabled).
 * Repeats the transmitions according to params.getRepeatCount().
*/
void transmit()
{
    if (!isLoraInitialized) {
        return;
    }

    setLoraActive(true);

    for (uint8_t i = 0; i < 1 + params.getRepeatCount(); i++) {
        if (LoRaBee.send(1, sendBuffer, sendBufferSize) != 0) {
            debugPrintln("There was an error while transmitting through LoRaWAN.");
        }
        else {
            debugPrintln("Data transmitted successfully.");

            uint16_t size = LoRaBee.receive(receiveBuffer, sizeof(receiveBuffer));
            receiveBufferSize = size;

            if (size > 0) {
                debugPrintln("Received OTA Configuration.");
                updateConfigOverTheAir();
            }
        }
    }

    setLoraActive(false);
}

/**
 * Uses the "receiveBuffer" (received from LoRaWAN) to update the configuration.
*/
void updateConfigOverTheAir()
{
    OverTheAirConfigDataRecord record;
    record.init();
    record.copyFrom(receiveBuffer, receiveBufferSize);

    if (record.isValid()) {
        params._defaultFixInterval = record.getDefaultFixInterval();
        params._alternativeFixInterval = record.getAlternativeFixInterval();

        // time of day seconds assumed
        params._alternativeFixFromHours = record.getAlternativeFixFrom() / 3600;
        params._alternativeFixFromMinutes = (record.getAlternativeFixFrom() - params._alternativeFixFromHours * 3600) / 60;

        params._alternativeFixToHours = record.getAlternativeFixTo() / 3600;
        params._alternativeFixToMinutes = (record.getAlternativeFixTo() - params._alternativeFixToHours * 3600) / 60;

        params._gpsFixTimeout = record.getGpsFixTimeout();

        // apply the rtc timer changes
        resetRtcTimerEvents();
    }
}

/**
 * Converts the given hex array and returns true if it is valid hex and non-zero.
 * "hex" is assumed to be 2*resultSize bytes.
 */
bool convertAndCheckHexArray(uint8_t* result, const char* hex, size_t resultSize)
{
    bool foundNonZero = false;

    uint16_t inputIndex = 0;
    uint16_t outputIndex = 0;

    // stop at the first string termination char, or if output buffer is over
    while (outputIndex < resultSize && hex[inputIndex] != 0 && hex[inputIndex + 1] != 0) {
        if (!isxdigit(hex[inputIndex]) || !isxdigit(hex[inputIndex + 1])) {
            return false;
        }

        result[outputIndex] = HEX_PAIR_TO_BYTE(hex[inputIndex], hex[inputIndex + 1]);

        if (result[outputIndex] > 0) {
            foundNonZero = true;
        }

        inputIndex += 2;
        outputIndex++;
    }

    result[outputIndex] = 0; // terminate the string

    return foundNonZero;
}

/**
 * Initializes the lora module.
 * Returns true if successful.
*/
bool initLora()
{
    consolePrintln("Initializing LoRa...");

    LORA_STREAM.begin(LoRaBee.getDefaultBaudRate());
#ifdef DEBUG
    LoRaBee.setDiag(DEBUG_STREAM);
#endif

    bool allParametersValid;
    bool result;
    if (params.getIsOtaaEnabled()) {
        uint8_t devEui[8];
        uint8_t appEui[8];
        uint8_t appKey[16];

        allParametersValid = convertAndCheckHexArray((uint8_t*)devEui, params.getDevAddrOrEUI(), sizeof(devEui))
            && convertAndCheckHexArray((uint8_t*)appEui, params.getAppSKeyOrEUI(), sizeof(appEui))
            && convertAndCheckHexArray((uint8_t*)appKey, params.getNwSKeyOrAppKey(), sizeof(appKey));

        // try to initialize the lorabee regardless the validity of the parameters,
        // in order to allow the sleeping mechanism to work
        if (LoRaBee.initOTA(LORA_STREAM, devEui, appEui, appKey, true)) {
            result = true;
        }
        else {
            consolePrintln("LoRa init failed!");

            result = false;
        }
    }
    else {
        uint8_t devAddr[4];
        uint8_t appSKey[16];
        uint8_t nwkSKey[16];

        allParametersValid = convertAndCheckHexArray((uint8_t*)devAddr, params.getDevAddrOrEUI(), sizeof(devAddr))
            && convertAndCheckHexArray((uint8_t*)appSKey, params.getAppSKeyOrEUI(), sizeof(appSKey))
            && convertAndCheckHexArray((uint8_t*)nwkSKey, params.getNwSKeyOrAppKey(), sizeof(nwkSKey));

        // try to initialize the lorabee regardless the validity of the parameters,
        // in order to allow the sleeping mechanism to work
        if (LoRaBee.initABP(LORA_STREAM, devAddr, appSKey, nwkSKey, true)) {
            result = true;
        }
        else {
            consolePrintln("LoRa init failed!");

            result = false;
        }
    }

    if (!allParametersValid) {
        consolePrintln("The parameters for LoRa are not valid. LoRa will not be enabled.");

        result = false; // override the result from the initialization above
    }

    setLoraActive(false);
    return result; // false by default
}

/**
 * Powers down all devices and puts the system to deep sleep.
 */
void systemSleep()
{
    setLedColor(NONE);
    setGpsActive(false); // explicitly disable after resetting the pins

    sodaq_wdt_disable();

    // do not go to sleep if DEBUG is enabled, to keep USB connected
#ifndef DEBUG
    noInterrupts();
    if (!(sodaq_wdt_flag || minuteFlag)) {
        interrupts();

        __WFI(); // SAMD sleep
    }
    interrupts();
#endif

    // Re-enable watchdog
    sodaq_wdt_enable();
}

/**
 * Returns the current datetime (seconds since unix epoch).
 */
uint32_t getNow()
{
    return rtc.getEpoch();
}

/**
 * Sets the RTC epoch and "rtcEpochDelta".
 */
void setNow(uint32_t newEpoch)
{
    uint32_t currentEpoch = getNow();

    debugPrint("Setting RTC from ");
    debugPrint(currentEpoch);
    debugPrint(" to ");
    debugPrintln(newEpoch);

    rtcEpochDelta = newEpoch - currentEpoch;
    rtc.setEpoch(newEpoch);

    timer.adjust(currentEpoch, newEpoch);

    isRtcInitialized = true;
}

/**
 * Shows and handles the boot up commands.
 */
void handleBootUpCommands()
{
    setResetDevAddrOrEUItoHWEUICallback(setDevAddrOrEUItoHWEUI);

    do {
        showBootMenu(CONSOLE_STREAM);
    } while (!params.checkConfig(CONSOLE_STREAM));

    params.showConfig(&CONSOLE_STREAM);
    params.commit();
}

/**
 * Initializes the RTC.
 */
void initRtc()
{
    rtc.begin();

    // Schedule the wakeup interrupt for every minute
    // Alarm is triggered 1 cycle after match
    rtc.setAlarmSeconds(59);
    rtc.enableAlarm(RTCZero::MATCH_SS); // alarm every minute

    // Attach handler
    rtc.attachInterrupt(rtcAlarmHandler);

    // This sets it to 2000-01-01
    rtc.setEpoch(0);
}

/**
 * Runs every minute by the rtc alarm.
*/
void rtcAlarmHandler()
{
    minuteFlag = true;
}

/**
 * Initializes the RTC Timer and schedules the default events.
 */
void initRtcTimer()
{
    timer.setNowCallback(getNow); // set how to get the current time
    timer.allowMultipleEvents();

    resetRtcTimerEvents();
}

/**
 * Clears the RTC Timer events and schedules the default events.
 */
void resetRtcTimerEvents()
{
    timer.clearAllEvents();

    // Schedule the default fix event
    timer.every(params.getDefaultFixInterval() * 60, runDefaultFixEvent);

    // check if the alternative fix event should be scheduled at all
    if (params.getAlternativeFixInterval() > 0) {
        // Schedule the alternative fix event
        timer.every(params.getAlternativeFixInterval() * 60, runAlternativeFixEvent);
    }

    // if lora is not enabled, schedule an event that takes care of extending the sleep time of the module
    if (!isLoraInitialized) {
        timer.every(24 * 60 * 60, runLoraModuleSleepExtendEvent); // once a day
    }
}

/**
 * Returns true if the alternative fix event should run at the current time.
*/
bool isAlternativeFixEventApplicable()
{
    // - RTC should be initialized (synced time)
    // - alternative fix interval should be set
    // - the span between FROM and TO should be at least as much as the alternative fix interval
    // - current time should be within the FROM and TO times set
    return (isRtcInitialized
        && (params.getAlternativeFixInterval() > 0)
        && (params.getAlternativeFixTo() - params.getAlternativeFixFrom() >= params.getAlternativeFixInterval() * 60)
        && (isCurrentTimeOfDayWithin(params.getAlternativeFixFrom(), params.getAlternativeFixTo())));
}

/**
 * Returns true if the current rtc time is within the given times of day (in seconds).
*/
bool isCurrentTimeOfDayWithin(uint32_t daySecondsFrom, uint32_t daySecondsTo)
{
    uint32_t daySecondsCurrent = rtc.getHours() * 60 * 60 + rtc.getMinutes() * 60;

    return (daySecondsCurrent >= daySecondsFrom && daySecondsCurrent < daySecondsTo);
}

/**
 * Runs the default fix event sequence (only if applicable).
 */
void runDefaultFixEvent(uint32_t now)
{
    if (!isAlternativeFixEventApplicable()) {
        debugPrintln("Default fix event started.");
        getGpsFixAndTransmit();
    }
}

/**
 * Runs the alternative fix event sequence (only if it set and is within the set time).
 */
void runAlternativeFixEvent(uint32_t now)
{
    if (isAlternativeFixEventApplicable()) {
        debugPrintln("Alternative fix event started.");
        getGpsFixAndTransmit();
    }
}

/**
 * Wakes up the lora module to put it back to sleep, i.e. extends the sleep period
*/
void runLoraModuleSleepExtendEvent(uint32_t now)
{
    debugPrintln("Extending LoRa module sleep period.");

    setLoraActive(true);
    sodaq_wdt_safe_delay(80);
    setLoraActive(false);
}

/**
 * Turns the led on according to the given color. Makes no assumptions about the status of the pins
 * i.e. it sets them every time,
 */
void setLedColor(LedColor color)
{
    pinMode(LED_RED, OUTPUT);
    pinMode(LED_GREEN, OUTPUT);
    pinMode(LED_BLUE, OUTPUT);

    digitalWrite(LED_RED, HIGH);
    digitalWrite(LED_GREEN, HIGH);
    digitalWrite(LED_BLUE, HIGH);

    switch (color)
    {
    case NONE:
        break;
    case RED:
        digitalWrite(LED_RED, LOW);
        break;
    case GREEN:
        digitalWrite(LED_GREEN, LOW);
        break;
    case BLUE:
        digitalWrite(LED_BLUE, LOW);
        break;
    default:
        break;
    }
}

/**
 *  Checks validity of data, adds valid points to the points list, syncs the RTC
 */
void delegateNavPvt(NavigationPositionVelocityTimeSolution* NavPvt)
{
    sodaq_wdt_reset();

    // note: db_printf gets enabled/disabled according to the "DEBUG" define (ublox.cpp)
    ublox.db_printf("%4.4d-%2.2d-%2.2d %2.2d:%2.2d:%2.2d.%d valid=%2.2x lat=%d lon=%d sats=%d fixType=%2.2x\r\n",
        NavPvt->year, NavPvt->month, NavPvt->day,
        NavPvt->hour, NavPvt->minute, NavPvt->seconds, NavPvt->nano, NavPvt->valid,
        NavPvt->lat, NavPvt->lon, NavPvt->numSV, NavPvt->fixType);

    // sync the RTC time
    if ((NavPvt->valid & GPS_TIME_VALIDITY) == GPS_TIME_VALIDITY) {
        uint32_t epoch = time.mktime(NavPvt->year, NavPvt->month, NavPvt->day, NavPvt->hour, NavPvt->minute, NavPvt->seconds);

        // check if there is an actual offset before setting the RTC
        if (abs((int64_t)getNow() - (int64_t)epoch) > MAX_RTC_EPOCH_OFFSET) {
            setNow(epoch);
        }
    }

    // check that the fix is OK and that it is a 3d fix or GNSS + dead reckoning combined
    if (((NavPvt->flags & GPS_FIX_FLAGS) == GPS_FIX_FLAGS) && ((NavPvt->fixType == 3) || (NavPvt->fixType == 4))) {
        pendingReportDataRecord.setAltitude(NavPvt->hMSL < 0 ? 0xFFFF : (uint16_t)(NavPvt->hMSL / 1000)); // mm to m
        pendingReportDataRecord.setCourse(NavPvt->heading);
        pendingReportDataRecord.setLat(NavPvt->lat);
        pendingReportDataRecord.setLong(NavPvt->lon);
        pendingReportDataRecord.setSatelliteCount(NavPvt->numSV);
        pendingReportDataRecord.setSpeed((NavPvt->gSpeed * 36) / 10000); // mm/s converted to km/h

        isPendingReportDataRecordNew = true;
    }
}

/**
 * Tries to get a GPS fix and sends the data through LoRa if applicable.
 * Times-out after params.getGpsFixTimeout seconds.
 * Please see the documentation for more details on how this process works.
 */
bool getGpsFixAndTransmit()
{
    debugPrintln("Starting getGpsFixAndTransmit()...");

    bool isSuccessful = false;
    setGpsActive(true);

    uint32_t startTime = getNow();
    while (getNow() - startTime <= params.getGpsFixTimeout())
    {
        sodaq_wdt_reset();
        uint16_t bytes = ublox.available();

        if (bytes) {
            rtcEpochDelta = 0;
            isPendingReportDataRecordNew = false;
            ublox.GetPeriodic(bytes); // calls the delegate method for passing results

            startTime += rtcEpochDelta; // just in case the clock was changed (by the delegate in ublox.GetPeriodic)

            if (isPendingReportDataRecordNew) {
                isSuccessful = true;
                break;
            }
        }
    }

    setGpsActive(false); // turn off gps as soon as it is not needed

    // populate all fields of the report record
    pendingReportDataRecord.setTimestamp(getNow());
    pendingReportDataRecord.setBatteryVoltage(getBatteryVoltage());
    pendingReportDataRecord.setBoardTemperature(getBoardTemperature());

    GpsFixDataRecord record;
    record.init();
    if (isSuccessful) {
        pendingReportDataRecord.setTimeToFix(pendingReportDataRecord.getTimestamp() - startTime);

        // add the new gpsFixDataRecord to the ringBuffer
        record.setLat(pendingReportDataRecord.getLat());
        record.setLong(pendingReportDataRecord.getLong());
        record.setTimestamp(pendingReportDataRecord.getTimestamp());

        gpsFixLiFoRingBuffer_push(&record);
    }
    else {
        pendingReportDataRecord.setTimeToFix(0xFF);

        // no need to check the buffer or the record for validity, default for Lat/Long is 0 anyway
        gpsFixLiFoRingBuffer_peek(0, &record);
        pendingReportDataRecord.setLat(record.getLat());
        pendingReportDataRecord.setLong(record.getLong());
    }

#ifdef DEBUG
    pendingReportDataRecord.printHeaderLn(&DEBUG_STREAM);
    pendingReportDataRecord.printRecordLn(&DEBUG_STREAM);
    debugPrintln();
#endif
    updateSendBuffer();
    transmit();

    return isSuccessful;
}

/**
 * Turns the GPS on or off.
 */
void setGpsActive(bool on)
{
    sodaq_wdt_reset();

    if (on) {
        pinMode(GPS_ENABLE, OUTPUT);
        pinMode(GPS_TIMEPULSE, INPUT);

        ublox.enable();
        ublox.flush();

        PortConfigurationDDC pcd;
        if (ublox.getPortConfigurationDDC(&pcd)) {
            pcd.outProtoMask = 1; // Disable NMEA
            ublox.setPortConfigurationDDC(&pcd);

            ublox.CfgMsg(UBX_NAV_PVT, 1); // Navigation Position Velocity TimeSolution
            ublox.funcNavPvt = delegateNavPvt;
        }
        else {
            debugPrintln("uBlox.getPortConfigurationDDC(&pcd) failed!");
        }
    }
    else {
        ublox.disable();
    }
}

/**
 * Turns the LoRa module on or off (wake up or sleep)
 */
void setLoraActive(bool on)
{
    sodaq_wdt_reset();

    if (on) {
        LoRaBee.wakeUp();
    }
    else {
        LoRaBee.sleep();
    }
}

/**
* Initializes the LSM303 or puts it in power-down mode.
*/
void setLsm303Active(bool on)
{
    if (on) {
        if (!lsm303.init(LSM303::device_D, LSM303::sa0_low)) {
            debugPrintln("Initialization of the LSM303 failed!");
            return;
        }

        lsm303.enableDefault();
        lsm303.writeReg(LSM303::CTRL5, lsm303.readReg(LSM303::CTRL5) | 0b10000000);
        sodaq_wdt_safe_delay(100);
    }
    else {
        // disable accelerometer, power-down mode
        lsm303.writeReg(LSM303::CTRL1, 0);

        // zero CTRL5 (including turn off TEMP sensor)
        lsm303.writeReg(LSM303::CTRL5, 0);

        // disable magnetometer, power-down mode
        lsm303.writeReg(LSM303::CTRL7, 0b00000010);
    }
}

/**
 * Prints the cause of the last reset to the given stream.
 *
 * It uses the PM->RCAUSE register to detect the cause of the last reset.
 */
static void printCpuResetCause(Stream& stream)
{
    stream.print("CPU reset by");

    if (PM->RCAUSE.bit.SYST) {
        stream.print(" Software");
    }

    // Syntax error due to #define WDT in CMSIS/4.0.0-atmel/Device/ATMEL/samd21/include/samd21j18a.h
    // if (PM->RCAUSE.bit.WDT) {
    if ((PM->RCAUSE.reg & PM_RCAUSE_WDT) != 0) {
        stream.print(" Watchdog");
    }

    if (PM->RCAUSE.bit.EXT) {
        stream.print(" External");
    }

    if (PM->RCAUSE.bit.BOD33) {
        stream.print(" BOD33");
    }

    if (PM->RCAUSE.bit.BOD12) {
        stream.print(" BOD12");
    }

    if (PM->RCAUSE.bit.POR) {
        stream.print(" Power On Reset");
    }

    stream.print(" [");
    stream.print(PM->RCAUSE.reg);
    stream.println("]");
}

/**
 * Prints a boot-up message that includes project name, version,
 * and Cpu reset cause.
 */
static void printBootUpMessage(Stream& stream)
{
    stream.println("** " PROJECT_NAME " - " VERSION " **");

    stream.print(" -> ");
    printCpuResetCause(stream);

    stream.println();
}

/**
 * Callback from Config.reset(), used to override default values.
 */
void onConfigReset(void)
{
    setDevAddrOrEUItoHWEUI();
}

void setDevAddrOrEUItoHWEUI(void)
{
    // only read the HWEUI once
    if (!isLoraHWEuiInitialized) {
        initLora();
        setLoraActive(true);
        uint8_t len = LoRaBee.getHWEUI(loraHWEui, sizeof(loraHWEui));
        setLoraActive(false);

        if (len == sizeof(loraHWEui)) {
            isLoraHWEuiInitialized = true;
        }
    }

    if (isLoraHWEuiInitialized) {
        for (uint8_t i = 0; i < sizeof(loraHWEui); i++) {
            params._devAddrOrEUI[i * 2] = NIBBLE_TO_HEX_CHAR(HIGH_NIBBLE(loraHWEui[i]));
            params._devAddrOrEUI[i * 2 + 1] = NIBBLE_TO_HEX_CHAR(LOW_NIBBLE(loraHWEui[i]));
        }
    }
}
