// TODO config record (lora receive)
// TODO transmition record (lora send)
// TODO lora send+receive
// TODO last 4 fix points array

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

#include "version.h"


#define DEBUG

#define PROJECT_NAME "Generic Tracker"
#define STARTUP_DELAY 5000

#define GPS_TIME_VALIDITY 0b00000011 // date and time (but not fully resolved)
#define GPS_FIX_FLAGS 0b00000001 // just gnssFixOK

#define MAX_RTC_EPOCH_OFFSET 25

#define DEBUG_STREAM SerialUSB
#define CONSOLE_STREAM SerialUSB
#define LORA_STREAM Serial1

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
#define debugPrintLn(x)
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

volatile bool minuteFlag;

static uint8_t lastResetCause;
static bool isLoraInitialized;
static bool isRtcInitialized;
static bool isDeviceInitialized;


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
bool convertAndCheckHexArray(uint8_t* result, const char* hex, size_t resultSize);
bool isAlternativeFixEventApplicable();
bool isCurrentTimeOfDayWithin(uint32_t daySecondsFrom, uint32_t daySecondsTo);
void delegateNavPvt(NavigationPositionVelocityTimeSolution* NavPvt);
bool getGpsFix();
//void transmit();

static void printCpuResetCause(Stream& stream);
static void printBootUpMessage(Stream& stream);


void setup()
{
    lastResetCause = PM->RCAUSE.reg;
    sodaq_wdt_disable();

    SerialUSB.begin(115200);
    setLedColor(RED);
    sodaq_wdt_safe_delay(STARTUP_DELAY);
    printBootUpMessage(SerialUSB);

    initSleep();
    initRtc();
    handleBootUpCommands();
    isLoraInitialized = initLora();
    setGpsActive(true);
    initRtcTimer();

    isDeviceInitialized = true;

    consolePrintln("** Boot-up completed successfully!");
    setLedColor(GREEN);
    sodaq_wdt_safe_delay(2000);

    // disable the USB if the app is not in debug mode
#ifndef DEBUG
    debugPrintln("The USB is going to be disabled now.");
    sodaq_wdt_safe_delay(3000);
    SerialUSB.end();
    USBDevice.detach();
    USB->DEVICE.CTRLA.reg &= ~USB_CTRLA_ENABLE; // Disable USB
#endif
}

void loop()
{
    // Reset watchdog
    sodaq_wdt_reset();
    sodaq_wdt_flag = false;

    if (minuteFlag) {
#ifdef DEBUG
        setLedColor(BLUE);
        sodaq_wdt_safe_delay(1000);
#endif

        getGpsFix();

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
    LORA_STREAM.begin(LoRaBee.getDefaultBaudRate());
#ifdef DEBUG
    LoRaBee.setDiag(DEBUG_STREAM);
#endif

    // TODO enable sleeping

    uint8_t devAddr[4];
    uint8_t appSKey[16];
    uint8_t nwkSKey[16];

    bool allValid = convertAndCheckHexArray((uint8_t*)devAddr, params.getDevAddr(), sizeof(devAddr))
        && convertAndCheckHexArray((uint8_t*)appSKey, params.getAppSKey(), sizeof(appSKey))
        && convertAndCheckHexArray((uint8_t*)nwkSKey, params.getNwSKey(), sizeof(nwkSKey));

    if (!allValid) {
        consolePrintln("The parameters for LoRa are not valid. LoRa will not be enabled.");
        return false;
    }

    if (LoRaBee.initABP(LORA_STREAM, devAddr, appSKey, nwkSKey, false)) {
        consolePrintln("LoRa network init completed.");
    }
    else {
        consolePrintln("LoRa network init failed!");
        return false;
    }

    return true;
}

/**
 * Powers down all devices and puts the system to deep sleep.
 */
void systemSleep()
{
    setLedColor(NONE);
    // TODO disconnect pins?
    // TODO explicitly? setGpsActive(false);

    sodaq_wdt_disable();

    // do not go to sleep if DEBUG is enabled, to keep USB connected
#ifndef DEBUG
    noInterrupts();
    if (!(sodaq_wdt_flag && minuteFlag)) {
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

void setNow(uint32_t now)
{
    debugPrint("Setting RTC to ");
    debugPrintln(now);

    rtc.setEpoch(now);
    isRtcInitialized = true;
}

/**
 * Shows and handles the boot up commands.
 */
void handleBootUpCommands()
{
    params.read();

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
        timer.every(5 * 24 * 60 * 60, runLoraModuleSleepExtendEvent); // every 5 days
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
        // TODO get fix and report
    }
}

/**
 * Runs the alternative fix event sequence (only if it set and is within the set time).
 */
void runAlternativeFixEvent(uint32_t now)
{
    if (isAlternativeFixEventApplicable()) {
        debugPrintln("Alternative fix event started.");
        // TODO get fix and report
    }
}

/**
 * Wakes up the lora module to put it back to sleep, i.e. extends the sleep period
*/
void runLoraModuleSleepExtendEvent(uint32_t now)
{
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
    // note: db_printf gets enabled/disabled according to the "DEBUG" define (ublox.cpp)
    ublox.db_printf("%4.4d-%2.2d-%2.2d %2.2d:%2.2d:%2.2d.%d valid=%2.2x lat=%d lon=%d sats=%d fixType=%2.2x\r\n",
        NavPvt->year, NavPvt->month, NavPvt->day,
        NavPvt->hour, NavPvt->minute, NavPvt->seconds, NavPvt->nano, NavPvt->valid,
        NavPvt->lat, NavPvt->lon, NavPvt->numSV, NavPvt->fixType);

    // check that the fix is OK and that it is a 3d fix or GNSS + dead reckoning combined
    if (((NavPvt->flags & GPS_FIX_FLAGS) == GPS_FIX_FLAGS) && ((NavPvt->fixType == 3) || (NavPvt->fixType == 4))) {
        debugPrintln("new point!");


    }

    // sync the RTC time
    if ((NavPvt->valid & GPS_TIME_VALIDITY) == GPS_TIME_VALIDITY) {
        uint32_t epoch = time.mktime(NavPvt->year, NavPvt->month, NavPvt->day, NavPvt->hour, NavPvt->minute, NavPvt->seconds);

        // check if there is an actual offset before setting the RTC
        if (abs(rtc.getEpoch() - epoch) > MAX_RTC_EPOCH_OFFSET) {
            setNow(epoch);
        }
    }
}

/**
 *
 */
bool getGpsFix()
{
    setGpsActive(true);

    uint32_t startTime = getNow();
    while (getNow() - startTime <= params.getGpsFixTimeout())
    {
        uint16_t bytes = ublox.available();

        if (bytes) {
            ublox.GetPeriodic(bytes); // calls the delegate method for passing results

            // TODO check if the last point in the list has changed and send through lora if applicable

            return true;
        }
    }

    setGpsActive(false);
    return false;
}

/**
 * Turns the GPS on or off.
 */
void setGpsActive(bool on)
{
    if (on) {
        pinMode(GPS_ENABLE, OUTPUT);
        pinMode(GPS_TIMEPULSE, INPUT);

        Wire.begin();

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
        Wire.end();
    }
}

/**
 * Turns the LoRa module on or off (wake up or sleep)
 */
void setLoraActive(bool on)
{
    if (on) {
        // TODO wake up module
    }
    else {
        // TODO put module to sleep
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
