
// TODO flash config
// TODO configuration
// TODO config menu
// TODO config record (lora receive)
// TODO transmition record (lora send)
// TODO lora send+receive
// TODO GPS
// TODO last 4 fix points array

#include <Arduino.h>
#include "RTCTimer.h"
#include "RTCZero.h"
#include "Sodaq_RN2483.h"
#include "Sodaq_wdt.h"

#include "version.h"


#define DEBUG

#define PROJECT_NAME "Generic Tracker"
#define STARTUP_DELAY 5000
#define DEBUG_STREAM SerialUSB
#define CONSOLE_STREAM SerialUSB
#define LORA_STREAM Serial1

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

volatile bool minuteFlag;

static uint8_t lastResetCause;
static bool isLoraInitialized;
static bool isRtcInitialized;
static bool isDeviceInitialized;


void setup();
void loop();

uint32_t getNow();
void handleBootUpCommands();
void initRtc();
void rtcAlarmHandler();
void initRtcTimer();
void resetRtcTimerEvents();
void initSleep();
bool initLora();
bool syncRtc();
void systemSleep();
void runDefaultFixEvent(uint32_t now);
void runAlternativeFixEvent(uint32_t now);
void setLedColor(LedColor color);
void setGpsActive(bool on);
void setLoraActive(bool on);
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
    // TODO gps
    isRtcInitialized = syncRtc();
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
        setLedColor(GREEN);
        sodaq_wdt_safe_delay(2000);
#endif

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
 * Initializes the lora module. 
 * Returns true if successful.
*/
bool initLora()
{
    LORA_STREAM.begin(LoRaBee.getDefaultBaudRate());
#ifdef DEBUG
    LoRaBee.setDiag(DEBUG_STREAM);
#endif
    
    // TODO check params validity
    // if not valid
    // println("The keys are not valid. LoRa will not be enabled.");
    // return false;
    
    //if (LoRaBee.initABP(loraSerial, devAddr, appSKey, nwkSKey, false)) {
    //    println("LoRa network init finished.");
    //    
    //    return true;
    //}
    //else {
    //    println("LoRa network init failed!");
    //    
    //    return false;
    //}

    return false;
}

/**
 * Syncs the RTC time with the GPS time.
 * Returns true if successful.
 */
bool syncRtc()
{
    // TODO
    return false;
}

/**
 * Powers down all devices and puts the system to deep sleep.
 */
void systemSleep()
{
    // TODO turn off devices
    setLedColor(NONE);
    // put lora module to sleep
    // disconnect pins
    // actively disable GPS

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

/**
 * Shows and handles the boot up commands.
 */
void handleBootUpCommands()
{
    // TODO
    //DFlashSegment& configSegment = hapStorage.getConfigSegment();

    //params.read(configSegment);

    //uint8_t count = 0;
    //do {
    //    startupCommands(CONSOLE_STREAM);
    //    count++;
    //} while ((!params.checkConfig()) || (count < MIN_STARTUP_PROMPTS));

    //params.dump();
    //params.commit(configSegment);

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
    // TODO clear all events

    // Schedule the default fix event
    // TODO timer.every(params.getDefaultFixInterval() * 60, runDefaultFixEvent);

    // TODO check if it should be scheduled at all
    // Schedule the alternative fix event
    // TODO timer.every(params.getAlternativeFixInterval() * 60, runAlternativeFixEvent);
}

/**
 * Runs the default fix event sequence (only if it is within the set time).
 */
void runDefaultFixEvent(uint32_t now)
{
    // TODO check check applicability
    // TODO get fix and report
}

/**
 * Runs the alternative fix event sequence (only if it set and is within the set time).
 */
void runAlternativeFixEvent(uint32_t now)
{
    // TODO check applicability
    // TODO get fix and report
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
 * Turns the GPS on or off.
 */
void setGpsActive(bool on)
{
    if (on) {
        // TODO pinMode
        // TODO turn on
    }
    else {
        // TODO turn off
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
