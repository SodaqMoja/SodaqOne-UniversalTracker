
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

//#include "version.h"

#define PROJECT_NAME "Generic Tracker"

RTCZero rtc;
RTCTimer timer;

static uint8_t lastResetCause;
static bool isLoraInitialized;
static bool isRtcInitialized;
static bool isDeviceInitialized;

void setup();
void loop();

uint32_t getNow();
void handleBootUpCommands();
void initLeds();
void initRtc();
void initRtcTimer();
void resetRtcTimerEvents();
void initSleep();
bool syncRtc();
void systemSleep();
void runDefaultFixEvent(uint32_t now);
void runAlternativeFixEvent(uint32_t now);

static void printCpuResetCause(Stream& stream);
static void printBootUpMessage(Stream& stream);

void setup() 
{
    lastResetCause = PM->RCAUSE.reg;
    sodaq_wdt_disable();

    SerialUSB.begin(115200);
    initLeds();
    initRtc(); 
    handleBootUpCommands();
        
    // TODO init GPS
    // TODO set RTC from GPS or timeout
    // TODO init RN2483
    initRtcTimer();

    initSleep();

    isDeviceInitialized = true;

    //debugPrintLn("** Boot-up completed successfully!");

    sodaq_wdt_enable();
}

void loop() 
{
    if (sodaq_wdt_flag) {

        // Reset watchdog
        sodaq_wdt_reset();
        sodaq_wdt_flag = false;

        // Update the scheduler
        timer.update();
    }

    // Force diag off before going to sleep
    //stopDiagStream(&DIAG_STREAM);

    systemSleep();

    //if (params.getEnableDiag()) {
    //    startDiagStream(&DIAG_STREAM, PC_BAUD);
    //}
}

void initSleep()
{
    // Set the sleep mode
    SCB->SCR |= SCB_SCR_SLEEPDEEP_Msk;
}

/**
 * Syncs the RTC time with the GPS time.
 */
bool syncRtc()
{
    return false;
}

/**
 * Powers down all devices and puts the system to deep sleep.
 */
void systemSleep()
{
    // TODO turn off devices

    bool did_usb_switch_off = false;
    if (USB->DEVICE.FSMSTATUS.bit.FSMSTATE == USB_FSMSTATUS_FSMSTATE_SUSPEND_Val) {
        // Disable USB
        USBDevice.detach();
        did_usb_switch_off = true;
    }

    // In which cases do we NOT want to sleep?
    // * USB is ON
    if (!(USB->DEVICE.FSMSTATUS.bit.FSMSTATE == USB_FSMSTATUS_FSMSTATE_ON_Val)) {
        // Don't sleep if the timer_flag is set
        noInterrupts();
        if (!sodaq_wdt_flag) {
            interrupts();
            // SAMD sleep
            __WFI();
        }
        interrupts();
    }

    if (did_usb_switch_off) {
        // Enable USB and wait for resume if attached
        USBDevice.attach();
        USB->DEVICE.CTRLB.bit.UPRSM = 0x01u;
        while (USB->DEVICE.CTRLB.bit.UPRSM) {
            // Wait for ...
        }
    }
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
 * Initializes the RGB led.
 */
void initLeds()
{
    // TODO
}

/**
 * Initializes the RTC.
 */
void initRtc()
{
    rtc.begin();

    // This sets it to 2000-01-01
    rtc.setEpoch(0);
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
    // TODO stream.println("** " PROJECT_NAME " - " VERSION " **");

    stream.print(" -> ");
    printCpuResetCause(stream);

    stream.println();
}
