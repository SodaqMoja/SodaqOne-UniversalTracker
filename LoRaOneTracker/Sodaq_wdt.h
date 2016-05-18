#ifndef SODAQ_WDT_H_
#define SODAQ_WDT_H_

#ifdef ARDUINO_ARCH_AVR

#include <avr/wdt.h>

#endif

// Approximate periods supported by both platforms
// The SAMD also supports 8ms and 16s.
enum wdt_period : uint8_t 
{
#ifdef ARDUINO_ARCH_AVR

  // See avr/wdt.h
  WDT_PERIOD_1DIV64  = WDTO_15MS,  // 15ms   = ~1/64s
  WDT_PERIOD_1DIV32  = WDTO_30MS,  // 30ms   = ~1/32s
  WDT_PERIOD_1DIV16  = WDTO_60MS,  // 60ms   = ~1/16s
  WDT_PERIOD_1DIV8   = WDTO_120MS, // 120ms  = ~1/8s
  WDT_PERIOD_1DIV4   = WDTO_250MS, // 250ms  = 1/4s
  WDT_PERIOD_1DIV2   = WDTO_500MS, // 500ms  = 1/2s
  WDT_PERIOD_1X      = WDTO_1S,    // 1000ms = 1s
  WDT_PERIOD_2X      = WDTO_2S,    // 2000ms = 2s
  WDT_PERIOD_4X      = WDTO_4S,    // 4000ms = 4s
  WDT_PERIOD_8X      = WDTO_8S     // 8000ms = 8s
  
#elif ARDUINO_ARCH_SAMD  

  // See Arduino15/packages/arduino/tools/CMSIS/4.0.0-atmel/Device/ATMEL/samd21/include/component/wdt.h
  // It is easier to use numeric values as there are two
  // sets of macros one for the reset WDT_CONFIG_WINDOW()
  // and the other for the early warning period WDT_EWCTRL_EWOFFSET().
  // Clock is running at 1024hz
  WDT_PERIOD_1DIV64 = 1,   // 16 cycles   = 1/64s
  WDT_PERIOD_1DIV32 = 2,   // 32 cycles   = 1/32s
  WDT_PERIOD_1DIV16 = 3,   // 64 cycles   = 1/16s
  WDT_PERIOD_1DIV8  = 4,   // 128 cycles  = 1/8s
  WDT_PERIOD_1DIV4  = 5,   // 256 cycles  = 1/4s
  WDT_PERIOD_1DIV2  = 6,   // 512 cycles  = 1/2s
  WDT_PERIOD_1X     = 7,   // 1024 cycles = 1s
  WDT_PERIOD_2X     = 8,   // 2048 cycles = 2s
  WDT_PERIOD_4X     = 9,   // 4096 cycles = 4s
  WDT_PERIOD_8X     = 10   // 8192 cycles = 8s
  
#endif
};

void sodaq_wdt_enable(wdt_period period = WDT_PERIOD_1X);

void sodaq_wdt_disable();

void sodaq_wdt_reset();

void sodaq_wdt_safe_delay(uint32_t ms);

extern volatile bool sodaq_wdt_flag;

#endif /* SODAQ_WDT_H_ */
