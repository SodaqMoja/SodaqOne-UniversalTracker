/*
 * Based on SQ_StartupCommands.h by Kees Bakker
 */

#ifndef BOOTMENU_H_
#define BOOTMENU_H_

#include <Arduino.h>

typedef void(*VoidCallbackMethodPtr)(void);

void showBootMenu(Stream& stream);
void setResetDevAddrOrEUItoHWEUICallback(VoidCallbackMethodPtr callback);

#endif
