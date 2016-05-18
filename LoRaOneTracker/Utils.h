/*
* Copyright (c) 2015 SODAQ. All rights reserved.
*
* This file is part of MicrochipLoRaWAN.
*
* MicrochipLoRaWAN is free software: you can redistribute it and/or modify
* it under the terms of the GNU Lesser General Public License as
* published by the Free Software Foundation, either version 3 of
* the License, or(at your option) any later version.
*
* MicrochipLoRaWAN is distributed in the hope that it will be useful,
* but WITHOUT ANY WARRANTY; without even the implied warranty of
* MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE. See the
* GNU Lesser General Public License for more details.
*
* You should have received a copy of the GNU Lesser General Public
* License along with MicrochipLoRaWAN.  If not, see
* <http://www.gnu.org/licenses/>.
*/

#ifndef _UTILS_h
#define _UTILS_h

#define BOOL_TO_ONOFF(b) (b ? "on" : "off")
#define NIBBLE_TO_HEX_CHAR(i) ((i <= 9) ? ('0' + i) : ('A' - 10 + i))
#define HIGH_NIBBLE(i) ((i >> 4) & 0x0F)
#define LOW_NIBBLE(i) (i & 0x0F)

#define HEX_CHAR_TO_NIBBLE(c) ((c >= 'A') ? (c - 'A' + 0x0A) : (c - '0'))
#define HEX_PAIR_TO_BYTE(h, l) ((HEX_CHAR_TO_NIBBLE(h) << 4) + HEX_CHAR_TO_NIBBLE(l))

#define ARRAY_SIZE(a) (sizeof(a)/sizeof(a[0]))

#endif
