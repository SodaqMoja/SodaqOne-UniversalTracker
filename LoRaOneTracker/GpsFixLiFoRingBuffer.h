/*
 * Ring buffer used as gps fix stack.
*/

#ifndef GPSFIXLIFORINGBUFFER_H_
#define GPSFIXLIFORINGBUFFER_H_

#include <stdbool.h>
#include <stdint.h>
#include "GpsFixDataRecord.h"

#define GPS_FIX_LIFO_RING_BUFFER_SIZE 4
//#define GPS_FIX_LIFO_RING_BUFFER_ENABLE_POP

void gpsFixLiFoRingBuffer_init(void);
bool gpsFixLiFoRingBuffer_isEmpty(void);
void gpsFixLiFoRingBuffer_push(const GpsFixDataRecord* value);
#ifdef GPS_FIX_LIFO_RING_BUFFER_ENABLE_POP
bool gpsFixRingBuffer_pop(GpsFixDataRecord* returnValue);
#endif
bool gpsFixLiFoRingBuffer_peek(uint16_t offset, GpsFixDataRecord* returnValue);

#endif /* GPSFIXRINGBUFFER_H_ */
