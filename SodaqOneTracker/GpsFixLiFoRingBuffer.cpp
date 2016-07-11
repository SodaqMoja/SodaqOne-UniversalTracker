/*
Copyright (c) 2016, SODAQ
All rights reserved.

Redistribution and use in source and binary forms, with or without
modification, are permitted provided that the following conditions are met:

1. Redistributions of source code must retain the above copyright notice,
this list of conditions and the following disclaimer.

2. Redistributions in binary form must reproduce the above copyright notice,
this list of conditions and the following disclaimer in the documentation
and/or other materials provided with the distribution.

3. Neither the name of the copyright holder nor the names of its contributors
may be used to endorse or promote products derived from this software without
specific prior written permission.

THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS"
AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO,
THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE
ARE DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT HOLDER OR CONTRIBUTORS BE
LIABLE FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR
CONSEQUENTIAL DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF
SUBSTITUTE GOODS OR SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS
INTERRUPTION) HOWEVER CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN
CONTRACT, STRICT LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE)
ARISING IN ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE
POSSIBILITY OF SUCH DAMAGE.
*/

#include "GpsFixLiFoRingBuffer.h"

static GpsFixDataRecord _data[GPS_FIX_LIFO_RING_BUFFER_SIZE];

static uint16_t _bottom;
static uint16_t _top;
static uint16_t _validItemCount;


void gpsFixLiFoRingBuffer_init(void)
{
    _validItemCount = 0;
    _bottom = 0;
    _top = 0;

    for (uint16_t i = 0; i < GPS_FIX_LIFO_RING_BUFFER_SIZE; i++) {
        _data[i].init();
    }
}

bool gpsFixLiFoRingBuffer_isEmpty(void)
{
    return (_validItemCount == 0);
}

void gpsFixLiFoRingBuffer_push(const GpsFixDataRecord* value)
{
    _top = (_top + 1) % GPS_FIX_LIFO_RING_BUFFER_SIZE;
    _data[_top] = *value;

    _validItemCount++;
        
    // check for roll over
    if (_validItemCount > GPS_FIX_LIFO_RING_BUFFER_SIZE) {
        _validItemCount = GPS_FIX_LIFO_RING_BUFFER_SIZE;
            
        _bottom = (_bottom + 1) % GPS_FIX_LIFO_RING_BUFFER_SIZE; // bump to next item
    }
}

#ifdef GPS_FIX_LIFO_RING_BUFFER_ENABLE_POP
bool gpsFixRingBuffer_pop(GpsFixDataRecord* returnValue)
{
    if (gpsFixLiFoRingBuffer_isEmpty()) {
        return false;
    }

    *returnValue = _data[_top];
    if (_top == 0) {
        _top = GPS_FIX_LIFO_RING_BUFFER_SIZE - 1;
    }
    else {
        _top--;
    }
    _validItemCount--;

    return true;
}
#endif

bool gpsFixLiFoRingBuffer_peek(uint16_t offset, GpsFixDataRecord* returnValue)
{
    if (gpsFixLiFoRingBuffer_isEmpty() || offset > _validItemCount) {
        return false;
    }

    *returnValue = _data[(_top - offset) % GPS_FIX_LIFO_RING_BUFFER_SIZE];
    return true;
}
