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
