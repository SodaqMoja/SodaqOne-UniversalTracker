/*
 * Copyright (c) 2014 Kees Bakker.  All rights reserved.
 *
 * This file is part of RTCTimer.
 *
 * RTCTimer is free software: you can redistribute it and/or
 * modify it under the terms of the GNU Lesser General Public License
 * as published by the Free Software Foundation, either version 3 of
 * the License, or(at your option) any later version.
 *
 * RTCTimer is distributed in the hope that it will be useful,
 * but WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE. See the
 * GNU Lesser General Public License for more details.
 *
 * You should have received a copy of the GNU Lesser General Public
 * License along with RTCTimer.  If not, see
 * <http://www.gnu.org/licenses/>.
 */

/*
 * The RTCTimer can be used to implement a simple scheduler.
 * It was inspired by the Arduino Timer library by Simon Monk.
 */

#include "RTCTimer.h"
//#include "Diag.h"

#if 0
RTCEvent::RTCEvent()
{
  _eventType = RTCEvent_None;
  _lastEventTime = 0;
  _period = 0;
  _callback = 0;
  _count = 0;
  _repeatCount = 0;
}

RTCTimer::RTCTimer()
{
}
#endif

bool RTCEvent::update(uint32_t now)
{
  bool doneEvent = false;
  if ((int32_t)(now - (_lastEventTime + _period)) >= 0) {
    //DIAGPRINT(F("RTCEvent::update ")); DIAGPRINTLN(_lastEventTime);
    if (_lastEventTime == 0) {
      // In case this wasn't initialized properly
      _lastEventTime = now;
    } else {
      while ((int32_t)(now - (_lastEventTime + _period)) >= 0) {
        // Skip all the events that are in the past
        _lastEventTime += _period;
      }
    }
    switch (_eventType) {
    case RTCEvent_Every:
      (*_callback)(now);
      break;
    default:
      break;
    }
    if (_repeatCount > 0 && ++_count >= _repeatCount) {
      // Done. Free the event.
      _eventType = RTCEvent_None;
    }
    doneEvent = true;
  }
  return doneEvent;
}

int8_t RTCTimer::every(uint32_t period, void (*callback)(uint32_t now),
    int repeatCount)
{
  int8_t i = findFreeEventIndex();
  if (i == -1)
    return -1;

  _events[i]._eventType = RTCEvent::RTCEvent_Every;
  _events[i]._period = period;
  _events[i]._repeatCount = repeatCount;
  _events[i]._callback = callback;
  _events[i]._lastEventTime = _now ? _now() : 0;
  _events[i]._count = 0;

  return i;
}

/*
 * Reset the "last event" time of all events to the new value.
 *
 * This function should be called once (e.g. in setup() after creating
 * all the ...
 */
void RTCTimer::resetAll(uint32_t now)
{
  for (uint8_t i = 0; i < sizeof(_events) / sizeof(_events[0]); ++i) {
    if (_events[i]._eventType != RTCEvent::RTCEvent_None) {
      _events[i]._lastEventTime = now;
    }
  }
}

void RTCTimer::clearAllEvents()
{
  for (uint8_t i = 0; i < sizeof(_events) / sizeof(_events[0]); ++i) {
    _events[i]._eventType = RTCEvent::RTCEvent_None;
  }
}

void RTCTimer::adjust(uint32_t old, uint32_t now)
{
  if (old == 0) {
    resetAll(now);
    return;
  }
  int32_t diff = now - old;
  for (uint8_t i = 0; i < sizeof(_events) / sizeof(_events[0]); ++i) {
    if (_events[i]._eventType != RTCEvent::RTCEvent_None) {
      if (_events[i]._lastEventTime == 0) {
        _events[i]._lastEventTime = now;
      } else {
        _events[i]._lastEventTime += diff;
      }
    }
  }
}

void RTCTimer::update()
{
  uint32_t now = _now ? _now() : 0;
  if (now) {
    update(now);
  }
}

void RTCTimer::update(uint32_t now)
{
  for (uint8_t i = 0; i < sizeof(_events) / sizeof(_events[0]); ++i) {
    if (_events[i]._eventType != RTCEvent::RTCEvent_None) {
      if (_events[i].update(now)) {
        if (_noMultipleEvents) {
          // Only one event action per update.
          break;
        }
      }
    }
  }
}

int8_t RTCTimer::findFreeEventIndex()
{
  for (uint8_t i = 0; i < sizeof(_events) / sizeof(_events[0]); ++i) {
    if (_events[i]._eventType == RTCEvent::RTCEvent_None) {
      return i;
    }
  }
  return -1;
}
