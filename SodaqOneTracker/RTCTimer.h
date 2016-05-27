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

#include <stdint.h>

#ifndef RTCTIMER_H_
#define RTCTIMER_H_

#define MAX_NUMBER_OF_RTCEVENTS (10)

class RTCTimer;
class RTCEvent
{
  friend class RTCTimer;
public:
  enum RTCEventType {
    RTCEvent_None = 0,
    RTCEvent_Every,
  };
  //RTCEvent();

  bool update(uint32_t now);

protected:
  enum RTCEventType _eventType;
  uint32_t _lastEventTime;
  uint32_t _period;
  int _count;
  int _repeatCount;
  void (*_callback)(uint32_t now);
};

class RTCTimer
{
public:
  //RTCTimer();

  int8_t every(uint32_t period, void (*callback)(uint32_t ts), int repeatCount=-1);

  void resetAll(uint32_t now);
  void clearAllEvents();
  void setNowCallback(uint32_t (*now)()) { _now = now; }
  void adjust(uint32_t old, uint32_t now);
  void update();
  void update(uint32_t now);
  void allowMultipleEvents(bool allow=true) { _noMultipleEvents = !allow; }

protected:
  int8_t        findFreeEventIndex();
  uint32_t      (*_now)();
  RTCEvent      _events[MAX_NUMBER_OF_RTCEVENTS];
  bool          _noMultipleEvents;
};

#endif /* RTCTIMER_H_ */
