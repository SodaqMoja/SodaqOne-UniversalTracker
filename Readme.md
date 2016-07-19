# SODAQ ONE Tracker specs

Note: to be able to compile this application you need to add the right board file to your Arduino IDE.

Go to File, Preferences and set the following URL for the additional board files:

http://downloads.sodaq.net/package_sodaq_index.json

##  Configuration Menu

After compiling the sourcecode and loading it onto the board you will be able to configure the board through a menu.

Just open the Arduino Serial Monitor (at 9600 baud) and you will get this menu:
```
** SodaqOne Universal Tracker - 1.8 **
LoRa HWEUI: 571A82F8BADA0000
 -> CPU reset by Power On Reset [1]

Commands:
  Reset DevAddr / DevEUI to the Hardware EUI (EUI):
  Commit Settings (CS):

Settings:

  Fix Interval (min)         (fi=): 15
  Alt. Fix Interval (min)    (afi=): 0
  Alt. Fix From (HH)         (affh=): 0
  Alt. Fix From (MM)         (affm=): 0
  Alt. Fix To (HH)           (afth=): 0
  Alt. Fix To (MM)           (aftm=): 0
  GPS Fix Timeout (sec)      (gft=): 120
  OTAA Mode (OFF=0 / ON=1)   (otaa=): 0
  DevAddr / DevEUI           (dev=): 0000000000000000
  AppSKey / AppEUI           (app=): 00000000000000000000000000000000
  NWSKey / AppKey            (key=): 00000000000000000000000000000000
  Num Coords to Upload       (num=): 1
  Repeat Count               (rep=): 0
  Status LED (OFF=0 / ON=1)  (led=): 0

Enter command:
 ```




Entering commands is just a matter of typing the command as given in brackets with the right value. For example:

fi=5

Will set the time between the GPS fixes to 5 minutes.

The following parameters must be configurable through the menu:

**Configuration Parameters**

| Description | Length |
| --- | --- |
| Fix Interval  default | Minutes |
| Fix Interval alternate | Minutes |
| From | time HH:MM |
| To | time HH:MM |
| GPS fix timeout | seconds |
| DEVADDR |   |
| APPSKEY |   |
| NWSKEY |   |
| Number of coordinates to upload (1-4) |   |
| Repeat count (default 0) |   |

The LoRa communication must only start when the keys are not 0 (which is the default)

#### Sleep
After the startup the device by default willt be in deep sleep mode. In sleep it uses less than 50 uA. Using the RTC Timers it will wake up at the set intervals.

#### Timers and Watchdog
The application is based on the RTCTimer library. At startup the applicationtriesobtain a GPS fix until timeout. If no fix can be obtained initially the location will be set to 0,0. Once the first fix is obtained the RTC will be set and that fix location will be kept until the next proper fix.

There is a system watchdog running in case the application hangs it will be restarted by the watchdog. (See library Sodaq\_wdt)

#### GPS fixes
As seen in the configuration menu we allow for two different schedules for GPS fixes based on UTC time. The default could for instance be that we want a GPS fix every 30 minutes during the night, but during the day we want a fix every 5 minutes. In that case we configure the default to be 30 minutes, but the optional fix timer to be 5 minutes from 06:00 UTC to 18:00 UTC.

The RTC library allows for two of such timers. In case the second Fix interval is set to 0 the second timer is simply ignored.

After the GPS fix is obtained (or the timeout reached) a LoRa packet will be sent.

For redundance we could configure a repeat count. The value of the repeat count tells us to send the Lora frame an additional number of times (default 0) for redundancy.

The Lora frame should contain the below data. The minimum frame size is 21 bytes, the maximum frame size 51 bytes, depending on the number of coordinates we have configured to be sent.





#### LoRa Frame content

| Description | Length |
| --- | --- |
| Epoch Timestamp | long (4) |
| Battery voltage (between 3 and 4.5 V) | uint8 (1) |
| Board Temperature (degrees celcius) | int8 (1) |
| Lat | long (4) |
| Long | long (4) |
| Altitude (MSL in meters  below sea level is set to FFFF) | uint16 (2) |
| Speed (SOG \* 100 km/h) | uint16 (2) |
| Course (COG) | uint8 (1) |
| Number of satellites | uint8 (1) |
| Time to fix (seconds, FF = no fix, in that case the above position is the last known) | uint8 (1) |
| Plus 0 - 3 of the following 10 bytes: |   |
| Previous fix (seconds ago, FFFF means longer than) | uint16 (2) |
| Lat | long (4) |
| Long | long(4) |



#### Remote (over the air) re-configuration

You can send the following configuration parameters back to the device (as part as the LoRaWAN class A communication protocol)



| Description | Length |
| --- | --- |
| Fix Interval  default | uint16 (2) |
| Fix Interval alternate | uint16 (2) |
| From (EPOCH) | long (4) |
| To (EPOCH) | long (4) |
| GPS fix timeout in seconds | uint16 (2) |


## License

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

