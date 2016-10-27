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

#ifndef GPSFIXDATARECORD_H_
#define GPSFIXDATARECORD_H_

#include <Arduino.h>
#include <stdint.h>
#include "DataRecord.h"
#include "MacroSum.h"
#include "MacroCount.h"

//    uint16_t PreviousFix;
//    uint32_t Lat;
//    uint32_t Long;

#define GPS_FIX_DATA_RECORD_HEADER "PreviousFix, Lat, Long"

#define GPS_FIX_DATA_FIELD_SIZES sizeof(uint16_t), \
                                 sizeof(int32_t), \
                                 sizeof(int32_t)

#define GPS_FIX_DATA_BUFFER_SIZE (SUM(GPS_FIX_DATA_FIELD_SIZES))
#define GPS_FIX_DATA_FIELD_COUNT (COUNT(GPS_FIX_DATA_FIELD_SIZES))

class GpsFixDataRecord: public DataRecord
{
public:
    GpsFixDataRecord() { }
    ~GpsFixDataRecord() { }

    void init();
    bool isValid() const;
    uint16_t getSize() const { return GPS_FIX_DATA_BUFFER_SIZE; }
    uint8_t getFieldCount() const { return GPS_FIX_DATA_FIELD_COUNT; }
    uint8_t* getBuffer() const { return (uint8_t*)buffer; }

    void printHeaderLn(Stream* stream) const { if (stream) { stream->println(GPS_FIX_DATA_RECORD_HEADER); } }
    void printRecordLn(Stream* stream, const char* separator = DATA_RECORD_SEPARATOR) const;

    // field getter/setters
    uint16_t getPreviousFix() const { return getFieldValue<uint16_t>(PreviousFix); }
    void setPreviousFix(uint16_t value) const { setFieldValue(PreviousFix, value); }

    int32_t getLat() const { return getFieldValue<int32_t>(Lat); }
    void setLat(int32_t value) const { setFieldValue(Lat, value); }

    int32_t getLong() const { return getFieldValue<int32_t>(Long); }
    void setLong(int32_t value) const { setFieldValue(Long, value); }

    uint32_t getTimestamp() const { return timestamp; }
    void setTimestamp(uint32_t value) { timestamp = value; }

    void updatePreviousFixValue(uint32_t now);

protected:
    uint8_t getFieldSize(uint8_t fieldIndex) const;
private:
    uint8_t buffer[GPS_FIX_DATA_BUFFER_SIZE];

    uint32_t timestamp;

    enum Fields { PreviousFix, Lat, Long };
};

#endif /* GPSFIXDATARECORD_H_ */

