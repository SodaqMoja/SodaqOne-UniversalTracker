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

#ifndef OVERTHEAIRCONFIGDATARECORD_H_
#define OVERTHEAIRCONFIGDATARECORD_H_

#include <Arduino.h>
#include <stdint.h>
#include "DataRecord.h"
#include "MacroSum.h"
#include "MacroCount.h"

//    uint16_t DefaultFixInterval;
//    uint16_t AlternativeFixInterval;
//    uint32_t AlternativeFixFrom;
//    uint32_t AlternativeFixTo;
//    uint16_t GpsFixTimeout;

#define OVER_THE_AIR_CONFIG_DATA_RECORD_HEADER "DefaultFixInterval, AlternativeFixInterval, AlternativeFixFrom, AlternativeFixTo, GpsFixTimeout"

#define OVER_THE_AIR_CONFIG_DATA_FIELD_SIZES sizeof(uint16_t), \
                                             sizeof(uint16_t), \
                                             sizeof(uint32_t), \
                                             sizeof(uint32_t), \
                                             sizeof(uint16_t)

#define OVER_THE_AIR_CONFIG_DATA_BUFFER_SIZE (SUM(OVER_THE_AIR_CONFIG_DATA_FIELD_SIZES))
#define OVER_THE_AIR_CONFIG_DATA_FIELD_COUNT (COUNT(OVER_THE_AIR_CONFIG_DATA_FIELD_SIZES))

class OverTheAirConfigDataRecord: public DataRecord
{
public:
    OverTheAirConfigDataRecord() { }
    ~OverTheAirConfigDataRecord() { }

    void init();
    bool isValid() const;
    uint16_t getSize() const { return OVER_THE_AIR_CONFIG_DATA_BUFFER_SIZE; }
    uint8_t getFieldCount() const { return OVER_THE_AIR_CONFIG_DATA_FIELD_COUNT; }
    uint8_t* getBuffer() const { return (uint8_t*)buffer; }

    void printHeaderLn(Stream* stream) const { if (stream) { stream->println(OVER_THE_AIR_CONFIG_DATA_RECORD_HEADER); } }
    void printRecordLn(Stream* stream, const char* separator = DATA_RECORD_SEPARATOR) const;

    // field getter/setters
    uint16_t getDefaultFixInterval() const { return getFieldValue<uint16_t>(DefaultFixInterval); }
    void setDefaultFixInterval(uint16_t value) const { setFieldValue(DefaultFixInterval, value); }

    uint16_t getAlternativeFixInterval() const { return getFieldValue<uint16_t>(AlternativeFixInterval); }
    void setAlternativeFixInterval(uint16_t value) const { setFieldValue(AlternativeFixInterval, value); }

    uint32_t getAlternativeFixFrom() const { return getFieldValue<uint32_t>(AlternativeFixFrom); }
    void setAlternativeFixFrom(uint32_t value) const { setFieldValue(AlternativeFixFrom, value); }

    uint32_t getAlternativeFixTo() const { return getFieldValue<uint32_t>(AlternativeFixTo); }
    void setAlternativeFixTo(uint32_t value) const { setFieldValue(AlternativeFixTo, value); }

    uint16_t getGpsFixTimeout() const { return getFieldValue<uint16_t>(GpsFixTimeout); }
    void setGpsFixTimeout(uint16_t value) const { setFieldValue(GpsFixTimeout, value); }

protected:
    uint8_t getFieldSize(uint8_t fieldIndex) const;
private:
    uint8_t buffer[OVER_THE_AIR_CONFIG_DATA_BUFFER_SIZE];

    enum Fields { DefaultFixInterval, AlternativeFixInterval, AlternativeFixFrom, AlternativeFixTo, GpsFixTimeout };
};

#endif /* OVERTHEAIRCONFIGDATARECORD_H_ */

