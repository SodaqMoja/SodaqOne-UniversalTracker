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

#include "GpsFixDataRecord.h"

#define fieldPrintWithSeparator(streamVar, separatorVar, val) { streamVar->print(val, DEC); streamVar->print(separatorVar); }

// setup the field sizes
static const uint8_t GpsFixDataFieldSizes[] = { GPS_FIX_DATA_FIELD_SIZES };

bool GpsFixDataRecord::isValid() const
{
    return (getTimestamp() != 0xFFFFFFFF);
}

void GpsFixDataRecord::printRecordLn(Stream* stream, const char* separator) const
{
    if (stream) {
        fieldPrintWithSeparator(stream, separator, getPreviousFix());
        fieldPrintWithSeparator(stream, separator, getLat());
        fieldPrintWithSeparator(stream, separator, getLong());
        fieldPrintWithSeparator(stream, "\n", getTimestamp());
    }
}

void GpsFixDataRecord::init()
{
    // zero everything
    for (uint16_t i = 0; i < getSize(); i++) {
        buffer[i] = 0;
    }
    
    // set default value for Timestamp (used for validity check)
    setTimestamp(0xFFFFFFFF);
}

uint8_t GpsFixDataRecord::getFieldSize(uint8_t fieldIndex) const
{
    if (fieldIndex > getFieldCount()) {
        // TODO sanity check fail
        return 0;
    }

    return GpsFixDataFieldSizes[fieldIndex];
}

void GpsFixDataRecord::updatePreviousFixValue(uint32_t now)
{ 
    uint32_t val = now - getTimestamp(); 
    setPreviousFix(val > 0xFFFF ? 0xFFFF : (uint16_t)val); 
}
