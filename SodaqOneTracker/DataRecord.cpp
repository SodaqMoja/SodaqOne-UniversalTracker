/*
Copyright (c) 2016, Alex Tsamakos

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

#include "DataRecord.h"

void DataRecord::copyTo(uint8_t* buffer, size_t size) const
{
    if (size < getSize()) {
        // TODO failed sanity check!
        return;
    }

    memcpy(buffer, getBuffer(), size);
}

void DataRecord::copyFrom(const uint8_t* buffer, size_t size)
{
    if (size != getSize()) {
        // TODO failed sanity check!
        return;
    }

    memcpy(getBuffer(), buffer, size);
}

uint16_t DataRecord::getFieldStart(uint8_t fieldIndex) const
{
    uint16_t result = 0;
    for (uint8_t i = 0; i < fieldIndex; i++) {
        result += getFieldSize(i);
    }

    return result;
}

uint8_t DataRecord::getFieldValue(uint8_t fieldIndex, uint8_t* buffer, size_t size) const
{
    uint8_t fieldSize = getFieldSize(fieldIndex);

    if (fieldIndex > getFieldCount() || fieldSize > size) {
        // TODO debugPrintLn("getFieldValue(): sanity check failed!");
        return 0;
    }

    uint16_t fieldStart = getFieldStart(fieldIndex);
    memcpy(buffer, getBuffer() + fieldStart, fieldSize); // already checked buffer size >= getFieldSize(fieldIndex)

    return fieldSize;
}

void DataRecord::setFieldValue(uint8_t fieldIndex, const uint8_t* buffer, uint8_t size) const
{
    if (fieldIndex > getFieldCount() || size > getFieldSize(fieldIndex)) {
        // TODO debugPrintLn("setFieldValue(): sanity check failed!");
        return;
    }

    uint16_t fieldStart = getFieldStart(fieldIndex);
    memcpy(getBuffer() + fieldStart, buffer, getFieldSize(fieldIndex));
}
