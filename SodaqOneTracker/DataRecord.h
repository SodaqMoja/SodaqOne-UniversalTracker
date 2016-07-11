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

#ifndef DATARECORD_H_
#define DATARECORD_H_

#include <Arduino.h>

/**
 * Base class for standardizing data record definitions with fixed-size fields.
 * This makes an absolutely safe way of dealing with the records vs the unaligned structs.
 * To allow for portability it uses memcpy instead of casts for converting fields to and from the buffer.
 */

#define DATA_RECORD_SEPARATOR ", "

//
// Derived classes should:
// * implement all virtual methods (obv.)
// * (optional) create enum Fields { FieldName1 = 0, FieldName2, FieldName3, ... } for ease of use
// * (optional but strongly suggested) create getters/setters for all fields
//

// TODO: make fieldsizes needed only in one place for each derived class

class DataRecord
{
public:
    virtual void init();

    virtual bool isValid() const = 0;
    virtual uint16_t getSize() const = 0;
    virtual uint8_t getFieldCount() const = 0;

    virtual uint8_t* getBuffer() const = 0;

    virtual void printHeaderLn(Stream* stream) const = 0;
    virtual void printRecordLn(Stream* stream, const char* separator = DATA_RECORD_SEPARATOR) const = 0;

    void copyTo(uint8_t* buffer, size_t size) const;
    void copyFrom(const uint8_t* buffer, size_t size);

    uint8_t getFieldValue(uint8_t fieldIndex, uint8_t* buffer, size_t size) const;

    template <typename T>
    T getFieldValue(uint8_t fieldIndex) const
    {
        // implementation has to be here
        if (fieldIndex > getFieldCount() || getFieldSize(fieldIndex) > sizeof(T)) {
            // TODO debugPrintLn("getFieldValue(): sanity check failed!");
            return 0;
        }

        T result;
        uint16_t fieldStart = getFieldStart(fieldIndex);
        memcpy(&result, getBuffer() + fieldStart, sizeof(T));

        return result;
    }

    void setFieldValue(uint8_t fieldIndex, const uint8_t* buffer, uint8_t size) const;

    template <typename T>
    void setFieldValue(uint8_t fieldIndex, T value) const
    {
        // implementation has to be here
        if (fieldIndex > getFieldCount() || sizeof(T) > getFieldSize(fieldIndex)) {
            // TODO debugPrintLn("setFieldValue(): sanity check failed!");
            return;
        }

        uint16_t fieldStart = getFieldStart(fieldIndex);
        memcpy(getBuffer() + fieldStart, &value, getFieldSize(fieldIndex));
    }

protected:
    virtual uint8_t getFieldSize(uint8_t fieldIndex) const = 0;
    uint16_t getFieldStart(uint8_t fieldIndex) const;
private:
};

#endif /* DATARECORD_H_ */

