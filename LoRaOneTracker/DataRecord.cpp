//
// DataRecord.cpp
// Alex Tsamakos
//

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
