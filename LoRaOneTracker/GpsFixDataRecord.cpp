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
