#include "OverTheAirConfigDataRecord.h"

#define fieldPrintWithSeparator(streamVar, separatorVar, val) { streamVar->print(val, DEC); streamVar->print(separatorVar); }

// setup the field sizes
static const uint8_t OverTheAirConfigDataFieldSizes[] = { OVER_THE_AIR_CONFIG_DATA_FIELD_SIZES };
const uint8_t* OverTheAirConfigDataRecord::fieldSizes = OverTheAirConfigDataFieldSizes;

bool OverTheAirConfigDataRecord::isValid() const
{
    return (getDefaultFixInterval() != 0xFFFF);
}

void OverTheAirConfigDataRecord::printRecordLn(Stream* stream, const char* separator) const
{
    //    uint16_t DefaultFixInterval;
    //    uint16_t AlternativeFixInterval;
    //    uint32_t AlternativeFixFrom;
    //    uint32_t AlternativeFixTo;
    //    uint16_t GpsFixTimeout;

    if (stream) {
        fieldPrintWithSeparator(stream, separator, getDefaultFixInterval());
        fieldPrintWithSeparator(stream, separator, getAlternativeFixInterval());
        fieldPrintWithSeparator(stream, separator, getAlternativeFixFrom());
        fieldPrintWithSeparator(stream, separator, getAlternativeFixTo());
        fieldPrintWithSeparator(stream, "\n", getGpsFixTimeout());
    }
}

void OverTheAirConfigDataRecord::init()
{
    // zero everything
    for (uint16_t i = 0; i < getSize(); i++) {
        buffer[i] = 0;
    }
    
    // set default value for DefaultFixInterval (used for validity check)
    setDefaultFixInterval(0xFFFF);
}

uint8_t OverTheAirConfigDataRecord::getFieldSize(uint8_t fieldIndex) const
{
    if (fieldIndex > getFieldCount()) {
        // TODO sanity check fail
        return 0;
    }

    return fieldSizes[fieldIndex];
}
