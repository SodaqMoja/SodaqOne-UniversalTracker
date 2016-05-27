#include "ReportDataRecord.h"

#define fieldPrintWithSeparator(streamVar, separatorVar, val) { streamVar->print(val, DEC); streamVar->print(separatorVar); }

// setup the field sizes
static const uint8_t ReportDataFieldSizes[] = { REPORT_DATA_FIELD_SIZES };

bool ReportDataRecord::isValid() const
{
    return (getTimestamp() != 0xFFFFFFFF);
}

void ReportDataRecord::printRecordLn(Stream* stream, const char* separator) const
{
    if (stream) {
        fieldPrintWithSeparator(stream, separator, getTimestamp());
        fieldPrintWithSeparator(stream, separator, getBatteryVoltage());
        fieldPrintWithSeparator(stream, separator, getBoardTemperature());
        fieldPrintWithSeparator(stream, separator, getLat());
        fieldPrintWithSeparator(stream, separator, getLong());
        fieldPrintWithSeparator(stream, separator, getAltitude());
        fieldPrintWithSeparator(stream, separator, getSpeed());
        fieldPrintWithSeparator(stream, separator, getCourse());
        fieldPrintWithSeparator(stream, separator, getSatelliteCount());
        fieldPrintWithSeparator(stream, "\n", getTimeToFix());
    }
}

void ReportDataRecord::init()
{
    // zero everything
    for (uint16_t i = 0; i < getSize(); i++) {
        buffer[i] = 0;
    }
    
    // set default value for Timestamp (used for validity check)
    setTimestamp(0xFFFFFFFF);
}

uint8_t ReportDataRecord::getFieldSize(uint8_t fieldIndex) const
{
    if (fieldIndex > getFieldCount()) {
        // TODO sanity check fail
        return 0;
    }

    return ReportDataFieldSizes[fieldIndex];
}
