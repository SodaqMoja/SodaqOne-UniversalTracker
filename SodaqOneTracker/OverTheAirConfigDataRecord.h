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

