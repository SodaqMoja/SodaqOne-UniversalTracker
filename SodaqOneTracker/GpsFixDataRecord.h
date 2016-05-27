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
                                 sizeof(uint32_t), \
                                 sizeof(uint32_t)

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

    uint32_t getLat() const { return getFieldValue<uint32_t>(Lat); }
    void setLat(uint32_t value) const { setFieldValue(Lat, value); }

    uint32_t getLong() const { return getFieldValue<uint32_t>(Long); }
    void setLong(uint32_t value) const { setFieldValue(Long, value); }

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

