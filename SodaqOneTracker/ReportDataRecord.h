#ifndef REPORTDATARECORD_H_
#define REPORTDATARECORD_H_

#include <Arduino.h>
#include <stdint.h>
#include "DataRecord.h"
#include "MacroSum.h"
#include "MacroCount.h"

//    uint32_t Timestamp;
//    uint8_t BatteryVoltage;
//    int8_t BoardTemperature;
//    uint32_t Lat;
//    uint32_t Long;
//    uint16_t Altitude;
//    uint16_t Speed;
//    uint8_t Course;
//    uint8_t SatelliteCount;
//    uint8_t TimeToFix;

#define REPORT_DATA_RECORD_HEADER "Timestamp, BatteryVoltage, BoardTemperature, Lat, Long, Altitude, Speed, Course, SatelliteCount, TimeToFix"

#define REPORT_DATA_FIELD_SIZES sizeof(uint32_t), \
                                sizeof(uint8_t), \
                                sizeof(int8_t), \
                                sizeof(uint32_t), \
                                sizeof(uint32_t), \
                                sizeof(uint16_t), \
                                sizeof(uint16_t), \
                                sizeof(uint8_t), \
                                sizeof(uint8_t), \
                                sizeof(uint8_t)

#define REPORT_DATA_BUFFER_SIZE (SUM(REPORT_DATA_FIELD_SIZES))
#define REPORT_DATA_FIELD_COUNT (COUNT(REPORT_DATA_FIELD_SIZES))

class ReportDataRecord: public DataRecord
{
public:
    ReportDataRecord() { }
    ~ReportDataRecord() { }

    void init();
    bool isValid() const;
    uint16_t getSize() const { return REPORT_DATA_BUFFER_SIZE; }
    uint8_t getFieldCount() const { return REPORT_DATA_FIELD_COUNT; }
    uint8_t* getBuffer() const { return (uint8_t*)buffer; }

    void printHeaderLn(Stream* stream) const { if (stream) { stream->println(REPORT_DATA_RECORD_HEADER); } }
    void printRecordLn(Stream* stream, const char* separator = DATA_RECORD_SEPARATOR) const;

    // field getter/setters
    uint32_t getTimestamp() const { return getFieldValue<uint32_t>(Timestamp); }
    void setTimestamp(uint32_t value) const { setFieldValue(Timestamp, value); }

    uint8_t getBatteryVoltage() const { return getFieldValue<uint8_t>(BatteryVoltage); }
    void setBatteryVoltage(uint8_t value) const { setFieldValue(BatteryVoltage, value); }

    int8_t getBoardTemperature() const { return getFieldValue<int8_t>(BoardTemperature); }
    void setBoardTemperature(int8_t value) const { setFieldValue(BoardTemperature, value); }

    uint32_t getLat() const { return getFieldValue<uint32_t>(Lat); }
    void setLat(uint32_t value) const { setFieldValue(Lat, value); }

    uint32_t getLong() const { return getFieldValue<uint32_t>(Long); }
    void setLong(uint32_t value) const { setFieldValue(Long, value); }

    uint16_t getAltitude() const { return getFieldValue<uint16_t>(Altitude); }
    void setAltitude(uint16_t value) const { setFieldValue(Altitude, value); }

    uint16_t getSpeed() const { return getFieldValue<uint16_t>(Speed); }
    void setSpeed(uint16_t value) const { setFieldValue(Speed, value); }

    uint8_t getCourse() const { return getFieldValue<uint8_t>(Course); }
    void setCourse(uint8_t value) const { setFieldValue(Course, value); }

    uint8_t getSatelliteCount() const { return getFieldValue<uint8_t>(SatelliteCount); }
    void setSatelliteCount(uint8_t value) const { setFieldValue(SatelliteCount, value); }

    uint8_t getTimeToFix() const { return getFieldValue<uint8_t>(TimeToFix); }
    void setTimeToFix(uint8_t value) const { setFieldValue(TimeToFix, value); }

protected:
    uint8_t getFieldSize(uint8_t fieldIndex) const;
private:
    uint8_t buffer[REPORT_DATA_BUFFER_SIZE];

    enum Fields { Timestamp, BatteryVoltage, BoardTemperature, Lat, Long, Altitude, Speed, Course, SatelliteCount, TimeToFix };
};

#endif /* REPORTDATARECORD_H_ */

