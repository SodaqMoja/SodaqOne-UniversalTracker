//
// DataRecord.h
// Alex Tsamakos
//

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

