#ifndef DS1820_H
#define DS1820_H

#include "mbed.h"
#include <chrono>

class DS1820 {
public:
    DS1820(PinName pin);
    bool begin();
    float readTemperature();

private:
    DigitalInOut _dataPin;

    bool reset();
    void writeByte(uint8_t byte);
    uint8_t readByte();
    bool isConversionDone();

};

#endif // DS1820_H
