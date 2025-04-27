#ifndef LM35DZ_H
#define LM35DZ_H

#include "mbed.h"

class LM35DZ {
public:
    /** Konstruktor
     *  @param pin Physikalischer Pin z. B. PC_0
     */
    LM35DZ(PinName pin);

    /** Temperatur in °C lesen
     *  @return Temperaturwert
     */
    float read_temperature();

private:
    AnalogIn _analogIn;
};

#endif // LM35DZ_H
