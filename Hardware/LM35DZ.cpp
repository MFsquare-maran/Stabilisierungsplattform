#include "LM35DZ.h"

LM35DZ::LM35DZ(PinName pin): _analogIn(pin)
{
    
}

float LM35DZ::read_temperature() {
    float voltage = _analogIn.read() * 3.3f; // Annahme: Vref = 3.3 V
    return voltage * 100.0f; // 10 mV/°C
}
