#include "U_Battery.h"

U_Battery::U_Battery(PinName pin,float ts)
    : adc(pin), voltage(0)
{
    Filter_Uin.setPeriod(ts);
    Filter_Uin.setFrequency(10);
    
}

float U_Battery::linearInterpolation(float input, const float (*table)[2], uint8_t tableSize)
{
    for (size_t i = 0; i < tableSize - 1; ++i)
    {
        if (input >= table[i][0] && input <= table[i + 1][0])
        {
            float x0 = table[i][0];
            float y0 = table[i][1];
            float x1 = table[i + 1][0];
            float y1 = table[i + 1][1];
            // Lineare Interpolationsformel
            return y0 + (y1 - y0) * (input - x0) / (x1 - x0);
        }
    }
    // Falls der Eingabewert außerhalb des Bereichs der Tabelle liegt, gib den letzten Wert zurück
    return table[tableSize - 1][1];
}

float U_Battery::get_input_voltage() 
{
    return Filter_Uin.filter(linearInterpolation(adc.read(),Uin_tablePtr,55));
}
