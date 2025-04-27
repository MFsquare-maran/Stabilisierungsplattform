#ifndef U_BATTERY_H
#define U_BATTERY_H

#include "mbed.h"
#include "LowpassFilter.h"

class U_Battery {
public:
    U_Battery(PinName pin,float ts);
    
                       // ADC lesen und Spannung berechnen
    float get_input_voltage();   // Letzte berechnete Batteriespannung zurückgeben
    float linearInterpolation(float input, const float (*table)[2], uint8_t tableSize);

private:
    AnalogIn adc;
    LowpassFilter Filter_Uin;
    float voltage;


    const float Uin_table[55][2] = {
    {0.077692f, 2.031f},
    {0.094682f, 2.613f},
    {0.144010f, 4.26f},
    {0.154027f, 4.595f},
    {0.171911f, 5.191f},
    {0.180626f, 5.474},
    {0.199525f, 6.102f},
    {0.213302f, 6.553f},
    {0.226624f, 7.00},
    {0.244332f, 7.57},
    {0.259170f, 8.06f},
    {0.272096f, 8.47f},
    {0.290532f, 9.08f},
    {0.304348f, 9.52f},
    {0.322057f, 10.09f},
    {0.334605f, 10.49f},
    {0.350883f, 11.01f},
    {0.367110f, 11.53f},
    {0.381349f, 11.99f},
    {0.397944f, 12.51f},
    {0.413902f, 13.02f},
    {0.428593f, 13.49f},
    {0.444890f, 14.00f},
    {0.460713f, 14.51f},
    {0.478599f, 15.08f},
    {0.492045f, 15.5f},
    {0.508252f, 16.01f},
    {0.523749f, 16.50f},
    {0.540219f, 17.03f},
    {0.554143f, 17.48f},
    {0.571217f, 18.02f},
    {0.587597f, 18.55f},
    {0.602530f, 19.03f},
    {0.617046f, 19.50f},
    {0.633096f, 20.01f},
    {0.649570f, 20.55f},
    {0.663595f, 21.00f},
    {0.679820f, 21.52f},
    {0.694512f, 22.01f},
    {0.710668f, 22.53f},
    {0.726347f, 23.05f},
    {0.740137f, 23.50f},
    {0.757427f, 24.07f},
    {0.770778f, 24.51f},
    {0.785487f, 25.00f},
    {0.801248f, 25.52f},
    {0.817052f, 26.05f},
    {0.831411f, 26.52f},
    {0.846103f, 27.02f},
    {0.861772f, 27.54f},
    {0.877131f, 28.05f},
    {0.890849f, 28.51f},
    {0.906566f, 29.04f},
    {0.921703f, 29.55f},
    {0.934680f, 29.98f}};

    const float (*Uin_tablePtr)[2] = Uin_table;
    
};

#endif // U_BATTERY_H
