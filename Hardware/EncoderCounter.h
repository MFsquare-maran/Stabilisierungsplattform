#ifndef ENCODERCOUNTER_H
#define ENCODERCOUNTER_H

#include "mbed.h"
#include <cstdint>
#include "env.h"



class EncoderCounter {
public:
    EncoderCounter(PinName a, PinName b,PinName I,float gear, float ts,int direction);
    ~EncoderCounter();

    void update();  
    void reset(float position = 0.0);

    float get_rad_s();
    float get_rad();
    float get_revolutions_s();
    float get_revolutions();

    float get_revolutions_min();
    float get_mm_s();
    float get_mm();
    
    


private:
    TIM_TypeDef* TIM;
    TIM_HandleTypeDef TIM_Handle;
    Mutex mutex;

    DigitalIn I_Counter;

    float ts;
    float gear;
    int direction;

    int32_t delta = 0.0;
    int32_t counter_val = 0.0;
    int32_t counter_val_old = 0.0;
    float rad = 0.0;
    float rad_s = 0.0;
    float rotation_s = 0.0f;
    float rotation = 0.0f;
    

  

 
    

    
    
    
};

#endif // ENCODERCOUNTER_H
