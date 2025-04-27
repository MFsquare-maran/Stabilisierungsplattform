// PWMGenerator.h
// Header file for the PWMGenerator class

#ifndef PWM_GENERATOR_H
#define PWM_GENERATOR_H

#include "mbed.h"
#include "env.h"

class PWMGenerator {
public:
    
    PWMGenerator();

    // Set duty cycle for both PWM channels (in percentage, 0% to 100%)
    void setDutyCycle_Pwm1( float dutyCycleCh4);
    void setDutyCycle_Pwm2( float dutyCycleCh3);

private:
    void configureTimer();     // Configures TIM2 timer
    void configureChannels();  // Configures TIM2 channels
};

#endif // PWM_GENERATOR_H
