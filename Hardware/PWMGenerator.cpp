// PWMGenerator.cpp
// Implementation file for the PWMGenerator class

#include "PWMGenerator.h"

// Constructor: Initializes the timer and channels for PWM generation
PWMGenerator::PWMGenerator() {
    configureTimer();
    configureChannels();
}

// Set the duty cycle for PWM channel 3 (CH3)
// Parameters:
// - dutyCycleCh3: Duty cycle for channel 3 in percentage (0% to 100%)
void PWMGenerator::setDutyCycle_Pwm1(float dutyCycleCh4) {
    // Clamp duty cycles between 0% and 100%

    //dutyCycleCh4 = 100.0f-dutyCycleCh4;


    if (dutyCycleCh4 < 0.0f) dutyCycleCh4 = 0.0f;
    if (dutyCycleCh4 > 100.0f) dutyCycleCh4 = 100.0f;
    // Calculate and set the pulse width for channel 3
    TIM4->CCR4 = static_cast<uint32_t>((TIM4->ARR + 1) * dutyCycleCh4 / 100.0f);
}

// Set the duty cycle for PWM channel 4 (CH4)
// Parameters:
// - dutyCycleCh4: Duty cycle for channel 4 in percentage (0% to 100%)
void PWMGenerator::setDutyCycle_Pwm2(float dutyCycleCh3) {
    // Clamp duty cycles between 0% and 100%

    //dutyCycleCh3 = 100.0f-dutyCycleCh3;

    if (dutyCycleCh3 < 0.0f) dutyCycleCh3 = 0.0f;
    if (dutyCycleCh3 > 100.0f) dutyCycleCh3 = 100.0f;
    // Calculate and set the pulse width for channel 4
    TIM4->CCR3 = static_cast<uint32_t>((TIM4->ARR + 1) *  dutyCycleCh3 / 100.0f);
}




// Configures TIM4 for PWM generation
void PWMGenerator::configureTimer() {
    // Enable GPIOB clock for PB9 and PB8
    RCC->AHB1ENR |= RCC_AHB1ENR_GPIOBEN;

    // Configure PB9 and PB8 as alternate function (AF2 for TIM4)
    GPIOB->MODER &= ~(GPIO_MODER_MODER9 | GPIO_MODER_MODER8);
    GPIOB->MODER |= (GPIO_MODER_MODER9_1 | GPIO_MODER_MODER8_1); // Set alternate function mode
    GPIOB->AFR[1] &= ~(0xF << (4 * (9 - 8))); // Clear alternate function bits for PB9
    GPIOB->AFR[1] |= (2 << (4 * (9 - 8)));    // Set AF2 for TIM4 CH4 on PB9
    GPIOB->AFR[1] &= ~(0xF << (4 * (8 - 8))); // Clear alternate function bits for PB8
    GPIOB->AFR[1] |= (2 << (4 * (8 - 8)));    // Set AF2 for TIM4 CH3 on PB8

    // Enable TIM4 clock
    RCC->APB1ENR |= RCC_APB1ENR_TIM4EN;

    // Set TIM4 prescaler to achieve a 10 kHz PWM frequency
    TIM4->PSC = 0;
    //TIM4->ARR = 8994 - 1; // Set auto-reload value for 100 steps (0% to 100% duty cycle)
    TIM4->ARR = (90000000/PWMFREQUENZ)-1; // Für ~22 kHz PWM-Frequenz


    // Enable auto-reload preload
    TIM4->CR1 |= TIM_CR1_ARPE;

    // Configure channels 3 and 4 for PWM mode 1
    TIM4->CCMR2 |= (TIM_CCMR2_OC3M_1 | TIM_CCMR2_OC3M_2 | TIM_CCMR2_OC3PE); // Channel 3
    TIM4->CCMR2 |= (TIM_CCMR2_OC4M_1 | TIM_CCMR2_OC4M_2 | TIM_CCMR2_OC4PE); // Channel 4

    // Enable capture/compare for channels 3 and 4
    TIM4->CCER |= (TIM_CCER_CC3E | TIM_CCER_CC4E);

    // Enable TIM4 main output
    TIM4->BDTR |= TIM_BDTR_MOE;

    // Start TIM4
    TIM4->CR1 |= TIM_CR1_CEN;
}

// Configures the initial state of the PWM channels
void PWMGenerator::configureChannels() {
    // Initialize duty cycles to 0%
    TIM4->CCR3 = 0;
    TIM4->CCR4 = 0;
}
