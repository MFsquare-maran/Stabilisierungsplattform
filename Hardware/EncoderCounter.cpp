/*
 * EncoderCounter.cpp
 * Copyright (c) 2024, ZHAW
 * All rights reserved.
 */

#include "EncoderCounter.h"
#include <cstdint>
#include <cstdio>


using namespace std;

EncoderCounter::EncoderCounter(PinName a, PinName b, PinName I,float gear, float ts,int direction) : TIM(nullptr), I_Counter(I) {
    if ((a == PA_6) && (b == PA_7)) {
        // TIM3 auf PA_6 und PA_7
        __HAL_RCC_GPIOA_CLK_ENABLE();
        __HAL_RCC_TIM3_CLK_ENABLE();

        GPIO_InitTypeDef GPIO_InitStruct = {0};
        GPIO_InitStruct.Pin = GPIO_PIN_6 | GPIO_PIN_7;
        GPIO_InitStruct.Mode = GPIO_MODE_AF_PP;
        GPIO_InitStruct.Pull = GPIO_PULLDOWN;  // <-- Pull-Down aktiviert
        GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
        GPIO_InitStruct.Alternate = GPIO_AF2_TIM3;
        HAL_GPIO_Init(GPIOA, &GPIO_InitStruct);

        TIM = TIM3;
    }
    else if ((a == PC_6) && (b == PC_7)) {
        // TIM8 auf PC_6 und PC_7
        __HAL_RCC_GPIOC_CLK_ENABLE();
        __HAL_RCC_TIM8_CLK_ENABLE();

        GPIO_InitTypeDef GPIO_InitStruct = {0};
        GPIO_InitStruct.Pin = GPIO_PIN_6 | GPIO_PIN_7;
        GPIO_InitStruct.Mode = GPIO_MODE_AF_PP;
        GPIO_InitStruct.Pull = GPIO_PULLDOWN;  // <-- Pull-Down aktiviert
        GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
        GPIO_InitStruct.Alternate = GPIO_AF3_TIM8;
        HAL_GPIO_Init(GPIOC, &GPIO_InitStruct);

        TIM = TIM8;
    }
    else {
        printf("Unsupported pin configuration!\n");
        return;
    }

    // Encoder-Konfiguration
    TIM_Handle.Instance = TIM;
    TIM_Handle.Init.Prescaler = 0;
    TIM_Handle.Init.CounterMode = TIM_COUNTERMODE_UP;
    TIM_Handle.Init.Period = 0xFFFF;
    TIM_Handle.Init.ClockDivision = TIM_CLOCKDIVISION_DIV1;
    TIM_Handle.Init.AutoReloadPreload = TIM_AUTORELOAD_PRELOAD_DISABLE;

    TIM_Encoder_InitTypeDef sEncoderConfig = {0};

    sEncoderConfig.EncoderMode = TIM_ENCODERMODE_TI12;

    sEncoderConfig.IC1Polarity = TIM_ICPOLARITY_RISING;
    sEncoderConfig.IC1Selection = TIM_ICSELECTION_DIRECTTI;
    sEncoderConfig.IC1Prescaler = TIM_ICPSC_DIV1;
    sEncoderConfig.IC1Filter = 4;

    sEncoderConfig.IC2Polarity = TIM_ICPOLARITY_RISING;
    sEncoderConfig.IC2Selection = TIM_ICSELECTION_DIRECTTI;
    sEncoderConfig.IC2Prescaler = TIM_ICPSC_DIV1;
    sEncoderConfig.IC2Filter = 4;

    HAL_TIM_Encoder_Init(&TIM_Handle, &sEncoderConfig);
    HAL_TIM_Encoder_Start(&TIM_Handle, TIM_CHANNEL_ALL);

    this->ts = ts;
    this->gear = gear;
    this->direction = direction;

}

EncoderCounter::~EncoderCounter() {}


void EncoderCounter::update() {
    counter_val = TIM->CNT;

    // Differenz und Überlaufkorrektur
    delta = (int16_t)((counter_val - counter_val_old + 32768) % 65536 - 32768);

    float delta_rotation;

    if(direction == 0)
    {
        delta_rotation = (float)delta / (gear * 4.0f * TICKS_PER_REVOLUTION);
    }
    if(direction == 1)
    {
        delta_rotation = -(float)delta / (gear * 4.0f * TICKS_PER_REVOLUTION);
    }

    // Position in Encoder-Revs → Motor-Revs → Rad
    
    rotation += delta_rotation;

    rotation_s = delta_rotation / ts;

    rad = rotation * 2.0f * PI;
    rad_s = rotation_s * 2.0f * PI;

    counter_val_old = counter_val;
}





void EncoderCounter::reset(float position) {
    if (TIM != nullptr) {
        TIM->CNT = 0x0000;
    }


    rotation = position;
    rotation_s = 0.0;
    rad = rotation * 2.0f * PI;
    rad_s = rotation_s * 2.0f * PI;
    delta = 0.0;
    counter_val = 0.0;
    counter_val_old = 0.0;

}



float EncoderCounter::get_rad_s() {
    return rad_s;
}

float EncoderCounter::get_rad() {
    return rad;
}

float EncoderCounter::get_revolutions() {
    return rotation;
}

float EncoderCounter::get_revolutions_s() {
    return rotation_s;
}

float EncoderCounter::get_revolutions_min() {
    return rotation_s * 60;
}

float EncoderCounter::get_mm_s() {
    return rotation_s * PI*D_PULLEY;
}

float EncoderCounter::get_mm() {
    return rotation * PI*D_PULLEY;
}


