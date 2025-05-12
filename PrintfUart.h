// PrintfUart.h
#pragma once

#include "mbed.h"
#include "stm32f4xx_hal.h"

// Globale Handles für HAL-Interrupts
extern UART_HandleTypeDef huart2;
extern DMA_HandleTypeDef hdma_usart2_tx;

class PrintfUart {
public:
    PrintfUart();
    void init();
    void print(const char* format, ...);

private:
    char dma_buffer[128];
};
