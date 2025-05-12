// PrintfUart.cpp
#include "PrintfUart.h"
#include <cstdarg>

// Globale UART/DMA Handles – notwendig für STM32-HAL
UART_HandleTypeDef huart2;
DMA_HandleTypeDef hdma_usart2_tx;

// Flag für Übertragungsbereitschaft
static volatile bool uart_ready = true;

PrintfUart::PrintfUart() {}

void PrintfUart::init() {
    __HAL_RCC_USART2_CLK_ENABLE();
    __HAL_RCC_DMA1_CLK_ENABLE();
    __HAL_RCC_GPIOA_CLK_ENABLE();

    GPIO_InitTypeDef GPIO_InitStruct = {0};
    GPIO_InitStruct.Pin = GPIO_PIN_2 | GPIO_PIN_3;
    GPIO_InitStruct.Mode = GPIO_MODE_AF_PP;
    GPIO_InitStruct.Pull = GPIO_NOPULL;
    GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_VERY_HIGH;
    GPIO_InitStruct.Alternate = GPIO_AF7_USART2;
    HAL_GPIO_Init(GPIOA, &GPIO_InitStruct);

    huart2.Instance = USART2;
    huart2.Init.BaudRate = 115200;
    huart2.Init.WordLength = UART_WORDLENGTH_8B;
    huart2.Init.StopBits = UART_STOPBITS_1;
    huart2.Init.Parity = UART_PARITY_NONE;
    huart2.Init.Mode = UART_MODE_TX_RX;
    huart2.Init.HwFlowCtl = UART_HWCONTROL_NONE;
    huart2.Init.OverSampling = UART_OVERSAMPLING_16;
    HAL_UART_Init(&huart2);

    hdma_usart2_tx.Instance = DMA1_Stream6;
    hdma_usart2_tx.Init.Channel = DMA_CHANNEL_4;
    hdma_usart2_tx.Init.Direction = DMA_MEMORY_TO_PERIPH;
    hdma_usart2_tx.Init.PeriphInc = DMA_PINC_DISABLE;
    hdma_usart2_tx.Init.MemInc = DMA_MINC_ENABLE;
    hdma_usart2_tx.Init.PeriphDataAlignment = DMA_PDATAALIGN_BYTE;
    hdma_usart2_tx.Init.MemDataAlignment = DMA_MDATAALIGN_BYTE;
    hdma_usart2_tx.Init.Mode = DMA_NORMAL;
    hdma_usart2_tx.Init.Priority = DMA_PRIORITY_LOW;
    hdma_usart2_tx.Init.FIFOMode = DMA_FIFOMODE_DISABLE;
    HAL_DMA_Init(&hdma_usart2_tx);

    __HAL_LINKDMA(&huart2, hdmatx, hdma_usart2_tx);

    HAL_NVIC_SetPriority(DMA1_Stream6_IRQn, 0, 0);
    HAL_NVIC_EnableIRQ(DMA1_Stream6_IRQn);
    HAL_NVIC_SetPriority(USART2_IRQn, 1, 0);
    HAL_NVIC_EnableIRQ(USART2_IRQn);
}

void PrintfUart::print(const char* format, ...) {
    if (!uart_ready) return;

    va_list args;
    va_start(args, format);
    int len = vsnprintf(dma_buffer, sizeof(dma_buffer), format, args);
    va_end(args);

    if (len > 0 && len < sizeof(dma_buffer)) {
        uart_ready = false;
        HAL_UART_Transmit_DMA(&huart2, (uint8_t*)dma_buffer, len);
    }
}

// Interrupt Handler
extern "C" void DMA1_Stream6_IRQHandler(void) {
    HAL_DMA_IRQHandler(&hdma_usart2_tx);
}

extern "C" void USART2_IRQHandler(void) {
    HAL_UART_IRQHandler(&huart2);
}

// Wird aufgerufen, wenn DMA-Transfer abgeschlossen
extern "C" void HAL_UART_TxCpltCallback(UART_HandleTypeDef *huart) {
    if (huart->Instance == USART2) {
        uart_ready = true;
    }
}
