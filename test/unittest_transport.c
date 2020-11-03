#include "unittest_transport.h"
#include <stm32l4xx_hal.h>

static UART_HandleTypeDef uart;

void unittest_uart_begin() {
    __HAL_RCC_GPIOB_CLK_ENABLE();
    __HAL_RCC_USART1_CLK_ENABLE();

    // PB6  ----->  USART1_TX
    // PB7  ----->  USART1_RX
    GPIO_InitTypeDef gpio;
    gpio.Speed = GPIO_SPEED_FREQ_VERY_HIGH;
    gpio.Mode = GPIO_MODE_AF_PP;
    gpio.Pull = GPIO_PULLUP;
    gpio.Alternate = GPIO_AF7_USART1;
    gpio.Pin = GPIO_PIN_6 | GPIO_PIN_7;
    __GPIOB_CLK_ENABLE();
    HAL_GPIO_Init(GPIOB, &gpio);

    // USART1 configuration (115200 baud, 8bit, no parity, 1 stop bit, no flow control)
    __USART1_CLK_ENABLE();
    uart.Instance = USART1;
    uart.Init.BaudRate = 115200;
    uart.Init.WordLength = UART_WORDLENGTH_8B;
    uart.Init.StopBits = UART_STOPBITS_1;
    uart.Init.Parity = UART_PARITY_NONE;
    uart.Init.Mode = UART_MODE_TX_RX;
    uart.Init.HwFlowCtl = UART_HWCONTROL_NONE;
    HAL_UART_Init(&uart);
    if (HAL_UART_Init(&uart) != HAL_OK) {
        while(1) {}
    }
}

void unittest_uart_putchar(char c) {
    HAL_UART_Transmit(&uart, (uint8_t*)(&c), 1, HAL_MAX_DELAY);
}

void unittest_uart_flush() {}

void unittest_uart_end() {
    __HAL_RCC_GPIOB_CLK_DISABLE();
    __HAL_RCC_USART1_CLK_DISABLE();
}