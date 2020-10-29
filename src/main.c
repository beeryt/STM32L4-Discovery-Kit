#include <stm32l4xx_hal.h>
#include <string.h>

DMA_HandleTypeDef dma;
UART_HandleTypeDef uart;

void led_gpio_init();
void uart_gpio_init();
void uart_dma_init();
void uart_init();

void SysTick_Handler() {
    HAL_IncTick();
    HAL_SYSTICK_IRQHandler();
}

int main(void) {
    HAL_Init();

    led_gpio_init();
    uart_gpio_init();
    uart_dma_init();
    uart_init();

    static char buffer[512] = "Welcome to the STM32L4 Discovery Kit!\n";
    HAL_UART_Transmit(&uart, (uint8_t*)buffer, strnlen(buffer, sizeof(buffer)-1), HAL_MAX_DELAY);
}

void led_gpio_init() {
    // LED configuration
    // PA5  ----->  LED1
    // PB14 ----->  LED2
    GPIO_InitTypeDef gpio;
    gpio.Speed = GPIO_SPEED_FREQ_LOW;
    gpio.Mode = GPIO_MODE_OUTPUT_PP;
    gpio.Pull = GPIO_PULLDOWN;
    gpio.Pin = GPIO_PIN_5;
    __GPIOA_CLK_ENABLE();
    HAL_GPIO_Init(GPIOA, &gpio);
    gpio.Pin = GPIO_PIN_14;
    __GPIOB_CLK_ENABLE();
    HAL_GPIO_Init(GPIOB, &gpio);
}

void uart_gpio_init() {
    // USART1 GPIO configuration
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
}

void uart_init() {
    // USART1 configuration (115200 baud, 8bit, no parity, 1 stop bit, no flow control)
    __USART1_CLK_ENABLE();
    uart.Instance = USART1;
    uart.Init.Mode = UART_MODE_TX_RX;
    uart.Init.BaudRate = 115200;
    uart.Init.WordLength = UART_WORDLENGTH_8B;
    uart.Init.Parity = UART_PARITY_NONE;
    uart.Init.StopBits = UART_STOPBITS_1;
    uart.Init.HwFlowCtl = UART_HWCONTROL_NONE;
    HAL_UART_Init(&uart);

    HAL_NVIC_SetPriority(USART1_IRQn, 0, 0);
    HAL_NVIC_EnableIRQ(USART1_IRQn);
}

void uart_dma_init() {
    __DMA1_CLK_ENABLE();

    dma.Instance = DMA1_Channel4;
    dma.Init.Request = DMA_REQUEST_2;
    dma.Init.Direction = DMA_MEMORY_TO_PERIPH;
    dma.Init.PeriphInc = DMA_PINC_DISABLE;
    dma.Init.MemInc = DMA_MINC_ENABLE;
    dma.Init.PeriphDataAlignment = DMA_PDATAALIGN_BYTE;
    dma.Init.MemDataAlignment = DMA_MDATAALIGN_BYTE;
    dma.Init.Mode = DMA_NORMAL;
    dma.Init.Priority = DMA_PRIORITY_VERY_HIGH;
    HAL_DMA_Init(&dma);

    __HAL_LINKDMA(&uart, hdmatx, dma);

    HAL_NVIC_SetPriority(DMA1_Channel4_IRQn, 0, 0);
    HAL_NVIC_EnableIRQ(DMA1_Channel4_IRQn);
}

void DMA1_Channel4_IRQHandler(void) {
    HAL_DMA_IRQHandler(&dma);
}

void USART1_IRQHandler(void) {
    HAL_UART_IRQHandler(&uart);
}
