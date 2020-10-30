#include <stm32l4xx_hal.h>
#include <string.h>
#include <stdio.h>
#include <stdbool.h>

#define HTS221_ADDR 0xBE

DMA_HandleTypeDef dma;
UART_HandleTypeDef uart;
I2C_HandleTypeDef i2c2;

void led_gpio_init();
void uart_gpio_init();
void uart_dma_init();
void uart_init();
void i2c_init();

ssize_t _write(int fd, const void *buf, size_t count) {
    HAL_GPIO_WritePin(GPIOA, GPIO_PIN_5, GPIO_PIN_SET);
    HAL_UART_Transmit(&uart, (uint8_t*)buf, count, HAL_MAX_DELAY);
    HAL_GPIO_WritePin(GPIOA, GPIO_PIN_5, GPIO_PIN_RESET);
    return count;
}

void SysTick_Handler() {
    HAL_IncTick();
    HAL_SYSTICK_IRQHandler();
}

uint8_t buffer[12];
volatile bool do_read = false;

void hts221_dump() {
    HAL_I2C_Mem_Read(&i2c2, HTS221_ADDR, 0x0f, 1, &buffer[0], 1, HAL_MAX_DELAY);
    HAL_I2C_Mem_Read(&i2c2, HTS221_ADDR, 0x10, 1, &buffer[1], 1, HAL_MAX_DELAY);
    HAL_I2C_Mem_Read(&i2c2, HTS221_ADDR, 0x20, 1, &buffer[2], 1, HAL_MAX_DELAY);
    HAL_I2C_Mem_Read(&i2c2, HTS221_ADDR, 0x21, 1, &buffer[3], 1, HAL_MAX_DELAY);
    HAL_I2C_Mem_Read(&i2c2, HTS221_ADDR, 0x22, 1, &buffer[4], 1, HAL_MAX_DELAY);
    HAL_I2C_Mem_Read(&i2c2, HTS221_ADDR, 0x27, 1, &buffer[5], 1, HAL_MAX_DELAY);

    printf("  WHO_AM_I:   %02X\n", buffer[0]);
    printf("  AV_CONF:    %02X\n", buffer[1]);
    printf("  CTRL_REG1:  %02X\n", buffer[2]);
    printf("  CTRL_REG2:  %02X\n", buffer[3]);
    printf("  CTRL_REG3:  %02X\n", buffer[4]);
    printf("  STATUS_REG: %02X\n", buffer[5]);
}

void hts221_read() {
        HAL_I2C_Mem_Read(&i2c2, HTS221_ADDR, 0x27, 1, &buffer[5], 1, HAL_MAX_DELAY);
        HAL_I2C_Mem_Read(&i2c2, HTS221_ADDR, 0x28, 1, &buffer[6], 1, HAL_MAX_DELAY);
        HAL_I2C_Mem_Read(&i2c2, HTS221_ADDR, 0x29, 1, &buffer[7], 1, HAL_MAX_DELAY);
        HAL_I2C_Mem_Read(&i2c2, HTS221_ADDR, 0x2A, 1, &buffer[8], 1, HAL_MAX_DELAY);
        HAL_I2C_Mem_Read(&i2c2, HTS221_ADDR, 0x2B, 1, &buffer[9], 1, HAL_MAX_DELAY);
        printf("  HUMIDITY_OUT_L: %02X\n", buffer[6]);
        printf("  HUMIDITY_OUT_H: %02X\n", buffer[7]);
        printf("  TEMP_OUT_L:     %02X\n", buffer[8]);
        printf("  TEMP_OUT_H:     %02X\n", buffer[9]);
        printf("  STATUS_REG:     %02X\n", buffer[5]);
}

int main(void) {
    HAL_Init();

    led_gpio_init();
    uart_gpio_init();
    uart_dma_init();
    uart_init();
    i2c_init();

    printf("Welcome to the STM32L4 Discovery Kit!\n");

    // configure data ready pin
    // PD15 ----->  HTS221_DRDY_EXTI15
    __GPIOD_CLK_ENABLE();
    GPIO_InitTypeDef drdy_pin;
    drdy_pin.Pin = GPIO_PIN_15;
    drdy_pin.Mode = GPIO_MODE_IT_FALLING;
    drdy_pin.Pull = GPIO_NOPULL;
    drdy_pin.Speed = GPIO_SPEED_FREQ_VERY_HIGH;
    HAL_GPIO_Init(GPIOD, &drdy_pin);

    // configure EXTI15 for PD15
    __SYSCFG_CLK_ENABLE();
    SYSCFG->EXTICR[3] |= SYSCFG_EXTICR4_EXTI15_PD;
    EXTI->IMR1 |= EXTI_IMR1_IM15;
    EXTI->FTSR1 |= EXTI_FTSR1_FT15;
    HAL_NVIC_SetPriority(EXTI15_10_IRQn, 0, 0);
    HAL_NVIC_EnableIRQ(EXTI15_10_IRQn);

    buffer[0] = 0x81; // power on, 1Hz
    buffer[1] = 0x84; // DRDY enabled, active low, push-pull

    // power-on HTS221 and setup DRDY_EXTI15
    HAL_I2C_Mem_Write(&i2c2, HTS221_ADDR, 0x20, 1, &buffer[0], 1, HAL_MAX_DELAY);
    HAL_I2C_Mem_Write(&i2c2, HTS221_ADDR, 0x22, 1, &buffer[1], 1, HAL_MAX_DELAY);

    printf("initial memory:\n");
    hts221_dump();

    if ((buffer[5] & 0x3)) {
        printf("initial reading:\n");
        hts221_read();
    }

    check_exti15();
    printf("triggering one-shot!\n");
    buffer[3] |= 0x01; // trigger one-shot
    HAL_I2C_Mem_Write(&i2c2, HTS221_ADDR, 0x21, 1, &buffer[3], 1, HAL_MAX_DELAY);
    printf("mem dump:\n");
    hts221_dump();

    while (1)
    {
        if (do_read)
        {
            do_read = false;
            printf("data ready:\n");
            hts221_read();
        }
    }

    return 0;
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

void i2c_init() {
    // configure i2c gpio pins
    // PB10 ----->  SCL
    // PB11 ----->  SDA
    __GPIOB_CLK_ENABLE();
    GPIO_InitTypeDef i2c_pins;
    i2c_pins.Pin = GPIO_PIN_10 | GPIO_PIN_11;
    i2c_pins.Mode = GPIO_MODE_AF_OD;
    i2c_pins.Pull = GPIO_PULLUP;
    i2c_pins.Speed = GPIO_SPEED_FAST;
    i2c_pins.Alternate = GPIO_AF4_I2C2;
    HAL_GPIO_Init(GPIOB, &i2c_pins);

    // configure I2C2 module
    __I2C2_CLK_ENABLE();
    i2c2.Instance = I2C2;
    i2c2.Init.Timing = 0x200090E;
    i2c2.Init.OwnAddress1 = i2c2.Init.OwnAddress2 = 0x77;
    i2c2.Init.AddressingMode = I2C_ADDRESSINGMODE_7BIT;
    i2c2.Init.DualAddressMode = I2C_DUALADDRESS_DISABLED;
    i2c2.Init.GeneralCallMode = I2C_GENERALCALL_DISABLED;
    i2c2.Init.NoStretchMode = I2C_NOSTRETCH_DISABLED;
    HAL_I2C_Init(&i2c2);
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

void EXTI15_10_IRQHandler(void) {
    if (__HAL_GPIO_EXTI_GET_FLAG(GPIO_PIN_15)) {
        do_read = true;
    }
    HAL_GPIO_EXTI_IRQHandler(GPIO_PIN_15);
}