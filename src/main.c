#include <stm32l4xx_hal.h>
#include <string.h>
#include <stdio.h>
#include <stdbool.h>

#include <i2c2.h>

#include <hts221.h>
#include <lps22hb.h>
#include <m24sr.h>

#define HTS221_ADDR 0xBE
#define LPS22HB_ADDR 0xBA
#define M24SR_ADDR 0xAC

volatile bool lps22hb_drdy = false;
volatile bool hts221_drdy = false;

DMA_HandleTypeDef dma;
UART_HandleTypeDef uart;

void led_gpio_init();
void uart_gpio_init();
void uart_dma_init();
void uart_init();

#include <fifo.h>
volatile size_t in_transit = 0;
#if 1
volatile uint8_t buffer[256];
volatile fifo_t fifo_g = {
    .buffer = buffer,
    .head = 0,
    .tail = 0,
    .capacity = 255
};
#else
FIFO_DEF(fifo_g, 255);
#endif

void start_transfer() {
    uint8_t* start = (uint8_t*)(fifo_g.buffer + fifo_g.tail);
    size_t size = fifo_size(&fifo_g);
    if (fifo_g.tail > fifo_g.head) size = fifo_g.capacity - fifo_g.tail + 1;
    // wait for previous transfer to complete
    while (dma.State != HAL_DMA_STATE_READY);
    HAL_UART_Transmit_DMA(&uart, start, size);
    in_transit = size;
}

void end_transfer() {
    fifo_pop(&fifo_g, NULL, in_transit);
    in_transit = 0;
}

ssize_t _write(int fd, const void *buf, size_t count) {
    HAL_GPIO_WritePin(GPIOA, GPIO_PIN_5, GPIO_PIN_SET);
    #if 1
    // memcpy(fifo_g.buffer, buf, count);
    for (int i = 0; i < count; ++i) {
        asm("nop");
    }
    #else
    // HAL_UART_Transmit(&uart, (uint8_t*)buf, count, HAL_MAX_DELAY);
    size_t to_send = count;
    while (to_send > 0) {
        size_t sent = fifo_push(&fifo_g, buf, to_send);
        to_send -= sent;

        // debug when waiting on full buffer
        GPIO_PinState status = (sent == 0) ? GPIO_PIN_SET : GPIO_PIN_RESET;
        HAL_GPIO_WritePin(GPIOB, GPIO_PIN_14, status);
    }
    start_transfer();
    #endif
    HAL_GPIO_WritePin(GPIOA, GPIO_PIN_5, GPIO_PIN_RESET);
    return count;
}

void SysTick_Handler() {
    HAL_IncTick();
    HAL_SYSTICK_IRQHandler();
}

int main(void) {
    asm(".global _printf_float");
    HAL_Init();

    float H_OUT, T_OUT, P_OUT;

    led_gpio_init();
    uart_gpio_init();
    uart_dma_init();
    uart_init();
    i2c2_init();

    // configure data ready pins
    // PD10 ----->  LPS22HB_INT_DRDY_EXTI10
    // PD15 ----->  HTS221_INT_DRDY_EXTI15
    __GPIOD_CLK_ENABLE();
    GPIO_InitTypeDef drdy_pin;
    drdy_pin.Pin = GPIO_PIN_10 | GPIO_PIN_15;
    drdy_pin.Mode = GPIO_MODE_IT_FALLING;
    drdy_pin.Pull = GPIO_NOPULL;
    drdy_pin.Speed = GPIO_SPEED_LOW;
    HAL_GPIO_Init(GPIOD, &drdy_pin);

    __SYSCFG_CLK_ENABLE();
    EXTI->IMR1 |= EXTI_IMR1_IM10 | EXTI_IMR1_IM15;
    EXTI->FTSR1 |= EXTI_FTSR1_FT10 | EXTI_FTSR1_FT15;
    HAL_NVIC_SetPriority(EXTI15_10_IRQn, 0, 0);
    HAL_NVIC_EnableIRQ(EXTI15_10_IRQn);

    LPS22HB_P_Init(LPS22HB_ADDR);
    // LPS22HB_T_Init(LPS22HB_ADDR, &t_init);
    SENSOR_IO_Write(LPS22HB_ADDR, LPS22HB_CTRL_REG1, 0x10);
    SENSOR_IO_Write(LPS22HB_ADDR, LPS22HB_CTRL_REG2, 0x01);
    SENSOR_IO_Write(LPS22HB_ADDR, LPS22HB_CTRL_REG3, 0x84);

    HTS221_H_Init(HTS221_ADDR);
    uint8_t ctrl1 = SENSOR_IO_Read(HTS221_ADDR, HTS221_CTRL_REG1);
    uint8_t ctrl2 = SENSOR_IO_Read(HTS221_ADDR, HTS221_CTRL_REG2);
    uint8_t ctrl3 = SENSOR_IO_Read(HTS221_ADDR, HTS221_CTRL_REG3);
    ctrl1 |= HTS221_PD_MASK | (0x01 << HTS221_ODR_BIT);
    ctrl2 |= HTS221_ONE_SHOT_MASK;
    ctrl3 |= HTS221_DRDY_H_L_MASK | HTS221_DRDY_MASK;
    ctrl3 &= ~(HTS221_PP_OD_MASK);
    SENSOR_IO_Write(HTS221_ADDR, HTS221_CTRL_REG1, ctrl1);
    SENSOR_IO_Write(HTS221_ADDR, HTS221_CTRL_REG2, ctrl2);
    SENSOR_IO_Write(HTS221_ADDR, HTS221_CTRL_REG3, ctrl3);

    uint8_t buffer[16384];
    printf("\n\n");
    M24SR_Init(M24SR_ADDR, M24SR_GPO_POLLING);
    NFC_IO_RfDisable(GPIO_PIN_RESET);
    #define printt(thing) { uint16_t ret = thing; if (ret != 0x9000) { printf(#thing ": %04X\n", ret); } }

    // open i2c session
    printt(M24SR_GetSession(M24SR_ADDR));
    // select application
    printt(M24SR_SelectApplication(M24SR_ADDR));
    // select CC file
    printt(M24SR_SelectCCfile(M24SR_ADDR));
    // read CC file
    printt(M24SR_ReadBinary(M24SR_ADDR, 0x0009, 2, buffer));
    int16_t file_id = buffer[1] | (buffer[0]<<8);
    printf("file_id: %04X\n", file_id);
    // select NDEF file
    printt(M24SR_SelectNDEFfile(M24SR_ADDR, file_id));
    // read NDEF file
    printt(M24SR_ReadBinary(M24SR_ADDR, 0x0000, 2, buffer));
    uint16_t file_len = buffer[1] | (buffer[0]<<8);
    printf("file_len: %04X\n", file_len);
    printt(M24SR_ReadBinary(M24SR_ADDR, 0x0002, file_len, buffer));
    printf("message: ");
    for (int i = 0; i < file_len; ++i) {
        printf("%02X", buffer[i]);
    }
    printf("\n");
    // deselect
    M24SR_Deselect(M24SR_ADDR);

    printf("Welcome to the STM32L4 Discovery Kit!\n");

    printf("initial HTS221 read:\n");
    H_OUT = HTS221_H_ReadHumidity(HTS221_ADDR);
    T_OUT = HTS221_T_ReadTemp(HTS221_ADDR);
    printf("  H:    %f\n", H_OUT);
    printf("  T:    %6.3f\n", T_OUT);

    printf("initial LPS22HB read:\n");
    P_OUT = LPS22HB_P_ReadPressure(LPS22HB_ADDR);
    T_OUT = LPS22HB_T_ReadTemp(LPS22HB_ADDR);
    printf("  P:    %5.2f\n", P_OUT * 0.02953);
    printf("  T:    %6.3f\n", T_OUT);

    while (1) {
        if (hts221_drdy) {
            printf("hts221 data ready:\n");
            H_OUT = HTS221_H_ReadHumidity(HTS221_ADDR);
            T_OUT = HTS221_T_ReadTemp(HTS221_ADDR);
            printf("  H:    %f\n", H_OUT);
            printf("  T:    %6.3f\n", T_OUT);
            hts221_drdy = false;
        }

        if (lps22hb_drdy) {
            printf("lps22hb data ready:\n");
            P_OUT = LPS22HB_P_ReadPressure(LPS22HB_ADDR);
            T_OUT = LPS22HB_T_ReadTemp(LPS22HB_ADDR);
            printf("  P:    %5.2f\n", P_OUT * 0.02953);
            printf("  T:    %6.3f\n", T_OUT);
            lps22hb_drdy = false;
        }

        if (in_transit == 0 && !fifo_empty(&fifo_g)) {
            start_transfer();
        }
        printf("hello world\n");
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
    end_transfer();
}

void USART1_IRQHandler(void) {
    HAL_UART_IRQHandler(&uart);
}

void EXTI15_10_IRQHandler(void) {
    if (__HAL_GPIO_EXTI_GET_FLAG(GPIO_PIN_15)) {
        hts221_drdy = true;
    }
    if (__HAL_GPIO_EXTI_GET_FLAG(GPIO_PIN_10)) {
        lps22hb_drdy = true;
    }
    HAL_GPIO_EXTI_IRQHandler(GPIO_PIN_15);
    HAL_GPIO_EXTI_IRQHandler(GPIO_PIN_10);
}