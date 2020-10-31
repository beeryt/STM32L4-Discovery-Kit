#include <stm32l4xx_hal.h>
#include <string.h>
#include <stdio.h>
#include <stdbool.h>

#define HTS221_ADDR 0xBE
#define LPS22HB_ADDR 0xBA

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
volatile bool do_lps22hb_read = false;

void lps22hb_dump() {
    HAL_I2C_Mem_Read(&i2c2, LPS22HB_ADDR, 0x0f, 1, &buffer[0], 1, HAL_MAX_DELAY);
    HAL_I2C_Mem_Read(&i2c2, LPS22HB_ADDR, 0x10, 1, &buffer[1], 1, HAL_MAX_DELAY);
    HAL_I2C_Mem_Read(&i2c2, LPS22HB_ADDR, 0x11, 1, &buffer[2], 1, HAL_MAX_DELAY);
    HAL_I2C_Mem_Read(&i2c2, LPS22HB_ADDR, 0x12, 1, &buffer[3], 1, HAL_MAX_DELAY);

    printf("  WHO_AM_I:     %02X\n", buffer[0]);
    printf("  CTRL_REG1:    %02X\n", buffer[1]);
    printf("  CTRL_REG2:    %02X\n", buffer[2]);
    printf("  CTRL_REG3:    %02X\n", buffer[3]);
}

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

void lps22hb_read() {
    HAL_I2C_Mem_Read(&i2c2, LPS22HB_ADDR, 0x2B, 1, &buffer[0], 1, HAL_MAX_DELAY);
    HAL_I2C_Mem_Read(&i2c2, LPS22HB_ADDR, 0x2C, 1, &buffer[1], 1, HAL_MAX_DELAY);
    HAL_I2C_Mem_Read(&i2c2, LPS22HB_ADDR, 0x28, 1, &buffer[2], 1, HAL_MAX_DELAY);
    HAL_I2C_Mem_Read(&i2c2, LPS22HB_ADDR, 0x29, 1, &buffer[3], 1, HAL_MAX_DELAY);
    HAL_I2C_Mem_Read(&i2c2, LPS22HB_ADDR, 0x2A, 1, &buffer[4], 1, HAL_MAX_DELAY);

    int16_t TEMP_OUT = (int16_t)(buffer[0] | (buffer[1]<<8));
    int32_t PRES_OUT = (int32_t)(((buffer[2]<<8) | (buffer[3]<<16) | (buffer[4]<<24))>>8);
    printf("  P:    %f\n", (float)PRES_OUT / 4096.0f);
    printf("  C:    %f\n", (float)TEMP_OUT / 100.0f);
}

typedef struct {
    uint8_t H0_rH_x2;
    uint8_t H1_rh_x2;
    uint16_t T0_degC_x8;
    uint16_t T1_degC_x8;

    int16_t H0_OUT;
    int16_t H1_OUT;

    int16_t T0_OUT;
    int16_t T1_OUT;

    float H_m;
    float H_b;
    float T_m;
    float T_b;
} HTS221_Calib;

static HTS221_Calib hts221_calib_dat;

void hts221_calibrate() {
    HTS221_Calib calib;
    uint8_t buffer[13];
    HAL_I2C_Mem_Read(&i2c2, HTS221_ADDR, 0x30, 1, &buffer[0],  1, HAL_MAX_DELAY);
    HAL_I2C_Mem_Read(&i2c2, HTS221_ADDR, 0x31, 1, &buffer[1],  1, HAL_MAX_DELAY);
    HAL_I2C_Mem_Read(&i2c2, HTS221_ADDR, 0x32, 1, &buffer[2],  1, HAL_MAX_DELAY);
    HAL_I2C_Mem_Read(&i2c2, HTS221_ADDR, 0x33, 1, &buffer[3],  1, HAL_MAX_DELAY);
    HAL_I2C_Mem_Read(&i2c2, HTS221_ADDR, 0x35, 1, &buffer[4],  1, HAL_MAX_DELAY);
    HAL_I2C_Mem_Read(&i2c2, HTS221_ADDR, 0x36, 1, &buffer[5],  1, HAL_MAX_DELAY);
    HAL_I2C_Mem_Read(&i2c2, HTS221_ADDR, 0x37, 1, &buffer[6],  1, HAL_MAX_DELAY);
    HAL_I2C_Mem_Read(&i2c2, HTS221_ADDR, 0x3A, 1, &buffer[7],  1, HAL_MAX_DELAY);
    HAL_I2C_Mem_Read(&i2c2, HTS221_ADDR, 0x3B, 1, &buffer[8],  1, HAL_MAX_DELAY);
    HAL_I2C_Mem_Read(&i2c2, HTS221_ADDR, 0x3C, 1, &buffer[9],  1, HAL_MAX_DELAY);
    HAL_I2C_Mem_Read(&i2c2, HTS221_ADDR, 0x3D, 1, &buffer[10], 1, HAL_MAX_DELAY);
    HAL_I2C_Mem_Read(&i2c2, HTS221_ADDR, 0x3E, 1, &buffer[11], 1, HAL_MAX_DELAY);
    HAL_I2C_Mem_Read(&i2c2, HTS221_ADDR, 0x3F, 1, &buffer[12], 1, HAL_MAX_DELAY);
    
    calib.H0_rH_x2 = (uint8_t)buffer[0];
    calib.H1_rh_x2 = (uint8_t)buffer[1];
    calib.T0_degC_x8 = (uint16_t)buffer[2] | (uint16_t)((buffer[4]&0x3)<<8);
    calib.T1_degC_x8 = (uint16_t)buffer[3] | (uint16_t)(((buffer[4]>>2)&0x3)<<8);
    calib.H0_OUT = (int16_t)(buffer[5] | (buffer[6]<<8));
    calib.H1_OUT = (int16_t)(buffer[7] | (buffer[8]<<8));
    calib.T0_OUT = (int16_t)(buffer[9] | (buffer[10]<<8));
    calib.T1_OUT = (int16_t)(buffer[11] | (buffer[12]<<8));

    calib.H_m = (float)(calib.H1_rh_x2 - calib.H0_rH_x2) / (float)(calib.H1_OUT - calib.H0_OUT);
    calib.H_b = (float)(calib.H0_rH_x2) - (float)(calib.H_m * calib.H0_OUT);

    calib.T_m = (float)(calib.T1_degC_x8 - calib.T0_degC_x8) / (float)(calib.T1_OUT - calib.T0_OUT);
    calib.T_b = (float)(calib.T0_degC_x8) - (float)(calib.T_m * calib.T0_OUT);

    hts221_calib_dat = calib;
    printf("calibration:\n");
    printf("  H0_rH_x2:   %d\n", calib.H0_rH_x2);
    printf("  H1_rh_x2:   %d\n", calib.H1_rh_x2);
    printf("  T0_degC_x8: %d\n", calib.T0_degC_x8);
    printf("  T1_degC_x8: %d\n", calib.T1_degC_x8);
    printf("  H0_T0_OUT:  %d\n", calib.H0_OUT);
    printf("  H1_T1_OUT:  %d\n", calib.H1_OUT);
    printf("  T0_OUT:     %d\n", calib.T0_OUT);
    printf("  T1_OUT:     %d\n", calib.T1_OUT);
    printf("  T_m:        %f\n", calib.T_m);
    printf("  T_b:        %f\n", calib.T_b);
    printf("  T1_degC_x8: %f\n", calib.T1_OUT * calib.T_m + calib.T_b);
}

float convert_to_rh(int16_t H_IN) {
    return ((float)H_IN * hts221_calib_dat.H_m + hts221_calib_dat.H_b) / 2.0;
}

float convert_to_c(int16_t T_IN) {
    return ((float)T_IN * hts221_calib_dat.T_m + hts221_calib_dat.T_b) / 8.0;
}

float convert_to_f(int16_t T_IN) {
    float C = convert_to_c(T_IN);
    return (C * (9.0/5.0)) + 32.0;
}

void hts221_read() {
    static bool calibrated = false;
    if (!calibrated) {
        calibrated = true;
        hts221_calibrate();
    }

    HAL_I2C_Mem_Read(&i2c2, HTS221_ADDR, 0x28, 1, &buffer[6], 1, HAL_MAX_DELAY);
    HAL_I2C_Mem_Read(&i2c2, HTS221_ADDR, 0x29, 1, &buffer[7], 1, HAL_MAX_DELAY);
    HAL_I2C_Mem_Read(&i2c2, HTS221_ADDR, 0x2A, 1, &buffer[8], 1, HAL_MAX_DELAY);
    HAL_I2C_Mem_Read(&i2c2, HTS221_ADDR, 0x2B, 1, &buffer[9], 1, HAL_MAX_DELAY);
    int16_t H_OUT = (int16_t)((buffer[7] << 8) | buffer[6]);
    int16_t T_OUT = (int16_t)((buffer[9] << 8) | buffer[8]);
    printf("  rH:   %f\n", convert_to_rh(H_OUT));
    printf("  C:    %f\n", convert_to_c(T_OUT));
}

int main(void) {
    asm(".global _printf_float");
    HAL_Init();

    led_gpio_init();
    uart_gpio_init();
    uart_dma_init();
    uart_init();
    i2c_init();

    printf("Welcome to the STM32L4 Discovery Kit!\n");

    // configure data ready pins
    // PD15 ----->  HTS221_DRDY_EXTI15
    // PD10 ----->  LPS22HB_INT_DRDY_EXTI10
    __GPIOD_CLK_ENABLE();
    GPIO_InitTypeDef drdy_pin;
    drdy_pin.Pin = GPIO_PIN_15 | GPIO_PIN_10;
    drdy_pin.Mode = GPIO_MODE_IT_FALLING;
    drdy_pin.Pull = GPIO_NOPULL;
    drdy_pin.Speed = GPIO_SPEED_FREQ_VERY_HIGH;
    HAL_GPIO_Init(GPIOD, &drdy_pin);

    // configure external interrupt falling trigger
    __SYSCFG_CLK_ENABLE();
    SYSCFG->EXTICR[3] |= SYSCFG_EXTICR4_EXTI15_PD | SYSCFG_EXTICR3_EXTI10_PD;
    EXTI->IMR1 |= EXTI_IMR1_IM15 | EXTI_IMR1_IM10;
    EXTI->FTSR1 |= EXTI_FTSR1_FT15 | EXTI_FTSR1_FT10;
    HAL_NVIC_SetPriority(EXTI15_10_IRQn, 0, 0);
    HAL_NVIC_EnableIRQ(EXTI15_10_IRQn);

    // power-on HTS221 and setup DRDY_EXTI15
    buffer[0] = 0x81; // power on, 1Hz
    buffer[1] = 0x84; // DRDY enabled, active low, push-pull
    HAL_I2C_Mem_Write(&i2c2, HTS221_ADDR, 0x20, 1, &buffer[0], 1, HAL_MAX_DELAY);
    HAL_I2C_Mem_Write(&i2c2, HTS221_ADDR, 0x22, 1, &buffer[1], 1, HAL_MAX_DELAY);

    // power-on LPS22HB and setup DRDY_EXTI10
    buffer[0] = 0x84; // DRDY enabled, active low, push-pull
    buffer[1] = 0x10; // ODR for 1Hz
    buffer[2] = 0x11; // one-shot
    HAL_I2C_Mem_Write(&i2c2, LPS22HB_ADDR, 0x12, 1, &buffer[0], 1, HAL_MAX_DELAY);
    HAL_I2C_Mem_Write(&i2c2, LPS22HB_ADDR, 0x10, 1, &buffer[1], 1, HAL_MAX_DELAY);
    HAL_I2C_Mem_Write(&i2c2, LPS22HB_ADDR, 0x11, 1, &buffer[2], 1, HAL_MAX_DELAY);

    printf("initial LPS22HB memory:\n");
    lps22hb_dump();
    lps22hb_read();

    printf("initial HTS221 memory:\n");
    hts221_dump();
    hts221_read();

    printf("triggering one-shot!\n");
    buffer[3] |= 0x01; // trigger one-shot
    HAL_I2C_Mem_Write(&i2c2, HTS221_ADDR, 0x21, 1, &buffer[3], 1, HAL_MAX_DELAY);

    while (1) {
        if (do_read) {
            do_read = false;
            printf("data ready:\n");
            hts221_read();
        }

        if (do_lps22hb_read) {
            do_lps22hb_read = false;
            printf("lps22hb data ready:\n");
            lps22hb_read();
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
    if (__HAL_GPIO_EXTI_GET_FLAG(GPIO_PIN_10)) {
        do_lps22hb_read = true;
    }
    HAL_GPIO_EXTI_IRQHandler(GPIO_PIN_15);
    HAL_GPIO_EXTI_IRQHandler(GPIO_PIN_10);
}