#include <i2c2.h>

I2C_HandleTypeDef i2c2;

void i2c2_init() {
    // configure i2c2 gpio pins
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