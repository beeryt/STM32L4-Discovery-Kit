// Base addresses for peripherals (from Table 19 in DS10969 Rev 5)
#define RCC_AHB2ENR_BASE    0x40021000
#define GPIOA_BASE          0x48000000
#define GPIOB_BASE          0x48000400

// Macros for IO register access
#define RCC_AHB2ENR (*((volatile unsigned int*)(RCC_AHB2ENR_BASE + 0x4C)))
#define GPIOA_MODER (*((volatile unsigned int*)(GPIOA_BASE + 0x00)))
#define GPIOA_ODR   (*((volatile unsigned int*)(GPIOA_BASE + 0x14))))
#define GPIOB_MODER (*((volatile unsigned int*)(GPIOB_BASE + 0x00)))
#define GPIOB_ODR   (*((volatile unsigned int*)(GPIOB_BASE + 0x14))))

// Macros for bit values
#define GPIOAEN     (1<<0)
#define GPIOBEN     (1<<1)
#define MODE5_0     (1<<10)
#define MODE5_1     (1<<11)
#define MODE14_0    (1<<28)
#define MODE14_1    (1<<29)
#define OD5         (1<<5)
#define OD14        (1<<14)

#define COUNTER_RESET (100000)

int main() {
    // setup LED1 (PA5) and LED2 (PB14) as GPIO output
    RCC_AHB2ENR |= (GPIOAEN | GPIOBEN); // enable clock for GPIOA and GPIOB
    GPIOA_MODER &= ~MODE5_1;            // enable output for PA5 (from reset value)
    GPIOB_MODER &= ~MODE14_1;           // enable output for PB14 (from reset value)

    // turn on LED2 (PB14)
    GPIOB_ODR |= OD14;

    int counter = 0;
    while (1) {
        // blink LED1 (PA5) and LED2 (PB14) indefinitely
        if (++counter > COUNTER_RESET) {
            GPIOA_ODR ^= OD5;
            GPIOB_ODR ^= OD14;
            counter = 0;
        }
    }
}