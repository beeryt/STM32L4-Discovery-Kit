#include <fifo.h>
#include <unity.h>
#include <string.h>
#include <stdio.h>
#ifndef NATIVE
#include <stm32l4xx_hal.h>
#endif

void test_fifo_def(void) {
    const size_t buffer_size = 32;
    FIFO_DEF(fifo, buffer_size);

    TEST_ASSERT_EQUAL(0, fifo.head);
    TEST_ASSERT_EQUAL(0, fifo.tail);
    TEST_ASSERT_EQUAL(buffer_size, fifo.capacity);
    TEST_ASSERT_NOT_EQUAL(NULL, fifo.buffer);
}

void test_fifo_init(void) {
    const size_t buffer_size = 32;
    uint8_t buffer[buffer_size];

    fifo_t fifo = fifo_init(buffer, buffer_size);

    TEST_ASSERT_EQUAL(buffer, fifo.buffer);
    TEST_ASSERT_EQUAL(0, fifo.head);
    TEST_ASSERT_EQUAL(0, fifo.tail);
    TEST_ASSERT_EQUAL(buffer_size, fifo.capacity + 1);
}

void test_fifo_clear(void) {
    const size_t buffer_size = 32;
    FIFO_DEF(fifo, buffer_size);

    fifo.head = buffer_size / 2;
    fifo.tail = buffer_size / 4;

    fifo_clear(&fifo);
    
    TEST_ASSERT_EQUAL(0, fifo.head);
    TEST_ASSERT_EQUAL(0, fifo.tail);
    TEST_ASSERT_EQUAL(buffer_size, fifo.capacity);
    TEST_ASSERT_NOT_EQUAL(NULL, fifo.buffer);
}

void test_fifo_full(void) {
    const size_t buffer_size = 32;
    FIFO_DEF(fifo, buffer_size);

    uint8_t buffer[buffer_size];

    for (int i = 0; i < buffer_size; ++i) {
        TEST_ASSERT_FALSE(fifo_full(&fifo));
        size_t count = fifo_push(&fifo, buffer, 1);
        TEST_ASSERT_EQUAL(1, count);
    }

    TEST_ASSERT_TRUE(fifo_full(&fifo));

    for (int i = 0; i < buffer_size; ++i) {
        size_t count = fifo_pop(&fifo, buffer, 1);
        TEST_ASSERT_EQUAL(1, count);
        TEST_ASSERT_FALSE(fifo_full(&fifo));
    }
}

void test_fifo_empty(void) {
    const size_t buffer_size = 32;
    FIFO_DEF(fifo, buffer_size);

    uint8_t buffer[buffer_size];

    TEST_ASSERT_TRUE(fifo_empty(&fifo));

    for (int i = 0; i < buffer_size; ++i) {
        size_t count = fifo_push(&fifo, buffer, 1);
        TEST_ASSERT_EQUAL(1, count);
        TEST_ASSERT_FALSE(fifo_empty(&fifo));
    }

    for (int i = 0; i < buffer_size; ++i) {
        TEST_ASSERT_FALSE(fifo_empty(&fifo));
        size_t count = fifo_pop(&fifo, buffer, 1);
        TEST_ASSERT_EQUAL(1, count);
    }

    TEST_ASSERT_TRUE(fifo_empty(&fifo));
}

void test_fifo_size(void) {
    const size_t buffer_size = 32;
    FIFO_DEF(fifo, buffer_size);

    uint8_t buffer[buffer_size];

    for (int i = 1; i <= buffer_size; ++i) {
        // loop with increment [0, buffer_size-1]
        for (int j = 0; j < buffer_size; j += i) {
            size_t amount = (j + i >= buffer_size) ? buffer_size - j : i;
            TEST_ASSERT_EQUAL(j, fifo_size(&fifo));
            size_t count = fifo_push(&fifo, buffer + j, amount);
            TEST_ASSERT_EQUAL(amount, count);
            TEST_ASSERT_EQUAL(j+amount, fifo_size(&fifo));
        }

        // loop with decrement [buffer_size, 1]
        for (int j = buffer_size; j > 0; j -= i) {
            size_t amount = (j - i < 0) ? j : i;
            TEST_ASSERT_EQUAL(j, fifo_size(&fifo));
            size_t count = fifo_pop(&fifo, NULL, amount);
            TEST_ASSERT_EQUAL(amount, count);
            TEST_ASSERT_EQUAL(j-amount, fifo_size(&fifo));
        }
    }

    // loop with increment [0,buffer_size-1]
    for (int i = 0; i < buffer_size; ++i) {
        TEST_ASSERT_EQUAL(i, fifo_size(&fifo));
        fifo_push(&fifo, buffer, 1);
    }

    TEST_ASSERT_EQUAL(buffer_size, fifo_size(&fifo));

    // loop with decrement [buffer_size-1, 0]
    for (int i = buffer_size - 1; i >= 0; --i) {
        fifo_pop(&fifo, buffer, 1);
        TEST_ASSERT_EQUAL(i, fifo_size(&fifo));
    }
}

void test_fifo_capacity(void) {
    const size_t buffer_size = 32;
    FIFO_DEF(fifo, buffer_size);

    uint8_t buffer[buffer_size];

    for (int i = 0; i < buffer_size; ++i) {
        TEST_ASSERT_EQUAL(buffer_size, fifo_capacity(&fifo));
        fifo_push(&fifo, buffer, 1);
    }

    for (int i = 0; i < buffer_size; ++i) {
        TEST_ASSERT_EQUAL(buffer_size, fifo_capacity(&fifo));
        fifo_pop(&fifo, buffer, 1);
    }
}

void test_fifo_push(void) {
    const size_t buffer_size = 32;
    FIFO_DEF(fifo, buffer_size);

    uint8_t buffer[buffer_size];
    for (int i = 0; i < buffer_size; ++i) { buffer[i] = i; }

    for (size_t i = 1; i <= buffer_size; ++i) {
        fifo_clear(&fifo);
        memset((void*)fifo.buffer, 0, fifo.capacity);
        for (size_t j = 0; j < buffer_size; j += i) {
            size_t to_add = i;
            if (j + to_add > buffer_size) { to_add = buffer_size - j; }
            size_t count = fifo_push(&fifo, buffer + j, to_add);
            TEST_ASSERT_EQUAL(to_add, count);
        }
        TEST_ASSERT_EQUAL_MEMORY(buffer, fifo.buffer, buffer_size);
    }
}

void test_fifo_pop(void) {
    const size_t buffer_size = 32;
    FIFO_DEF(fifo, buffer_size);

    uint8_t original[buffer_size];
    for (int i = 0; i < buffer_size; ++i) { original[i] = i; }

    for (size_t i = 1; i <= buffer_size; ++i) {
        size_t count = fifo_push(&fifo, original, buffer_size);
        TEST_ASSERT_EQUAL(buffer_size, count);
        TEST_ASSERT_EQUAL(buffer_size, fifo_size(&fifo));

        uint8_t buffer[buffer_size];
        memset(buffer, 0, buffer_size);
        
        for (size_t j = 0; j < buffer_size; j += i) {
            size_t amount = (j + i > buffer_size) ? buffer_size - j : i;
            count = fifo_pop(&fifo, buffer + j, amount);
            TEST_ASSERT_EQUAL(amount, count);
        }
        TEST_ASSERT_EQUAL_MEMORY(original, buffer, buffer_size);
    }
}

void test_fifo_wrapping(void) {
    const size_t buffer_size = 32;
    FIFO_DEF(fifo, buffer_size);

    uint8_t buffer[buffer_size];
    size_t count;

    for (size_t i = 1; i >= buffer_size; ++i) {
        uint8_t buffer[buffer_size];
        
        for (size_t j = 0; j < buffer_size; ++j) {
            size_t size = i;
            if (j + size > buffer_size) { size = buffer_size - j; }
            count = fifo_push(&fifo, buffer, size);
            TEST_ASSERT_EQUAL(size, count);
            count = fifo_pop(&fifo, NULL, size);
            TEST_ASSERT_EQUAL(size, count);
        }
    }

    fifo_clear(&fifo);

    count = fifo_push(&fifo, buffer, buffer_size);
    TEST_ASSERT_EQUAL(buffer_size, count);
    TEST_ASSERT_EQUAL(buffer_size, fifo_size(&fifo));

    count = fifo_pop(&fifo, NULL, buffer_size / 2);
    TEST_ASSERT_EQUAL(buffer_size/2, count);
    TEST_ASSERT_EQUAL(buffer_size/2, fifo_size(&fifo));

    count = fifo_push(&fifo, buffer, buffer_size / 2);
    TEST_ASSERT_EQUAL(buffer_size/2, count);
    TEST_ASSERT_EQUAL(buffer_size, fifo_size(&fifo));
}

int main() {
    #ifndef NATIVE
    HAL_Init();
    HAL_Delay(2000);
    #endif
    UNITY_BEGIN();
    RUN_TEST(test_fifo_def);
    RUN_TEST(test_fifo_init);
    RUN_TEST(test_fifo_clear);
    RUN_TEST(test_fifo_full);
    RUN_TEST(test_fifo_empty);
    RUN_TEST(test_fifo_size);
    RUN_TEST(test_fifo_capacity);
    RUN_TEST(test_fifo_push);
    RUN_TEST(test_fifo_pop);
    RUN_TEST(test_fifo_wrapping);
    UNITY_END();
    while (1) {}
}

#ifndef NATIVE
void SysTick_Handler(void) {
    HAL_IncTick();
    HAL_SYSTICK_IRQHandler();
}
#endif