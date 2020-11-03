#pragma once
#include <stddef.h>
#include <stdint.h>
#include <stdbool.h>

typedef struct {
    volatile uint8_t* const buffer;
    volatile size_t head;
    volatile size_t tail;
    size_t capacity;
} fifo_t;

/**
 * Defines and allocates a fifo structure on the stack
 * \param x name of fifo buffer
 * \param y capacity of fifo buffer
 */
#define FIFO_DEF(x, y)              \
    uint8_t x##_data_space[y+1];    \
    fifo_t x = {                    \
        .buffer = x##_data_space,   \
        .head = 0,                  \
        .tail = 0,                  \
        .capacity = y               \
    }

/**
 * Creates a fifo structure with a pre-allocated buffer.
 * \warning capacity of fifo will be `size-1`
 * \param buffer pre-allocated buffer to be assigned to fifo
 * \param size size of buffer in bytes
 * \returns a fifo structure for use with the fifo api
 */
fifo_t fifo_init(uint8_t* buffer, size_t size);

/**
 * Resets an existing fifo to be empty.
 * \warning leaves memory contents untouched
 * \param fifo handle to existing fifo structure
 */
void fifo_clear(volatile fifo_t* fifo);

/**
 * \param fifo handle to existing fifo structure
 * \returns capacity of fifo buffer
 */
size_t fifo_capacity(const volatile fifo_t* fifo);

/**
 * \param fifo handle to existing fifo structure
 * \returns current size of fifo buffer
 */
size_t fifo_size(const volatile fifo_t* fifo);

/**
 * \param fifo handle to existing fifo structure
 * \returns true if the fifo is full, false otherwise
 */
bool fifo_full(const volatile fifo_t* fifo);

/**
 * \param fifo handle to existing fifo structure
 * \returns true if the fifo is empty, false otherwise
 */
bool fifo_empty(const volatile fifo_t* fifo);

/**
 * Pushs elements on the back of fifo
 * \param fifo handle to existing fifo structure
 * \param data buffer of elements to add
 * \param count number of elements to add
 * \returns number of elements added, can be less than \p count
 */
size_t fifo_push(volatile fifo_t* fifo, const uint8_t* data, size_t count);

/**
 * Pops elements from the front of fifo
 * \param fifo handle to existing fifo structure
 * \param data buffer to store popped elements
 * \param count number of elements to pop
 * \returns number of elements popped, can be less than \p count
 */
size_t fifo_pop(volatile fifo_t* fifo, uint8_t* data, size_t count);
