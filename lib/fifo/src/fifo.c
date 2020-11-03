#include <fifo.h>

static size_t _fifo_advance(const volatile fifo_t* fifo, size_t idx) {
    size_t next = idx + 1;
    if (next > fifo->capacity) { next = 0; }
    return next;
}

fifo_t fifo_init(uint8_t* buffer, size_t size) {
    fifo_t fifo = {
        .buffer = buffer,
        .head = 0,
        .tail = 0,
        .capacity = size - 1
    };
    return fifo;
}

void fifo_clear(volatile fifo_t* fifo) {
    fifo->head = fifo->tail = 0;
}

size_t fifo_capacity(const volatile fifo_t* fifo) {
    return fifo->capacity;
}

size_t fifo_size(const volatile fifo_t* fifo) {
    if (fifo->head >= fifo->tail) {
        return fifo->head - fifo->tail;
    } else {
        return fifo->head + (fifo->capacity - fifo->tail + 1);
    }
}

bool fifo_full(const volatile fifo_t* fifo) {
    return _fifo_advance(fifo, fifo->head) == fifo->tail;
}

bool fifo_empty(const volatile fifo_t* fifo) {
    return fifo->head == fifo->tail;
}

size_t fifo_push(volatile fifo_t* fifo, const uint8_t* data, size_t count) {
    for (size_t i = 0; i < count; ++i) {
        if (fifo_full(fifo)) { return i; }
        fifo->buffer[fifo->head] = data[i];
        fifo->head = _fifo_advance(fifo, fifo->head);
    }
    return count;
}

size_t fifo_pop(volatile fifo_t* fifo, uint8_t* data, size_t count) {
    for (size_t i = 0; i < count; ++i) {
        if (fifo_empty(fifo)) return i;
        if (data != NULL) {
            data[i] = fifo->buffer[fifo->tail];
        }
        fifo->tail = _fifo_advance(fifo, fifo->tail);
    }
    return count;
}