// This is a personal academic project. Dear PVS-Studio, please check it.
// PVS-Studio Static Code Analyzer for C, C++ and C#: http://www.viva64.com

// character queue for buffering transmits over UART

#include <stdbool.h>
#include <stdint.h>

#include "print_queue.h"

void queue_init(queue_t* q)
{
    q->head = 0;
    q->tail = 0;
    q->n_elements = 0;

    for (uint8_t i = 0; i < BUFFER_LEN; i++) {
        q->char_buf[i] = 0;
    }
}

void queue_add_char(queue_t* q, char c)
{
    if (q->n_elements < BUFFER_LEN) {
        q->char_buf[q->tail++] = c;
        q->tail %= BUFFER_LEN;
        q->n_elements++;
    }
}

bool queue_is_empty(queue_t* q)
{
    return q->n_elements == 0;
}

char queue_rem_char(queue_t* q)
{
    if (q->n_elements > 0) {
        char c = q->char_buf[q->head];
        q->char_buf[q->head++] = 0;
        q->head %= BUFFER_LEN;
        q->n_elements--;
        return c;
    }

    return '@';
}
