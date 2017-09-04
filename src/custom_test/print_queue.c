// This is a personal academic project. Dear PVS-Studio, please check it.
// PVS-Studio Static Code Analyzer for C, C++ and C#: http://www.viva64.com


// character queue for buffering transmits over UART
// TODO: change buffer length to reasonable, 'justified' size

#include <stdbool.h>

#include "print_queue.h"

#define BUFFER_LEN 512

char char_buf[BUFFER_LEN] = {0};
int head = 0;
int tail = 0;
int n_elements = 0;

void pq_add_char(char c) {
    if (n_elements < BUFFER_LEN) {
        char_buf[tail++] = c;
        tail %= BUFFER_LEN;
        n_elements++;
    }
}

bool pq_is_empty(void){
  return n_elements == 0;
    /* return head == tail; */
}

char pq_rem_char(void) {
    if (n_elements > 0) {
        char c = char_buf[head];
        char_buf[head++] = 0;
        head %= BUFFER_LEN;
        n_elements--;
        return c;
    }

    return '@';
}
