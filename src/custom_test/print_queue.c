
// character queue for buffering transmits over UART
// TODO: change buffer length to reasonable, 'justified' size

#include <stdbool.h>

#include "print_queue.h"

#define BUFFER_LEN 80

char char_buf[BUFFER_LEN] = {0};
int head = 0;
int tail = 1;

void pq_add_char(char c) {
  char_buf[tail++] = c;
  tail %= BUFFER_LEN;
}

bool pq_is_empty(void){
  return head == tail;
}

char pq_rem_char(void) {
  char c = char_buf[head];
  char_buf[head] = '|';

  head++;
  head %= BUFFER_LEN;

  return c;
}
