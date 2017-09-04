// This is a personal academic project. Dear PVS-Studio, please check it.
// PVS-Studio Static Code Analyzer for C, C++ and C#: http://www.viva64.com

#include "log.h"

void pr_hex_digit(uint8_t h);

void pr_ch(char ch) {
  extern void putchar(char c);
  putchar(ch);
}

void pr_str(const char * str) {
  const char *c = str;
  while (*c) {
      pr_ch(*c++);
  }
}

// single-digit hex
void pr_hex_digit(uint8_t h) {
  // get lower nibble
  h &= 0xF;
  // if > 9, convert to letter in ASCII
  if (h > 9) h += 7;
  pr_ch(h + '0');
}

// two-digit hex
void pr_hex(uint8_t hex) {
  pr_hex_digit(hex >> 4);
  pr_hex_digit(hex);
}

void pr_nl(void) {
  pr_ch('\r');
  pr_ch('\n');
}
