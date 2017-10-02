// This is a personal academic project. Dear PVS-Studio, please check it.
// PVS-Studio Static Code Analyzer for C, C++ and C#: http://www.viva64.com

#include "log.h"

// single-digit hex
void pr_hex_digit(usart_num u, uint8_t h);
void pr_hex_digit(usart_num u, uint8_t h)
{
    // get lower nibble
    h &= 0xF;
    // if > 9, convert to letter in ASCII
    if (h > 9)
        h += 7;
    pr_ch(u, h + '0');
}

void pr_ch(usart_num u, char ch)
{
    extern void usart_putchar(usart_num u, char c);
    usart_putchar(u, ch);
}

void pr_str(usart_num u, const char* str)
{
    const char* c = str;
    while (*c) {
        pr_ch(u, *c++);
    }
}

// two-digit hex
void pr_hex(usart_num u, uint8_t hex)
{
    pr_hex_digit(u, hex >> 4);
    pr_hex_digit(u, hex);
}

void pr_nl(usart_num u)
{
    pr_ch(u, '\r');
    pr_ch(u, '\n');
}
