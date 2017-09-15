// This is a personal academic project. Dear PVS-Studio, please check it.
// PVS-Studio Static Code Analyzer for C, C++ and C#: http://www.viva64.com

#include "stm32l0xx.h"
#include <stdint.h>

extern int main(void);
extern void _exit(int code);

extern unsigned int __textdata__;
extern unsigned int __data_start__;
extern unsigned int __data_end__;
extern unsigned int __bss_start__;
extern unsigned int __bss_end__;

static void __attribute__((always_inline)) __initialize_data(unsigned int* from, unsigned int* region_begin, unsigned int* region_end);

static void __attribute__((always_inline)) __initialize_bss(unsigned int* region_begin, unsigned int* region_end);

extern void __libc_init_array(void);

void __attribute__((section(".after_vectors"), noreturn, used)) _start(void);

void _start(void)
{
    __initialize_data(&__textdata__, &__data_start__, &__data_end__);
    __initialize_bss(&__bss_start__, &__bss_end__);

    /*__libc_init_array();*/

    SystemInit();

    int code = main();

    _exit(code);
    while (1)
        ;
}

static inline void __initialize_data(unsigned int* from, unsigned int* region_begin, unsigned int* region_end)
{
    unsigned int* p = region_begin;
    while (p < region_end)
        *p++ = *from++;
}

static inline void __initialize_bss(unsigned int* region_begin, unsigned int* region_end)
{
    unsigned int* p = region_begin;
    while (p < region_end)
        *p++ = 0;
}

void __attribute__((noreturn, used)) _exit(int code)
{
    (void)code;

    NVIC_SystemReset();

    while (1)
        ;
}
