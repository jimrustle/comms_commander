// This is a personal academic project. Dear PVS-Studio, please check it.
// PVS-Studio Static Code Analyzer for C, C++ and C#: http://www.viva64.com

/* 2017-08-04 FIXME: rewrite spi driver to use hardware SPI instead of bitbanging */

#include "spi.h"
#include "log.h"
#include "peripherals.h"
#include "print_queue.h"

extern queue_t spi1_tx_queue;
extern queue_t spi1_rx_queue;

uint8_t spi_read_write_byte(uint8_t data)
{
    uint8_t status = 0;
#ifdef SPI_BITBANG
    // SS = PA4
    // SCK = PA5
    // MISO = PA6
    // MOSI = PA7

    for (int i = 7; i >= 0; i--) {
        // SCK low
        LL_GPIO_ResetOutputPin(GPIOA, LL_GPIO_PIN_5);

        if (data >> i & 1) {
            LL_GPIO_SetOutputPin(GPIOA, LL_GPIO_PIN_7);
        } else {
            LL_GPIO_ResetOutputPin(GPIOA, LL_GPIO_PIN_7);
        }

        // delay
        for (volatile int j = 0; j < 100; j++)
            ;
        ;

        // SCK high
        LL_GPIO_SetOutputPin(GPIOA, LL_GPIO_PIN_5);

        status = (status << 1) | LL_GPIO_IsInputPinSet(GPIOA, LL_GPIO_PIN_6);

        // delay
        for (volatile int j = 0; j < 100; j++)
            ;
        ;
    }
    status = status | LL_GPIO_IsInputPinSet(GPIOA, LL_GPIO_PIN_6);
#else
    spi_transfer_completed = false;
    queue_add_char(&spi1_tx_queue, data);
    LL_SPI_EnableIT_TXE(SPI1);

    while(!spi_transfer_completed);

    status = queue_rem_char(&spi1_rx_queue);
#endif

    return status;
}

void spi_ss_low(void)
{
    LL_GPIO_ResetOutputPin(GPIOA, LL_GPIO_PIN_4);
}

void spi_ss_high(void)
{
    LL_GPIO_SetOutputPin(GPIOA, LL_GPIO_PIN_4);
}
