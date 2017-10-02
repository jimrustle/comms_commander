// This is a personal academic project. Dear PVS-Studio, please check it.
// PVS-Studio Static Code Analyzer for C, C++ and C#: http://www.viva64.com

/* 2017-08-04 FIXME: rewrite spi driver to use hardware SPI instead of bitbanging */

#include "spi.h"
#include "log.h"
#include "peripherals.h"

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
    spi_tx_byte = data;
    LL_SPI_EnableIT_TXE(SPI1);

    status = spi_rx_byte;

#endif

    return status;
}

void spi_ss_low(void)
{
#ifdef SPI_BITBANG
    LL_GPIO_ResetOutputPin(GPIOA, LL_GPIO_PIN_4);
#endif
}

void spi_ss_high(void)
{
#ifdef SPI_BITBANG
    LL_GPIO_SetOutputPin(GPIOA, LL_GPIO_PIN_4);
#endif
}
