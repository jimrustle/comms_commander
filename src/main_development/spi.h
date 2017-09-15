
#include "../../libs/stm32l0_low_level/stm32l0_ll/stm32l0xx_ll_spi.h"
#include <stdint.h>

/* #define SPI_BITBANG */

extern uint8_t spi_tx_byte;
extern uint8_t spi_rx_byte;

uint8_t spi_read_write_byte(uint8_t data);

void spi_ss_low(void);
void spi_ss_high(void);
