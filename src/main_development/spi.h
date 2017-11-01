
#include "../../libs/stm32l0_low_level/stm32l0_ll/stm32l0xx_ll_spi.h"
#include <stdint.h>
#include <stdbool.h>

//#define SPI_BITBANG 

extern volatile uint8_t spi_tx_byte;
extern volatile uint8_t spi_rx_byte;
extern volatile bool spi_transfer_completed;

#ifdef SPI_BITBANG
uint8_t spi_read_write_byte(uint8_t data);
#else
// if using hardware SPI
void spi_write_byte(uint8_t data);
uint8_t spi_read_byte(void);
#endif

void spi_ss_low(void);
void spi_ss_high(void);
