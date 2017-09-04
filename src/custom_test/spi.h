
#include <stdint.h>
#include "../../libs/stm32l0_low_level/stm32l0_ll/stm32l0xx_ll_spi.h"

uint8_t spi_write_byte(uint8_t data);
uint8_t spi_read_byte(void);

void spi_ss_low(void);
void spi_ss_high(void);
