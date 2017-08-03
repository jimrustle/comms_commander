
#include "spi.h"
#include "printf.h"

void spi_write_byte(uint8_t data) {
  printf("in spi_write_byte: %x\r\n", data);
  while (LL_SPI_IsActiveFlag_BSY(SPI1));
  LL_SPI_TransmitData8(SPI1, data);
  while (!(LL_SPI_IsActiveFlag_TXE(SPI1)));
}

void spi_write_byte_array(uint8_t* data) {
  while (*data) {
      spi_write_byte(*data++);
    }
}

uint8_t spi_read_byte(void) {
  printf("in spi_read_byte\r\n");
  while (!LL_SPI_IsActiveFlag_RXNE(SPI1));
  return LL_SPI_ReceiveData8(SPI1);
}
