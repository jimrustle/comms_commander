// This is a personal academic project. Dear PVS-Studio, please check it.
// PVS-Studio Static Code Analyzer for C, C++ and C#: http://www.viva64.com

#include "spi.h"
#include "peripherals.h"

uint8_t spi_write_byte(uint8_t data) {
  uint8_t status = 0;
  // SS = PA4
  // SCK = PA5
  // MISO = PA6
  // MOSI = PA7

  /* // SS low */
  /* LL_GPIO_ResetOutputPin(GPIOA, LL_GPIO_PIN_4); */

  for (int i = 7; i >= 0; i--) {
    // SCK low
    LL_GPIO_ResetOutputPin(GPIOA, LL_GPIO_PIN_5);

    if (data >> i & 1) {
      LL_GPIO_SetOutputPin(GPIOA, LL_GPIO_PIN_7);
    } else {
      LL_GPIO_ResetOutputPin(GPIOA, LL_GPIO_PIN_7);
    }

    // delay
    for (volatile int j = 0; j < 600; j++);;

    // SCK high
    LL_GPIO_SetOutputPin(GPIOA, LL_GPIO_PIN_5);

    status = (status << 1) | LL_GPIO_IsInputPinSet(GPIOA, LL_GPIO_PIN_6);

    // delay
    for (volatile int j = 0; j < 600; j++);;
  }
  status = status | LL_GPIO_IsInputPinSet(GPIOA, LL_GPIO_PIN_6);
  /* // SS high */
  /* LL_GPIO_SetOutputPin(GPIOA, LL_GPIO_PIN_4); */

  /* while (!LL_SPI_IsActiveFlag_TXE(SPI1)); */
  /* LL_SPI_TransmitData8(SPI1, data); */
  /* while (!LL_SPI_IsActiveFlag_BSY(SPI1)); */
  /* LL_SPI_ClearFlag_OVR(SPI1); */

  return status;
}

uint8_t spi_read_byte(void) {
  // SS = PA4
  // SCK = PA5
  // MISO = PA6
  // MOSI = PA7
  uint8_t ret = 0;

  /* // SS low */
  /* LL_GPIO_ResetOutputPin(GPIOA, LL_GPIO_PIN_4); */

  for (int i = 0; i < 8; i++) {
    // SCK low
    LL_GPIO_ResetOutputPin(GPIOA, LL_GPIO_PIN_5);

    ret = (ret | LL_GPIO_IsInputPinSet(GPIOA, LL_GPIO_PIN_6)) << 1;

    // delay
    for (volatile int j = 0; j < 600; j++);;
    // SCK high
    LL_GPIO_SetOutputPin(GPIOA, LL_GPIO_PIN_5);

    // delay
    for (volatile int j = 0; j < 600; j++);;
  }
  ret = ret | LL_GPIO_IsInputPinSet(GPIOA, LL_GPIO_PIN_6);
  /* LL_GPIO_SetOutputPin(GPIOA, LL_GPIO_PIN_4); */

  /* while (LL_SPI_IsActiveFlag_BSY(SPI1)); */
  /* while (!LL_SPI_IsActiveFlag_RXNE(SPI1)); */
  /* return LL_SPI_ReceiveData8(SPI1); */
  return ret;
}

void spi_ss_low(void) {
  LL_GPIO_ResetOutputPin(GPIOA, LL_GPIO_PIN_4);
}

void spi_ss_high(void) {
  LL_GPIO_SetOutputPin(GPIOA, LL_GPIO_PIN_4);
}
