// This is a personal academic project. Dear PVS-Studio, please check it.
// PVS-Studio Static Code Analyzer for C, C++ and C#: http://www.viva64.com

#include <stdbool.h>
#include <stdnoreturn.h>

#ifndef USE_FULL_LL_DRIVER
#define USE_FULL_LL_DRIVER
#endif /* ifndef USE_FULL_LL_DRIVER */

#include "../../libs/stm32l0_low_level/stm32l0_ll/stm32l071xx.h"
#include "../../libs/stm32l0_low_level/stm32l0_ll/stm32l0xx_ll_system.h"
#include "../../libs/stm32l0_low_level/stm32l0_ll/stm32l0xx_ll_tim.h"
#include "../../libs/stm32l0_low_level/stm32l0_ll/stm32l0xx_ll_rcc.h"
#include "../../libs/stm32l0_low_level/stm32l0_ll/stm32l0xx_ll_gpio.h"
#include "../../libs/stm32l0_low_level/stm32l0_ll/stm32l0xx_ll_usart.h"
#include "../../libs/stm32l0_low_level/stm32l0_ll/stm32l0xx_ll_spi.h"
#include "../../libs/stm32l0_low_level/stm32l0_ll/stm32l0xx_ll_utils.h"

#include "command.h"
#include "peripherals.h"
#include "print_queue.h"
#include "radio.h"
#include "radio_cc1125.h"
#include "log.h"
#include "spi.h"

volatile uint8_t delay = 0;
static queue_t usart1_queue;
static queue_t usart2_queue;

// error_catch serves as a debugging function that will catch all failed
// asserts and trap execution for the debugger
noreturn void error_catch(void);
noreturn void error_catch(void) {
  __disable_irq();
  // constant LED
  LL_GPIO_SetOutputPin(GPIOC, LL_GPIO_PIN_13);

  while (1) {
    // if you can read this line in the debugger, you've made a terrible mistake
    __asm("BKPT");
  }
}

int main(void) {
  // enable prefetch
  // 2018-08-04 FIXME: explain why
  LL_FLASH_EnablePrefetch();

  config_system_clocks();
  config_gpio();
  config_uart();
  config_spi();
  config_tim2_nvic();

  queue_init(&usart1_queue);
  queue_init(&usart2_queue);

  radio_CC1125_power_on();
  radio_CANSAT_power_on();
  radio_set_mode(RM_TRANSMIT);

  LL_TIM_EnableIT_UPDATE(TIM2);

  // enable rx interrupt for USART1
  LL_USART_EnableIT_RXNE(USART1);

  while (1) {
    // goodnight sweet prince
    __WFI();
  }

  return 0;
}

// 2018-04-08 FIXME: implement DMA for UART transmission
// putchar loads a circular buffer with characters to transmit (print_queue.c)
void putchar(usart_num u, char c);
void putchar(usart_num u, char c) {
  if (u == USART_1) {
    queue_add_char(&usart1_queue, c);
    LL_USART_EnableIT_TXE(USART1);
  } else {
    queue_add_char(&usart2_queue, c);
    LL_USART_EnableIT_TXE(USART2);
  }
}

// interrupt handlers
void TIM2_IRQHandler(void);
void TIM2_IRQHandler() {
  __disable_irq();
  LL_TIM_ClearFlag_UPDATE(TIM2);
  delay++;
  if (delay > 10) {
    delay = 0;
  }

  radio_LED_toggle();
  /* radio_CANSAT_test(); */
  /* pr_hex(USART_1, delay); */
  /* pr_str(USART_1, " Shi\r\n"); */
  __enable_irq();
}

// 2017-08-07 FIXME: there is a race condition in the state machines of the USART1
// and USART2 interrupt handlers
// bonus points to [$favourite_harry_potter_house] if you can fix it
// (or just use the USART DMA and hope the problem goes away lol)
void USART1_IRQHandler(void);
void USART1_IRQHandler() {
  __disable_irq();
  // receive
  if (LL_USART_IsActiveFlag_RXNE(USART1)) {
    // RXNE flag is cleared by reading from the USART1 data register
    char c = LL_USART_ReceiveData8(USART1);
    add_c(c);

    // echo char to serial
    if (c == '\r') {
      queue_add_char(&usart1_queue, '\r');
      queue_add_char(&usart1_queue, '\n');
    }
    else {
      pr_ch(USART_1, c);
    }
  }

  // transmit
  if (LL_USART_IsEnabledIT_TXE(USART1) && LL_USART_IsActiveFlag_TXE(USART1)) {
    if (queue_is_empty(&usart1_queue)) {
      LL_USART_DisableIT_TXE(USART1);
    } else {
      // TXE flag cleared by writing to USART1 data register
      LL_USART_TransmitData8(USART1, queue_rem_char(&usart1_queue));
      LL_USART_EnableIT_TXE(USART1);
    }
  } else {
    LL_USART_DisableIT_TXE(USART1);
  }
  __enable_irq();
}

void USART2_IRQHandler(void);
void USART2_IRQHandler() {
  __disable_irq();
  // usart2 doesn't receive anything from the Stensat CANSAT

  // transmit
  if (LL_USART_IsEnabledIT_TXE(USART2) && LL_USART_IsActiveFlag_TXE(USART2)) {
    if (queue_is_empty(&usart2_queue)) {
      LL_USART_DisableIT_TXE(USART2);
    } else {
      // TXE flag cleared by writing to USART2 data register
      LL_USART_TransmitData8(USART2, queue_rem_char(&usart2_queue));
      LL_USART_EnableIT_TXE(USART2);
    }
  } else {
    LL_USART_DisableIT_TXE(USART2);
  }
  __enable_irq();
}

void SPI1_IRQHandler(void);
void SPI1_IRQHandler(void) {
  // if transmitting, then send data
  if (LL_SPI_IsActiveFlag_TXE(SPI1)){
    LL_SPI_TransmitData8(SPI1, spi_tx_byte);
    LL_SPI_DisableIT_TXE(SPI1);
  }

  // otherwise, get data
  /* if (LL_SPI_IsActiveFlag_RXNE(SPI1)) { */
  /*   spi_rx_byte = LL_SPI_ReceiveData8(SPI1); */
    /* pr_hex(USART_1, c); pr_nl(USART_1); */
  /* } */
}

// fstack-protector
uintptr_t __stack_chk_guard = 0xdeadbeef;

void __stack_chk_fail(void);
void __stack_chk_fail(void) {
  __disable_irq();
  /* printf("Stack smashing detected\0"); */
  error_catch();
}