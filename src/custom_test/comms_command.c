// This is a personal academic project. Dear PVS-Studio, please check it.
// PVS-Studio Static Code Analyzer for C, C++ and C#: http://www.viva64.com

#include <stdbool.h>

/*#define HSE_VALUE ((uint32_t) 14745600)*/
#define HSE_VALUE ((uint32_t) 8000000)

#ifndef USE_FULL_LL_DRIVER
#define USE_FULL_LL_DRIVER
#endif /* ifndef USE_FULL_LL_DRIVER */

#include "../../libs/stm32l0_low_level/stm32l0_ll/stm32l071xx.h"
#include "../../libs/stm32l0_low_level/stm32l0_ll/stm32l0xx_ll_system.h"
#include "../../libs/stm32l0_low_level/stm32l0_ll/stm32l0xx_ll_tim.h"
#include "../../libs/stm32l0_low_level/stm32l0_ll/stm32l0xx_ll_rcc.h"
#include "../../libs/stm32l0_low_level/stm32l0_ll/stm32l0xx_ll_gpio.h"
#include "../../libs/stm32l0_low_level/stm32l0_ll/stm32l0xx_ll_usart.h"
#include "../../libs/stm32l0_low_level/stm32l0_ll/stm32l0xx_ll_utils.h"

#include "command.h"
#include "peripherals.h"
#include "print_queue.h"
#include "radio.h"
#include "radio_cc1125.h"
#include "log.h"

volatile uint8_t delay = 0;

void error_catch(void);
void error_catch(void) {
  __disable_irq();
  // constant LED
  LL_GPIO_SetOutputPin(GPIOC, LL_GPIO_PIN_13);

  while (1) {
    __asm("nop");
  }
}

int main(void) {
  // enable prefetch
  // TODO: explain why
  LL_FLASH_EnablePrefetch();

  config_system_clocks();
  config_gpio();
  config_uart();
  config_tim2_nvic();
  config_spi();

  radio_CC1125_power_on();

  while (1) {
    //pr_str("hello world");
      // goodnight sweet prince
      __WFI();
  }

  return 0;
}

// TODO: implement DMA for UART transmission
// putchar loads a circular buffer with characters to transmit (print_queue.c)
void putchar(char c);
void putchar(char c) {
  pq_add_char(c);
  LL_USART_EnableIT_TXE(USART1);
}

// interrupt handlers
void TIM2_IRQHandler(void);
void TIM2_IRQHandler() {
  LL_TIM_ClearFlag_UPDATE(TIM2);
  delay++;
  if (delay > 10) {
    delay = 0;
  }
  LL_GPIO_TogglePin(GPIOC, LL_GPIO_PIN_13);
}

void USART1_IRQHandler(void);
void USART1_IRQHandler() {
  /* __disable_irq(); */
  // receive
  if (LL_USART_IsActiveFlag_RXNE(USART1)) {
    // RXNE flag is cleared by reading from the USART1 data register
    char c = LL_USART_ReceiveData8(USART1);
    add_c(c);

    // echo char to serial
    if (c == '\r') {
      pq_add_char('\r');
      pq_add_char('\n');
    }
    else {
      pr_ch(c);
    }
  }

  // transmit
  if (LL_USART_IsEnabledIT_TXE(USART1) && LL_USART_IsActiveFlag_TXE(USART1)) {
    if (pq_is_empty()) {
      LL_USART_DisableIT_TXE(USART1);
    } else {
      // TXE flag cleared by writing to USART1 data register
      LL_USART_TransmitData8(USART1, pq_rem_char());
    }
  } else {
    LL_USART_DisableIT_TXE(USART1);
  }
  /* __enable_irq(); */
 }

// fstack-protector
uintptr_t __stack_chk_guard = 0xdeadbeef;

void __stack_chk_fail(void);
void __stack_chk_fail(void) {
  __disable_irq();
  /* printf("Stack smashing detected\0"); */
  error_catch();
}
