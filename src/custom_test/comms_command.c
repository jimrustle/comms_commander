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

#include "peripherals.h"
#include "printf.h"
#include "radio.h"
#include "command.h"

void error_catch(void);
void error_catch(void) {
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

    while (1) {
        // goodnight sweet prince
        __WFI();
    }

    return 0;
}

// TODO: implement DMA for UART transmission
// printf putc
void putchar(char c) {
    while (!LL_USART_IsActiveFlag_TC(USART1));
    LL_USART_TransmitData8(USART1, c);
}

// interrupt handlers

void TIM2_IRQHandler() {
    LL_TIM_ClearFlag_UPDATE(TIM2);
    /*LL_GPIO_TogglePin(GPIOC, LL_GPIO_PIN_13);*/
    /*printf("hello world!\r\n");*/
}

void USART1_IRQHandler() {
    if (LL_USART_IsActiveFlag_RXNE(USART1)) {
        /*LL_GPIO_TogglePin(GPIOC, LL_GPIO_PIN_13);*/

        char c = LL_USART_ReceiveData8(USART1);
        /*putchar(c);*/
        add_c(c);
    }
}
