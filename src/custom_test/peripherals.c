// This is a personal academic project. Dear PVS-Studio, please check it.
// PVS-Studio Static Code Analyzer for C, C++ and C#: http://www.viva64.com


#ifndef USE_FULL_LL_DRIVER
#define USE_FULL_LL_DRIVER
#endif /* ifndef USE_FULL_LL_DRIVER */

#include "../../libs/stm32l0_low_level/stm32l0_ll/stm32l0xx_ll_bus.h"
#include "../../libs/stm32l0_low_level/stm32l0_ll/stm32l0xx_ll_rcc.h"
#include "../../libs/stm32l0_low_level/stm32l0_ll/stm32l0xx_ll_tim.h"
#include "../../libs/stm32l0_low_level/stm32l0_ll/stm32l0xx_ll_spi.h"
#include "../../libs/stm32l0_low_level/stm32l0_ll/stm32l0xx_ll_usart.h"
#include "../../libs/stm32l0_low_level/stm32l0_ll/stm32l0xx_ll_utils.h"

#include "../../libs/stm32l0_low_level/stm32l0_ll/stm32l0xx_hal_cortex.h"

#include "peripherals.h"

typedef struct pll_t {
    LL_UTILS_PLLInitTypeDef prescale;
    LL_UTILS_ClkInitTypeDef bus;
} pll_t;

pll_t pll_init = {
    .prescale = {.PLLMul = LL_RCC_PLL_MUL_4, .PLLDiv = LL_RCC_PLL_DIV_2
    },
    .bus = {.AHBCLKDivider = LL_RCC_SYSCLK_DIV_1,
            .APB1CLKDivider = LL_RCC_APB1_DIV_1,
            .APB2CLKDivider = LL_RCC_APB2_DIV_1,
    }
};

typedef struct usart_t {
  LL_USART_InitTypeDef init;
  LL_USART_ClockInitTypeDef clock;
} usart_t;

usart_t usart_init;

LL_TIM_InitTypeDef tim_init;

LL_SPI_InitTypeDef spi_init;

void config_system_clocks(void) {
    // set up clocks - OSC_IN and OSC_OUT on PH0 and PH1,
    // HSE input clock is 14.7456 MHz
    // set PLL as system clock, using HSE as source
    /*LL_PLL_ConfigSystemClock_HSE(HSE_VALUE, LL_UTILS_HSEBYPASS_OFF, &pll_init.prescale, &pll_init.bus);*/
    /*LL_SetSystemCoreClock(HSE_VALUE);*/

    LL_PLL_ConfigSystemClock_HSI(&pll_init.prescale, &pll_init.bus);
    /*LL_Init1msTick(SystemCoreClock);*/

    // enable peripheral clocks
    LL_IOP_GRP1_EnableClock(LL_IOP_GRP1_PERIPH_GPIOA |
                            LL_IOP_GRP1_PERIPH_GPIOB |
                            LL_IOP_GRP1_PERIPH_GPIOC);

    LL_APB2_GRP1_EnableClock(LL_APB2_GRP1_PERIPH_USART1);
    LL_APB2_GRP1_EnableClock(LL_APB2_GRP1_PERIPH_SPI1);

    LL_APB1_GRP1_EnableClock(LL_APB1_GRP1_PERIPH_USART2 |
                             LL_APB1_GRP1_PERIPH_TIM2);
}

void config_gpio(void) {
    // set up peripherals - GPIO for control/output:
    // | Pin  | Pin Name        | Description
    // | ---- | --------------- | -----------------------------------------
    // | PB0  | CC1125_Tx_or_Rx | Switches Rx and Tx chain (0 = Rx, 1 = Tx)
    // | PB4  | CANSAT_Enable   | Enables power switch to CANSAT
    LL_GPIO_SetPinMode(GPIOB, LL_GPIO_PIN_0, LL_GPIO_MODE_OUTPUT);
    LL_GPIO_SetPinOutputType(GPIOB, LL_GPIO_PIN_0, LL_GPIO_OUTPUT_PUSHPULL);
    LL_GPIO_SetPinPull(GPIOB, LL_GPIO_PIN_0, LL_GPIO_PULL_DOWN);

    LL_GPIO_SetPinMode(GPIOB, LL_GPIO_PIN_4, LL_GPIO_MODE_OUTPUT);
    LL_GPIO_SetPinOutputType(GPIOB, LL_GPIO_PIN_4, LL_GPIO_OUTPUT_PUSHPULL);
    LL_GPIO_SetPinPull(GPIOB, LL_GPIO_PIN_4, LL_GPIO_PULL_DOWN);

    // | Pin  | Pin Name        | Description
    // | ---- | --------------- | -----------------------------------------
    // | PA1  | CC1125_Enable   | Enables power switch to CC1125
    // | PA15 | PA_Enable       | Enables (logic to the gate of) power amplifier
    // | PA8  | N2420_Enable    | Enables power switch to N2420
    LL_GPIO_SetPinMode(GPIOA, LL_GPIO_PIN_1,  LL_GPIO_MODE_OUTPUT);
    LL_GPIO_SetPinOutputType(GPIOA, LL_GPIO_PIN_1, LL_GPIO_OUTPUT_PUSHPULL);
    LL_GPIO_SetPinPull(GPIOA, LL_GPIO_PIN_1, LL_GPIO_PULL_DOWN);

    LL_GPIO_SetPinMode(GPIOA, LL_GPIO_PIN_15, LL_GPIO_MODE_OUTPUT);
    LL_GPIO_SetPinOutputType(GPIOA, LL_GPIO_PIN_15, LL_GPIO_OUTPUT_PUSHPULL);
    LL_GPIO_SetPinPull(GPIOA, LL_GPIO_PIN_15, LL_GPIO_PULL_DOWN);

    LL_GPIO_SetPinMode(GPIOA, LL_GPIO_PIN_8,  LL_GPIO_MODE_OUTPUT);
    LL_GPIO_SetPinOutputType(GPIOA, LL_GPIO_PIN_8, LL_GPIO_OUTPUT_PUSHPULL);
    LL_GPIO_SetPinPull(GPIOA, LL_GPIO_PIN_8, LL_GPIO_PULL_DOWN);

    // | Pin  | Pin Name        | Description
    // | ---- | --------------- | -----------------------------------------
    // | PC13 | MCU_Alive       | Connected as current source to heartbeat LED
    LL_GPIO_SetPinMode(GPIOC, LL_GPIO_PIN_13, LL_GPIO_MODE_OUTPUT);
    LL_GPIO_SetPinOutputType(GPIOC, LL_GPIO_PIN_13, LL_GPIO_OUTPUT_PUSHPULL);

    /*   - UART for CANSAT - USART2 on PA2/3*/
    LL_GPIO_SetPinMode(GPIOA, LL_GPIO_PIN_2, LL_GPIO_MODE_ALTERNATE);
    LL_GPIO_SetPinMode(GPIOA, LL_GPIO_PIN_3, LL_GPIO_MODE_ALTERNATE);

    LL_GPIO_SetAFPin_0_7(GPIOA, LL_GPIO_PIN_2, LL_GPIO_AF_4);  // USART2_TX
    LL_GPIO_SetAFPin_0_7(GPIOA, LL_GPIO_PIN_3, LL_GPIO_AF_4);  // USART2_RX

    /*   - UART for n2420 - USART1 on PA9/10*/
    LL_GPIO_SetPinMode(GPIOA, LL_GPIO_PIN_9,  LL_GPIO_MODE_ALTERNATE);
    LL_GPIO_SetPinMode(GPIOA, LL_GPIO_PIN_10, LL_GPIO_MODE_ALTERNATE);

    LL_GPIO_SetAFPin_8_15(GPIOA, LL_GPIO_PIN_9,  LL_GPIO_AF_4);  // USART1_TX
    LL_GPIO_SetAFPin_8_15(GPIOA, LL_GPIO_PIN_10, LL_GPIO_AF_4);  // USART1_RX

    /* turn off CANSAT and CC1125 */
    LL_GPIO_ResetOutputPin(GPIOA, LL_GPIO_PIN_0);
    LL_GPIO_ResetOutputPin(GPIOB, LL_GPIO_PIN_4);
}

void config_uart(void) {
    /* USART configuration */
    LL_USART_StructInit(&usart_init.init);
    LL_USART_ClockStructInit(&usart_init.clock);

    LL_USART_ClockInit(USART1, &usart_init.clock);
    LL_USART_Init(USART1, &usart_init.init);
    LL_USART_Enable(USART1);

    HAL_NVIC_EnableIRQ(USART1_IRQn);
    HAL_NVIC_SetPriority(USART1_IRQn, 0, 0);
    // enable rx interrupt
    LL_USART_EnableIT_RXNE(USART1);
}

void config_tim2_nvic(void) {
    HAL_NVIC_EnableIRQ(TIM2_IRQn);
    HAL_NVIC_SetPriority(TIM2_IRQn, 0, 0);

    LL_TIM_StructInit(&tim_init);
    LL_TIM_Init(TIM2, &tim_init);
    LL_TIM_EnableCounter(TIM2);
    LL_TIM_SetPrescaler(TIM2, 65535);
    LL_TIM_SetClockDivision(TIM2, LL_TIM_CLOCKDIVISION_DIV4);
    LL_TIM_SetAutoReload(TIM2, 100);

    LL_TIM_EnableIT_UPDATE(TIM2);
}


void config_spi(void) {
  // set up peripherals - alternate pin functions are on page 45/136
  // of DM00141136 - STM32L071x8 datasheet
  //   - SPI for CC1125 - SPI1 on PA4/5/6/7
  /* LL_GPIO_SetPinMode(GPIOA, LL_GPIO_PIN_4, LL_GPIO_MODE_ALTERNATE); */
  /* LL_GPIO_SetPinMode(GPIOA, LL_GPIO_PIN_5, LL_GPIO_MODE_ALTERNATE); */
  /* LL_GPIO_SetPinMode(GPIOA, LL_GPIO_PIN_6, LL_GPIO_MODE_ALTERNATE); */
  /* LL_GPIO_SetPinMode(GPIOA, LL_GPIO_PIN_7, LL_GPIO_MODE_ALTERNATE); */

  LL_GPIO_SetPinMode(GPIOA, LL_GPIO_PIN_4, LL_GPIO_MODE_OUTPUT);
  LL_GPIO_SetPinMode(GPIOA, LL_GPIO_PIN_5, LL_GPIO_MODE_OUTPUT);
  LL_GPIO_SetPinMode(GPIOA, LL_GPIO_PIN_6, LL_GPIO_MODE_OUTPUT);
  LL_GPIO_SetPinMode(GPIOA, LL_GPIO_PIN_7, LL_GPIO_MODE_OUTPUT);

  LL_GPIO_SetPinOutputType(GPIOA, LL_GPIO_PIN_4, LL_GPIO_OUTPUT_PUSHPULL);
  LL_GPIO_SetPinOutputType(GPIOA, LL_GPIO_PIN_5, LL_GPIO_OUTPUT_PUSHPULL);
  LL_GPIO_SetPinOutputType(GPIOA, LL_GPIO_PIN_6, LL_GPIO_OUTPUT_PUSHPULL);
  LL_GPIO_SetPinOutputType(GPIOA, LL_GPIO_PIN_7, LL_GPIO_OUTPUT_PUSHPULL);

  /* LL_GPIO_SetAFPin_0_7(GPIOA, LL_GPIO_PIN_4, LL_GPIO_AF_0);  // SPI1_NSS */
  /* LL_GPIO_SetAFPin_0_7(GPIOA, LL_GPIO_PIN_5, LL_GPIO_AF_0);  // SPI1_SCK */
  /* LL_GPIO_SetAFPin_0_7(GPIOA, LL_GPIO_PIN_6, LL_GPIO_AF_0);  // SPI1_MISO */
  /* LL_GPIO_SetAFPin_0_7(GPIOA, LL_GPIO_PIN_7, LL_GPIO_AF_0);  // SPI1_MOSI */

  /* LL_SPI_StructInit(&spi_init); */
  /* LL_SPI_Init(SPI1, &spi_init); */

  /* LL_SPI_Enable(SPI1); */

  /* LL_SPI_SetStandard(SPI1, LL_SPI_PROTOCOL_MOTOROLA); */
  /* LL_SPI_SetTransferDirection(SPI1, LL_SPI_FULL_DUPLEX); */
  /* LL_SPI_SetNSSMode(SPI1, LL_SPI_NSS_HARD_OUTPUT); */

  /* LL_TIM_EnableIT_UPDATE(TIM2); */
}
