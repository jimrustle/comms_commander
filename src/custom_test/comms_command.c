#include <stdbool.h>

#define HSE_VALUE (14745600)

#include "../../libs/stm32l0_low_level/stm32l0_ll/stm32l071xx.h"
#include "../../libs/stm32l0_low_level/stm32l0_ll/stm32l0xx_ll_rcc.h"
#include "../../libs/stm32l0_low_level/stm32l0_ll/stm32l0xx_ll_gpio.h"
#include "../../libs/stm32l0_low_level/stm32l0_ll/stm32l0xx_ll_system.h"
#include "../../libs/stm32l0_low_level/stm32l0_ll/stm32l0xx_ll_bus.h"
#include "../../libs/stm32l0_low_level/stm32l0_ll/stm32l0xx_ll_utils.h"

typedef struct pll_t {
    LL_UTILS_PLLInitTypeDef prescale;
    LL_UTILS_ClkInitTypeDef bus;
} pll_t;

pll_t pll = {.prescale = {.PLLMul = 1, .PLLDiv = 1},
    .bus = {.AHBCLKDivider = 2,
        .APB1CLKDivider = 2,
        .APB2CLKDivider = 2
    }};

int main(void) {

    // enable prefetch
    // TODO: explain why
    LL_FLASH_EnablePrefetch();

    // set up clocks - OSC_IN and OSC_OUT on PH0 and PH1,
    // HSE input clock is 14.7456 MHz
    // set PLL as system clock, using HSE as source
    LL_PLL_ConfigSystemClock_HSE(HSE_VALUE, LL_UTILS_HSEBYPASS_OFF, &pll.prescale, &pll.bus);

    // enable peripheral clocks
    LL_IOP_GRP1_EnableClock(LL_IOP_GRP1_PERIPH_GPIOA |
                            LL_IOP_GRP1_PERIPH_GPIOB |
                            LL_IOP_GRP1_PERIPH_GPIOC);

    LL_APB2_GRP1_EnableClock(LL_APB2_GRP1_PERIPH_USART1 |
                             LL_APB2_GRP1_PERIPH_SPI1);

    LL_APB1_GRP1_EnableClock(LL_APB1_GRP1_PERIPH_USART2);

    // set up peripherals - GPIO for control/output:
    // | Pin  | Pin Name        | Description
    // | ---- | --------------- | -----------------------------------------
    // | PB0  | CC1125_Tx_or_Rx | Switches Rx and Tx chain (0 = Rx, 1 = Tx)
    // | PB4  | CANSAT_Enable   | Enables power switch to CANSAT
    LL_GPIO_SetPinMode(GPIOB, LL_GPIO_PIN_0, LL_GPIO_MODE_OUTPUT);
    LL_GPIO_SetPinMode(GPIOB, LL_GPIO_PIN_4, LL_GPIO_MODE_OUTPUT);

    // | Pin  | Pin Name        | Description
    // | ---- | --------------- | -----------------------------------------
    // | PA0  | CC1125_Enable   | Enables power switch to CC1125
    // | PA15 | PA_Enable       | Enables (logic to the gate of) power amplifier
    // | PA8  | N2420_Enable    | Enables power switch to N2420
    LL_GPIO_SetPinMode(GPIOA, LL_GPIO_PIN_0,  LL_GPIO_MODE_OUTPUT);
    LL_GPIO_SetPinMode(GPIOA, LL_GPIO_PIN_15, LL_GPIO_MODE_OUTPUT);
    LL_GPIO_SetPinMode(GPIOA, LL_GPIO_PIN_8,  LL_GPIO_MODE_OUTPUT);

    // | Pin  | Pin Name        | Description
    // | ---- | --------------- | -----------------------------------------
    // | PC13 | MCU_Alive       | Connected as current source to heartbeat LED
    LL_GPIO_SetPinMode(GPIOC, LL_GPIO_PIN_13, LL_GPIO_MODE_OUTPUT);

    // set up peripherals - alternate pin functions are on page 45/136
    // of DM00141136 - STM32L071x8 datasheet
    //   - SPI for CC1125 - SPI1 on PA4/5/6/7
    LL_GPIO_SetPinMode(GPIOA, LL_GPIO_PIN_4, LL_GPIO_MODE_ALTERNATE);
    LL_GPIO_SetPinMode(GPIOA, LL_GPIO_PIN_5, LL_GPIO_MODE_ALTERNATE);
    LL_GPIO_SetPinMode(GPIOA, LL_GPIO_PIN_6, LL_GPIO_MODE_ALTERNATE);
    LL_GPIO_SetPinMode(GPIOA, LL_GPIO_PIN_7, LL_GPIO_MODE_ALTERNATE);

    LL_GPIO_SetAFPin_0_7(GPIOA, LL_GPIO_PIN_4, LL_GPIO_AF_0); // SPI1_NSS
    LL_GPIO_SetAFPin_0_7(GPIOA, LL_GPIO_PIN_5, LL_GPIO_AF_0); // SPI1_SCK
    LL_GPIO_SetAFPin_0_7(GPIOA, LL_GPIO_PIN_6, LL_GPIO_AF_0); // SPI1_MISO
    LL_GPIO_SetAFPin_0_7(GPIOA, LL_GPIO_PIN_7, LL_GPIO_AF_0); // SPI1_MOSI

    //   - UART for CANSAT - USART2 on PA2/3
    LL_GPIO_SetPinMode(GPIOA, LL_GPIO_PIN_2, LL_GPIO_MODE_ALTERNATE);
    LL_GPIO_SetPinMode(GPIOA, LL_GPIO_PIN_3, LL_GPIO_MODE_ALTERNATE);

    LL_GPIO_SetAFPin_0_7(GPIOA, LL_GPIO_PIN_2, LL_GPIO_AF_4); // USART2_TX
    LL_GPIO_SetAFPin_0_7(GPIOA, LL_GPIO_PIN_3, LL_GPIO_AF_4); // USART2_RX

    //   - UART for n2420 - USART1 on PA9/10
    LL_GPIO_SetPinMode(GPIOA, LL_GPIO_PIN_9,  LL_GPIO_MODE_ALTERNATE);
    LL_GPIO_SetPinMode(GPIOA, LL_GPIO_PIN_10, LL_GPIO_MODE_ALTERNATE);

    LL_GPIO_SetAFPin_8_15(GPIOA, LL_GPIO_PIN_9,  LL_GPIO_AF_4); // USART1_TX
    LL_GPIO_SetAFPin_8_15(GPIOA, LL_GPIO_PIN_10, LL_GPIO_AF_4); // USART1_RX

    while (1) {
        // goodnight sweet prince
        // FIXME: figure out peripheral sleeping
        __WFI();
    }

    return 0;
}
