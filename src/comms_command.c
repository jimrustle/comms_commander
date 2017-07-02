#include <stdbool.h>

#ifndef STM32L0
#define STM32L0
#endif /* ifndef STM32L0 */

#include "../libs/libopencm3/include/libopencm3/stm32/rcc.h"
#include "../libs/libopencm3/include/libopencm3/stm32/gpio.h"

#define LED_GREEN_PIN GPIO13
#define LED_GREEN_PORT GPIOC

static void gpio_setup(void) {
    /* Enable GPIOB clock. */
    rcc_periph_clock_enable(RCC_GPIOA);
    rcc_periph_clock_enable(RCC_GPIOB);

    /* set pins to output mode, push pull */
    gpio_mode_setup(LED_GREEN_PORT, GPIO_MODE_OUTPUT, GPIO_PUPD_NONE, LED_GREEN_PIN);
}

int main(void) {
    // set up clocks - OSC_IN and OSC_OUT on PH0 and PH1,
    // HSE input clock is 14.7456 MHz

    // set up peripherals:
    //  - GPIO for control/output:
    // | PB0  | CC1125_Tx_or_Rx | Switches Rx and Tx chain (0 = Rx, 1 = Tx)
    // | PA0  | CC1125_Enable   | Enables power switch to CC1125
    // | PA15 | PA_Enable       | Enables (logic to the gate of) power amplifier
    // | PB4  | CANSAT_Enable   | Enables power switch to CANSAT
    // | PA8  | N2420_Enable    | Enables power switch to N2420
    // | PC13 | MCU_Alive       | Connected as current source to heartbeat LED

    //   - SPI for CC1125 - SPI1 on PA4/5/6/7

    //   - UART for CANSAT - USART2 on PA2/3

    //   - UART for n2420 - USART1 on PA9/10

    gpio_setup();

    while (1) {
        /*while (true) {*/
        /*// goodnight sweet prince*/
        /*__asm("WFI");*/
        gpio_toggle(LED_GREEN_PORT, LED_GREEN_PIN);
        for (int i = 0; i < 100000; i++) {	/* Wait a bit. */
            __asm__("nop");
        }
        /*}*/
    }

    return 0;
}
