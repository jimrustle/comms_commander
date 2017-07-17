
#include "radio.h"
#include "peripherals.h"

// CC1125
// | Pin  | Pin Name        | Description
// | PA0  | CC1125_Enable   | Enables power switch to CC1125
// | PB0  | CC1125_Tx_or_Rx | Switches Rx and Tx chain (0 = Rx, 1 = Tx)

void radio_CC1125_power_on(void) {
    LL_GPIO_SetOutputPin(GPIOA, LL_GPIO_PIN_1);
}

void radio_CC1125_power_off(void) {
    LL_GPIO_ResetOutputPin(GPIOA, LL_GPIO_PIN_1);
}

// CANSAT
// | PB4  | CANSAT_Enable   | Enables power switch to CANSAT

void radio_CANSAT_power_on(void) {
    LL_GPIO_SetOutputPin(GPIOB, LL_GPIO_PIN_4);
}

void radio_CANSAT_power_off(void) {
    LL_GPIO_ResetOutputPin(GPIOB, LL_GPIO_PIN_4);
}

// Power Amplifier - Mitsubishi RA07M4047M
// | PA15 | PA_Enable       | Enables (logic to the gate of) power amplifier

void radio_PA_power_on(void) {
    LL_GPIO_SetOutputPin(GPIOA, LL_GPIO_PIN_15);
}

void radio_PA_power_off(void) {
    LL_GPIO_ResetOutputPin(GPIOA, LL_GPIO_PIN_15);
}

// MCU
void radio_LED_on(void) {
    LL_GPIO_SetOutputPin(GPIOC, LL_GPIO_PIN_13);
}

void radio_LED_off(void) {
    LL_GPIO_ResetOutputPin(GPIOC, LL_GPIO_PIN_13);
}
