// This is a personal academic project. Dear PVS-Studio, please check it.
// PVS-Studio Static Code Analyzer for C, C++ and C#: http://www.viva64.com

#include "radio.h"
#include "error.h"
#include "log.h"
#include "peripherals.h"
#include "print_queue.h"
#include "radio_cc1125.h"
#include "spi.h"

uint8_t spi_tx_byte = 0;
/* uint8_t spi_rx_byte = 0; */

/***************************************************/

// Tx or Rx mode

void radio_CC1125_set_mode(CC1125_mode_t mode)
{
    if (mode == CC_TRANSMIT) {
        LL_GPIO_SetOutputPin(GPIOB, LL_GPIO_PIN_0);
    } else { // mode == CC_RECEIVE
        LL_GPIO_ResetOutputPin(GPIOB, LL_GPIO_PIN_0);
    }
}

// CC1125
// | Pin  | Pin Name        | Description
// | PA0  | CC1125_Enable   | Enables power switch to CC1125
// | PB0  | CC1125_Tx_or_Rx | Switches Rx and Tx chain (0 = Rx, 1 = Tx)

void radio_CC1125_power_on(void)
{
    LL_GPIO_SetOutputPin(GPIOA, LL_GPIO_PIN_1);
}

void radio_CC1125_power_off(void)
{
    LL_GPIO_ResetOutputPin(GPIOA, LL_GPIO_PIN_1);
}

void radio_CC1125_get_status(void)
{
    // get status by reading status byte from NOP command strobe
    pr_hex(USART_1, cc1125_read_byte(CC1125_SNOP));
    pr_ch(USART_1, ' ');

    /* pr_hex(USART_1, cc1125_read_byte(CC1125_MODEM_STATUS0)); pr_ch(USART_1, ' '); */

    /* pr_hex(USART_1, cc1125_read_byte(CC1125_NUM_TXBYTES)); pr_nl(USART_1); */

    /* // direct fifo debug */
    /* for (uint8_t i = 0; i < 7; i++) { */
    /*   spi_ss_low(); */
    /*   spi_read_write_byte(0x80 | 0x3E); // direct fifo read */
    /*   spi_read_write_byte(i); */
    /*   uint8_t ret = spi_read_write_byte(0x00); */
    /*   spi_ss_high(); */

    /*   pr_hex(USART_1, i); pr_str(USART_1, ": "); pr_hex(USART_1, ret); pr_nl(USART_1); */
    /* } */
}

// configure radio using the preferred settings (generated from SmartRF Studio)
void radio_CC1125_config_radio(void)
{
    cc1125_config_radio();
}

// enable tx mode - should be called only after filling the TX FIFO
// (check using polling, for now)
void radio_CC1125_enable_TX(void)
{
    cc1125_command_strobe(CC1125_SFSTXON);
    cc1125_command_strobe(CC1125_STX);
}

// enable rx mode - should be called only after filling the RX FIFO
// (check using polling, for now)
void radio_CC1125_enable_RX(void)
{
    cc1125_command_strobe(CC1125_SFSTXON);
    cc1125_command_strobe(CC1125_SRX);
}

// an ever-changing function for debugging
// current usage: fill out TX FIFO
void radio_CC1125_test(void)
{
    pr_hex(USART_1, cc1125_read_byte(0x3D));
    pr_hex(USART_1, cc1125_read_byte(0x80));
    pr_hex(USART_1, cc1125_read_byte(0x00));
    pr_hex(USART_1, cc1125_read_byte(0x01));
    pr_hex(USART_1, cc1125_read_byte(0x80));
}

/***************************************************/

// CANSAT
// | PB4  | CANSAT_Enable   | Enables power switch to CANSAT

void radio_CANSAT_power_on(void)
{
    LL_GPIO_SetOutputPin(GPIOB, LL_GPIO_PIN_4);
}

void radio_CANSAT_power_off(void)
{
    LL_GPIO_ResetOutputPin(GPIOB, LL_GPIO_PIN_4);
}

void radio_CANSAT_init_callsigns(void)
{
    pr_str(USART_2, "CNEUDOSE\r");
    /* pr_str(USART_2, "DCQ\r"); */
    /* pr_str(USART_2, "VTELEM\r"); */
}

void radio_CANSAT_send_data(uint8_t* data, uint8_t len)
{
    /* static const char* check_chars = "ABCDEFGHIJKLMNOPQRSTUVWXYZabcdefghijklmnopqrstuvwxyz0123456789+/="; */
    // len is max of 200
    len = (len > 200) ? 200 : len;

    // ensure all ascii
    for (uint8_t i = 0; i < len; i++) {
        char c = data[i];

        if (!((('A' <= c) && (c <= 'Z')) || (('a' <= c) && (c <= 'z')) || (('0' <= c) && (c <= '9')) || (c == '+') || (c == '/') || (c == '='))) {
            error_catch();
        }
    }

    // transmit
    pr_ch(USART_2, 'S');
    for (uint8_t i = 0; i < len; i++) {
        pr_ch(USART_2, data[i]);
    }
    pr_ch(USART_2, '\r');
}

void radio_CANSAT_set_freq(void)
{
    // wow, it does absolutely nothing!
}

void radio_CANSAT_set_baud(CANSAT_baud_t baud)
{
    switch (baud) {
    case M1200_AFSK: {
        pr_str(USART_2, "M1200\r");
        break;
    }
    case M9600_FSK: {
        pr_str(USART_2, "M9600\r");
        break;
    }
    }
}

void radio_CANSAT_test(void)
{
    radio_CANSAT_send_data((uint8_t*)"hi", 2);
}

/***************************************************/

// Power Amplifier - Mitsubishi RA07M4047M
// | PA15 | PA_Enable       | Enables (logic to the gate of) power amplifier

void radio_PA_power_on(void)
{
    LL_GPIO_SetOutputPin(GPIOA, LL_GPIO_PIN_15);
}

void radio_PA_power_off(void)
{
    LL_GPIO_ResetOutputPin(GPIOA, LL_GPIO_PIN_15);
}

/***************************************************/

// MCU
void radio_LED_on(void)
{
    LL_GPIO_SetOutputPin(GPIOC, LL_GPIO_PIN_13);
}

void radio_LED_off(void)
{
    LL_GPIO_ResetOutputPin(GPIOC, LL_GPIO_PIN_13);
}

void radio_LED_toggle(void)
{
    LL_GPIO_TogglePin(GPIOC, LL_GPIO_PIN_13);
}
