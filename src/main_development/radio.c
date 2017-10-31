// This is a personal academic project. Dear PVS-Studio, please check it.
// PVS-Studio Static Code Analyzer for C, C++ and C#: http://www.viva64.com

#include "radio.h"
#include "log.h"
#include "peripherals.h"
#include "print_queue.h"
#include "radio_cc1125.h"
#include "spi.h"

#include "error.h"

volatile uint8_t spi_tx_byte = 0;
volatile uint8_t spi_rx_byte = 0;

/***************************************************/

static void rxtx_switch_set_mode_RX_DATA(void)
{
    // set Tx/Rx switch to receive path
    LL_GPIO_ResetOutputPin(GPIOB, LL_GPIO_PIN_0);
}

static void rxtx_switch_set_mode_TX_DATA(void)
{
    // set Tx/Rx switch to transmit path
    LL_GPIO_SetOutputPin(GPIOB, LL_GPIO_PIN_0);
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

#ifdef SPI_BITBANG
    spi_ss_low();
    LL_GPIO_ResetOutputPin(GPIOA, LL_GPIO_PIN_5); // set SCLK low
    LL_GPIO_ResetOutputPin(GPIOA, LL_GPIO_PIN_7); // set MOSI low
#endif
}

// sets the radio mode using a "global" configuration parameter
void radio_set_mode(radio_config_t mode)
{
    if (mode.is_enabled) {
        if (mode.tx_type == RX_DATA) {
            rxtx_switch_set_mode_RX_DATA();
            radio_CC1125_power_on();
            radio_CANSAT_power_off();
        } else if (mode.tx_type == TX_DATA) {
            radio_CC1125_power_on();
            radio_CANSAT_power_off();
            rxtx_switch_set_mode_TX_DATA();
        } else if (mode.tx_type == TX_TELEMETRY) {
            rxtx_switch_set_mode_TX_DATA();
            radio_CC1125_power_off();
            radio_CANSAT_power_on();
        }
    } else {
        // is_enabled is false and thus we turn off both radios
            radio_CC1125_power_off();
            radio_CANSAT_power_off();
    }
}

static const char* status_states[] = {
    "IDLE",
    "RX",
    "TX",
    "FSTXON",
    "CALIBRATE",
    "SETTLING",
    "RX FIFO ERROR",
    "TX FIFO ERROR"
};

void radio_CC1125_get_status(void)
{
    // get status by reading status byte from NOP command strobe
    uint8_t status = cc1125_read_byte(CC1125_SNOP);
    status = (status & 0x70) >> 4;

    pr_nl(USART_1);
    pr_str(USART_1, status_states[status]);
    pr_nl(USART_1);

    /*uint8_t num_bytes = cc1125_read_byte(CC1125_NUM_TXBYTES);*/
    /*pr_hex(USART_1, num_bytes);*/
    /*pr_nl(USART_1);*/

    /*[>// direct fifo debug<]*/
    /*for (uint8_t i = 0; i < num_bytes; i++) {*/
        /*[> for (uint8_t i = 0; i < 7; i++) { <]*/
        /*spi_ss_low();*/
        /*spi_read_write_byte(0x80 | 0x3E); // direct fifo read*/
        /*spi_read_write_byte(i);*/
        /*uint8_t ret = spi_read_write_byte(0x00);*/
        /*spi_ss_high();*/

        /*pr_hex(USART_1, i);*/
        /*pr_str(USART_1, ": ");*/
        /*pr_hex(USART_1, ret);*/
        /*pr_ch(USART_1, ' ');*/
        /*pr_ch(USART_1, ret);*/
        /*pr_nl(USART_1);*/
    /*}*/
}

// configure radio using the preferred settings (generated from SmartRF Studio)
void radio_CC1125_config_radio(void)
{
    cc1125_command_strobe(CC1125_SRES);
    cc1125_config_radio();
}

// enable tx mode - should be called only after filling the TX FIFO
// (check using polling, for now)
void radio_CC1125_enable_TX(void)
{
    /* cc1125_command_strobe(CC1125_SFSTXON); */
    cc1125_command_strobe(CC1125_STX);
    cc1125_command_strobe(CC1125_SFTX);
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
    /* pr_hex(USART_1, cc1125_read_byte(0x3D)); */
    /* pr_hex(USART_1, cc1125_read_byte(0x80)); */
    /* pr_hex(USART_1, cc1125_read_byte(0x00)); */
    /* pr_hex(USART_1, cc1125_read_byte(0x01)); */
    /* pr_hex(USART_1, cc1125_read_byte(0x80)); */
    for (int i = 0; i < 16; i++) {
        cc1125_write_byte(0x3F, 'N');
        cc1125_write_byte(0x3F, 'E');
        cc1125_write_byte(0x3F, 'U');
        cc1125_write_byte(0x3F, 'D');
        cc1125_write_byte(0x3F, 'O');
        cc1125_write_byte(0x3F, 'S');
        cc1125_write_byte(0x3F, 'E');
        cc1125_write_byte(0x3F, 'E');
    }
}

void radio_CC1125_fill_buf(void)
{
    cc1125_write_byte(0x3F, 'E');
}
/***************************************************/

// CANSAT
// | PB4  | CANSAT_Enable   | Enables power switch to CANSAT

void radio_CANSAT_power_on(void)
{
    // set pin to TPS22945 to high
    LL_GPIO_SetOutputPin(GPIOB, LL_GPIO_PIN_4);

    /*   - UART for CANSAT - USART2 on PA2 and PA3 */
    LL_GPIO_SetPinMode(GPIOA, LL_GPIO_PIN_2, LL_GPIO_MODE_ALTERNATE);
    LL_GPIO_SetPinMode(GPIOA, LL_GPIO_PIN_3, LL_GPIO_MODE_ALTERNATE);

    LL_GPIO_SetAFPin_0_7(GPIOA, LL_GPIO_PIN_2, LL_GPIO_AF_4); // USART2_TX
    LL_GPIO_SetAFPin_0_7(GPIOA, LL_GPIO_PIN_3, LL_GPIO_AF_4); // USART2_RX

    /*LL_GPIO_SetPinOutputType(GPIOA, LL_GPIO_PIN_2, LL_GPIO_OUTPUT_PUSHPULL);*/
    /*LL_GPIO_SetPinOutputType(GPIOA, LL_GPIO_PIN_3, LL_GPIO_OUTPUT_PUSHPULL);*/

    LL_GPIO_SetPinOutputType(GPIOA, LL_GPIO_PIN_2, LL_GPIO_OUTPUT_OPENDRAIN);
    LL_GPIO_SetPinOutputType(GPIOA, LL_GPIO_PIN_3, LL_GPIO_OUTPUT_OPENDRAIN);

    LL_GPIO_SetPinPull(GPIOA, LL_GPIO_PIN_2, LL_GPIO_PULL_UP);
    LL_GPIO_SetPinPull(GPIOA, LL_GPIO_PIN_3, LL_GPIO_PULL_UP);
}

void radio_CANSAT_power_off(void)
{
    // set pin to TPS22945 to low
    LL_GPIO_ResetOutputPin(GPIOB, LL_GPIO_PIN_4);

    // set UART pins to high-impedance
    /*   - UART for CANSAT - USART2 on PA2 and PA3 */
    LL_GPIO_SetPinMode(GPIOA, LL_GPIO_PIN_2, LL_GPIO_MODE_INPUT);
    LL_GPIO_SetPinPull(GPIOA, LL_GPIO_PIN_2, LL_GPIO_PULL_DOWN);

    LL_GPIO_SetPinMode(GPIOA, LL_GPIO_PIN_3, LL_GPIO_MODE_INPUT);
    LL_GPIO_SetPinPull(GPIOA, LL_GPIO_PIN_3, LL_GPIO_PULL_DOWN);
}

void radio_CANSAT_init_callsigns(void)
{
    pr_str(USART_2, "CN\r");
    /* pr_str(USART_2, "DCQ\r"); */
    /* pr_str(USART_2, "VTELEM\r"); */
}

void radio_CANSAT_send_data(uint8_t* data, uint8_t len)
{
    /* static const char* check_chars = "ABCDEFGHIJKLMNOPQRSTUVWXYZabcdefghijklmnopqrstuvwxyz0123456789+/="; */
    // len is max of 200 (CANSAT datasheet)
    len = (len > 200) ? 200 : len;

    // ensure all ascii
    for (uint8_t i = 0; i < len; i++) {
        char c = data[i];

        if (!IS_BASE64_ENCODED(c)) {
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
    /* radio_CANSAT_send_data((uint8_t*)"hi", 2); */
    pr_ch(USART_2, 'S');
    pr_ch(USART_2, 'h');
    pr_ch(USART_2, 'i');
    pr_ch(USART_2, '\r');
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
