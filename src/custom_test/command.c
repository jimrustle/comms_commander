// This is a personal academic project. Dear PVS-Studio, please check it.
// PVS-Studio Static Code Analyzer for C, C++ and C#: http://www.viva64.com

#include "command.h"
#include "printf.h"
#include "log.h"
#include "radio.h"
#include <string.h>

#define N_COMMANDS 14
static const char* command_table[] = {
    "CC0",
    "CC1",
    "CS0",
    "CS1",
    "L0",
    "L1",
    "LT",
    "CCR",
    "CCT",
    "T",
    "S",
    "CCC",
    "PA0",
    "PA1",
};

static void (* commands[])(void) = {
    radio_CC1125_power_off,
    radio_CC1125_power_on,
    radio_CANSAT_power_off,
    radio_CANSAT_power_on,
    radio_LED_off,
    radio_LED_on,
    radio_LED_toggle,
    radio_CC1125_enable_RX,
    radio_CC1125_enable_TX,
    radio_CC1125_test,
    radio_CC1125_get_status,
    radio_CC1125_config_radio,
    radio_PA_power_off,
    radio_PA_power_on,
};

void add_c(char c) {
    static char command_buf[10] = {0};
    static int buf_idx = 0;

    // if '\r', interpret command
    if (c == '\r') {
        // zero out rest of buffer
        for (int i = buf_idx; i < 10; i++) {
            command_buf[i] = 0;
        }

        // check match
        for (int i = 0; i < N_COMMANDS; i++) {
            if (strcmp(command_buf, command_table[i]) == 0) {
                commands[i]();
                pr_nl(); pr_str("ok.");
                break;
            }
        }

        pr_nl();

        buf_idx = 0;
    }
    else { // else, add char
        command_buf[buf_idx++] = c;
        buf_idx = buf_idx % 10;
    }
}

