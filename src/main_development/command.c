// This is a personal academic project. Dear PVS-Studio, please check it.
// PVS-Studio Static Code Analyzer for C, C++ and C#: http://www.viva64.com

#include "command.h"
#include "error.h"
#include "log.h"
#include "radio.h"

#include <stdbool.h>
#include <string.h>

#define COM_BUF_LEN 10

typedef struct command_t {
    const char* command;
    void (*command_func)(void);
} command_t;

// should we even be keeping state?
typedef struct radio_t {
    bool CANSAT_state; // on/off
    bool CANSAT_baud; // M1200/M9600
    bool CC_state; // on/off
    bool CC_mode; // modulation/freq/idk
} radio_t;

static command_t command_table[] = {
    { "CS0", radio_CANSAT_power_off },
    { "CS1", radio_CANSAT_power_on },
    { "CST", radio_CANSAT_test },
    /*{ "CSTT", radio_CANSAT_tx_telemetry },*/

    { "L0", radio_LED_off },
    { "L1", radio_LED_on },
    { "LT", radio_LED_toggle },

    { "T", radio_CC1125_test },
    { "S", radio_CC1125_get_status },
    { "CCC", radio_CC1125_config_radio },
    { "CCT", radio_CC1125_enable_TX },
    { "CCR", radio_CC1125_enable_RX },
    { "CC0", radio_CC1125_power_off },
    { "CCTX", radio_CC1125_set_mode_TX },
    { "CCRX", radio_CC1125_set_mode_RX },
    { "CC1", radio_CC1125_power_on },

    { "PA0", radio_PA_power_off },
    { "PA1", radio_PA_power_on },

    { "K", error_catch },
    { "R", error_catch },
};

extern volatile bool flag;
void add_c(char c)
{
    static char command_buf[COM_BUF_LEN] = { 0 };
    static int buf_idx = 0;

    // c cannot be a non ASCII character
    /* if (!IS_BASE64_ENCODED(c) && (c != '\r')) { */
    /*   error_catch(); */
    /* } */

    // if '\r', interpret command
    if (c == '\r') {
        // zero out rest of buffer
        for (uint8_t i = buf_idx; i < COM_BUF_LEN; i++) {
            command_buf[i] = 0;
        }

        if (strcmp(command_buf, "F") == 0) {
          flag ^= 1;
          pr_nl(USART_1);
          pr_str(USART_1, "ok.");
          buf_idx = 0;
          return;
        }

        // check match
        for (uint8_t i = 0; i < sizeof(command_table) / sizeof(command_t); i++) {
            if (strcmp(command_buf, command_table[i].command) == 0) {
                command_table[i].command_func();
                pr_nl(USART_1);
                pr_str(USART_1, "ok.");
                break;
            }
        }

        pr_nl(USART_1);

        buf_idx = 0;
    } else { // else, add char
        command_buf[buf_idx++] = c;
        buf_idx = buf_idx % COM_BUF_LEN;
    }
}
