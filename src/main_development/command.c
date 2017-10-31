// This is a personal academic project. Dear PVS-Studio, please check it.
// PVS-Studio Static Code Analyzer for C, C++ and C#: http://www.viva64.com

#include <stdbool.h>
#include <stdint.h>
#include <string.h>

#include "command.h"
#include "error.h"
#include "log.h"
#include "radio.h"

#define COM_BUF_LEN 10
#define FIFO_LEN 128

typedef struct command_t {
    const char* command;
    void (*command_func)(void);
} command_t;

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
    /* { "CCF", radio_CC1125_fill_buf }, */
    { "CC0", radio_CC1125_power_off },
    /*{ "CCTX", radio_CC1125_set_mode_TX },*/
    /*{ "CCRX", radio_CC1125_set_mode_RX },*/
    { "CC1", radio_CC1125_power_on },

    { "PA0", radio_PA_power_off },
    { "PA1", radio_PA_power_on },

    { "K", error_catch },
};

// adds a character to a buffer and checks whether it matches a command string
// and executes the associated function
//
// or, if in "FIFO" mode, adds a maximum of 128 bytes into a FIFO for transmission
// from the CC1125 radios
void parse_char(char c)
{
    /* static bool fifo_mode = false; */
    static char command_buf[COM_BUF_LEN] = { 0 };
    /* static char fifo_buf[FIFO_LEN] = { 0 }; */
    static int buf_idx = 0;

    // c cannot be a non ASCII character
    if (!IS_BASE64_ENCODED(c) && (c != '\r')) {
        return;
        //error_catch();
    }

    /* if (fifo_mode) { */
    /*   // exit fifo mode if newline char */
    /*   if (c == '\r') { */
    /*     fifo_mode = false; */
    /*     buf_idx = 0; */
    /*     return; */
    /*   } */
    /*   fifo_buf[buf_idx++] = c; */
    /*   buf_idx = buf_idx % FIFO_LEN; */
    /* } */
    /*else {*/
        // if 'FIFO', enter FIFO mode
        /* if (strcmp(command_buf, "FIFO\r") == 0) { */
        /*   fifo_mode = true; */

        /*   // clear buffer before entering FIFO mode */
        /*   for (uint8_t i = 0; i < COM_BUF_LEN; i++) { */
        /*     command_buf[i] = 0; */
        /*   } */
        /*   buf_idx = 0; */
        /*   return; */
        /* } */

        // if '\r', interpret command
        if (c == '\r') {
            // zero out rest of buffer
            for (uint8_t i = buf_idx; i < COM_BUF_LEN; i++) {
                command_buf[i] = 0;
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
        } else {
            // else, add char to the buffer
            command_buf[buf_idx++] = c;
            buf_idx = buf_idx % COM_BUF_LEN;
        }
    /*}*/
}
