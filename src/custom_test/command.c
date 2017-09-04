// This is a personal academic project. Dear PVS-Studio, please check it.
// PVS-Studio Static Code Analyzer for C, C++ and C#: http://www.viva64.com

#include "command.h"
#include "log.h"
#include "radio.h"
#include <string.h>

typedef struct command_t {
  const char * command;
  void (* command_func)(void);
} command_t;

static command_t command_table[] = {
  {"CS0", radio_CANSAT_power_off},
  {"CS1", radio_CANSAT_power_on},

  {"L0",  radio_LED_off},
  {"L1",  radio_LED_on},
  {"LT",  radio_LED_toggle},

  {"T",   radio_CC1125_test},
  {"S",   radio_CC1125_get_status},
  {"CCC", radio_CC1125_config_radio},
  {"CCT", radio_CC1125_enable_TX},
  {"CCR", radio_CC1125_enable_RX},
  {"CC0", radio_CC1125_power_off},
  {"CC1", radio_CC1125_power_on},

  {"PA0", radio_PA_power_off},
  {"PA1", radio_PA_power_on},
};

void add_c(char c) {
    static char command_buf[10] = {0};
    static int buf_idx = 0;

    // if '\r', interpret command
    if (c == '\r') {
        // zero out rest of buffer
        for (uint8_t i = buf_idx; i < 10; i++) {
            command_buf[i] = 0;
        }

        // check match
        for (uint8_t i = 0; i < sizeof(command_table)/sizeof(command_t); i++) {
            if (strcmp(command_buf, command_table[i].command) == 0) {
                command_table[i].command_func();
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

