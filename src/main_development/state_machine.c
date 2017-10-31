
#include <stdint.h>

#include "error.h"
#include "radio.h"
#include "radio_cc1125.h"
#include "state.h"

typedef enum main_state_t {
    CC1125_ACTIVE,
    CANSAT_ACTIVE,
} main_state_t;

typedef enum CC1125_state_t {
    CC_PWR_ON,
    CC_PWR_OFF,
    CC_CONFIGURE,
    CC1_TRANSMIT,
    CC_FILL_BUF
} CC1125_state_t;

typedef enum CANSAT_state_t {
    CS_PWR_ON,
    CS_PWR_OFF,
    CS_TRANSMIT
} CANSAT_state_t;

typedef struct state_t {
    main_state_t m;
    union {
        CC1125_state_t ccs;
        CANSAT_state_t css;
    } s;
    uint32_t delay;
    void (*action)(void);
    main_state_t next_m;
    union {
        CC1125_state_t next_ccs;
        CANSAT_state_t next_css;
    } next_s;
} state_t;

state_t program_states[] = {
    // CS_PWR_ON, CS_PWR_OFF, CS_TRANSMIT
    {
        .m = CANSAT_ACTIVE,
        .s = CS_PWR_ON,
        .delay = 100,
        .action = radio_CANSAT_power_on,
        .next_m = CANSAT_ACTIVE,
        .next_s = CS_TRANSMIT,
    },
    {
        .m = CANSAT_ACTIVE,
        .s = CS_TRANSMIT,
        .delay = 800,
        .action = radio_CANSAT_test,
        .next_m = CANSAT_ACTIVE,
        .next_s = CS_PWR_OFF,
    },
    {
        .m = CANSAT_ACTIVE,
        .s = CS_PWR_OFF,
        .delay = 100,
        .action = radio_CANSAT_power_off,
        .next_m = CC1125_ACTIVE,
        .next_s = CC_PWR_ON,
    },
    // CC_PWR_ON, CC_PWR_OFF, CC_CONFIGURE, CC1_TRANSMIT, CC_FILL_BUF
    {
        .m = CC1125_ACTIVE,
        .s = CC_PWR_ON,
        .delay = 200,
        .action = radio_CC1125_power_on,
        .next_m = CC1125_ACTIVE,
        .next_s = CC_CONFIGURE,
    },
    {
        .m = CC1125_ACTIVE,
        .s = CC_CONFIGURE,
        .delay = 200,
        .action = radio_CC1125_config_radio,
        .next_m = CC1125_ACTIVE,
        .next_s = CC1_TRANSMIT,
    },
    {
        .m = CC1125_ACTIVE,
        .s = CC1_TRANSMIT,
        .delay = 100,
        .action = radio_CC1125_enable_TX,
        .next_m = CC1125_ACTIVE,
        .next_s = CC_FILL_BUF,
    },
    {
        .m = CC1125_ACTIVE,
        .s = CC_FILL_BUF,
        .delay = 200,
        .action = radio_CC1125_test,
        .next_m = CC1125_ACTIVE,
        .next_s = CC_PWR_OFF,
    },
    {
        .m = CC1125_ACTIVE,
        .s = CC_PWR_OFF,
        .delay = 100,
        .action = radio_CC1125_power_off,
        .next_m = CANSAT_ACTIVE,
        .next_s = CS_PWR_ON,
    },
};

void run(void)
{
    static volatile uint32_t prev_millis = 0;
    static bool done_already = false;
    static state_t status = {.m = CANSAT_ACTIVE, .s = CS_PWR_OFF };

    for (uint8_t i = 0; i < sizeof(program_states) / sizeof(state_t); i++) {
        state_t check = program_states[i];
        if ((status.m == check.m) && ((status.m == CANSAT_ACTIVE) ? (status.s.css == check.s.css) : (status.s.ccs == check.s.ccs))) {

            // action() will be called when you enter the new state
            if (!done_already) {
                check.action();
                done_already = true;
            }

            // do nothing until time exceeds delay
            else if (millis - prev_millis >= check.delay) {
                done_already = false;
                prev_millis = millis;

                status.m = check.next_m;
                if (status.m == CANSAT_ACTIVE) {
                    status.s.css = check.next_s.next_css;
                } else {
                    status.s.ccs = check.next_s.next_ccs;
                }
            }
            return;
        }
    }
}
