
#include<stdint.h>

typedef enum CANSAT_baud_t {
  M1200_AFSK, M9600_FSK
} CANSAT_baud_t;

void radio_CC1125_power_on(void);
void radio_CC1125_power_off(void);
void radio_CC1125_test(void);
void radio_CC1125_get_status(void);
void radio_CC1125_config_radio(void);
void radio_CC1125_enable_RX(void);
void radio_CC1125_enable_TX(void);
void radio_CANSAT_power_on(void);
void radio_CANSAT_power_off(void);
void radio_CANSAT_init_callsigns(void);
void radio_CANSAT_send_data(uint8_t * data, uint8_t len);
void radio_CANSAT_set_freq(void);
void radio_CANSAT_set_baud(CANSAT_baud_t);

void radio_PA_power_on(void);
void radio_PA_power_off(void);

void radio_LED_on(void);
void radio_LED_off(void);
void radio_LED_toggle(void);

