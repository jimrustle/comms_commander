
#include <stdint.h>

typedef enum CANSAT_baud_t {
    M1200_AFSK,
    M9600_FSK
} CANSAT_baud_t;

typedef enum CC1125_mode_t {
    CC_TRANSMIT,
    CC_RECEIVE
} CC1125_mode_t;

// dummy variables for now
typedef struct telemetry_t {
    uint8_t internal_temperature;
    uint8_t external_temperature;
    uint8_t gps_lat;
    uint8_t gps_long;
    uint8_t gps_lastfix;
    uint8_t utc_time;
    uint8_t battery_voltage;
} telemetry_t;

void radio_CC1125_set_mode(CC1125_mode_t mode);
void radio_CC1125_power_on(void);
void radio_CC1125_power_off(void);
void radio_CC1125_get_status(void);
void radio_CC1125_config_radio(void);
void radio_CC1125_enable_RX(void);
void radio_CC1125_enable_TX(void);

void radio_CC1125_test(void);

void radio_CANSAT_power_on(void);
void radio_CANSAT_power_off(void);
void radio_CANSAT_init_callsigns(void);
void radio_CANSAT_send_data(uint8_t* data, uint8_t len);
void radio_CANSAT_set_freq(void);
void radio_CANSAT_set_baud(CANSAT_baud_t);

void radio_CANSAT_test(void);

void radio_PA_power_on(void);
void radio_PA_power_off(void);

void radio_LED_on(void);
void radio_LED_off(void);
void radio_LED_toggle(void);
