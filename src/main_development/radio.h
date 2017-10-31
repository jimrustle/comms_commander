#include <stdint.h>
#include <stdbool.h>

typedef enum radio_t {
    CC1125, CANSAT
} radio_t;

typedef enum transmission_type_t {
    TX_TELEMETRY,
    TX_DATA,
    RX_DATA
} transmission_type_t;

typedef struct CANSAT_callsign_t {
    char source[6];
    char destination[6];
    char via_relay[6];
} CANSAT_callsign_t;

typedef enum CANSAT_baud_t {
    M1200_AFSK,
    M9600_FSK
} CANSAT_baud_t;

typedef struct CANSAT_config_t {
    CANSAT_callsign_t callsign;
    CANSAT_baud_t baud;
    // there's also a frequency setting here
} CANSAT_config_t;

typedef struct CC1125_config_t {
    // maybe if this radio could be configured
    // with sensible parameters
} CC1125_config_t;

typedef struct radio_config_t {
    bool is_enabled;
    transmission_type_t tx_type;
    CANSAT_config_t CANSAT_cfg;
    CC1125_config_t CC1125_cfg;
} radio_config_t;

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

void radio_set_mode(radio_config_t radio_mode);

void radio_CC1125_power_on(void);
void radio_CC1125_power_off(void);
void radio_CC1125_get_status(void); 
void radio_CC1125_config_radio(void); 
void radio_CC1125_enable_RX(void); 
void radio_CC1125_enable_TX(void); 
void radio_CC1125_fill_buf(void); 

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
