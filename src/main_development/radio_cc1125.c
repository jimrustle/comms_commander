// This is a personal academic project. Dear PVS-Studio, please check it.
// PVS-Studio Static Code Analyzer for C, C++ and C#: http://www.viva64.com

#include "spi.h"
#include "log.h"

#include "radio_cc1125.h"
#include "error.h"

typedef struct registerSetting_t {
  uint16_t addr;
  uint8_t data;
} registerSetting_t;

// settings generated by SmartRF Studio (434 MHz max throughput defaults)
static const registerSetting_t preferredSettings[]= {
  {CC1125_IOCFG3,            0xB0}, // analog transfer enabled, PKT_SYNC_RXTX
  {CC1125_IOCFG2,            0x06}, // PKT_CRC_OK
  {CC1125_IOCFG1,            0xB0}, // analog transfer enabled (IOCFG_3)
  {CC1125_IOCFG0,            0x40}, // inverter output enable, EXT_OSC_EN
  {CC1125_SYNC_CFG1,         0x07}, // sync threshold (see manual)
  {CC1125_DEVIATION_M,       0x53}, // if DEV_E > 0 => f_dev = f_xosc*(256+DEV_M)*2^DEV_E/2^24 [Hz],
                                    // if DEV_E = 0 => f_dev = f_xosc*DEV_M/2^23 [Hz]
  {CC1125_MODCFG_DEV_E,      0x2F}, // 4-GFSK, DEV_E (see DEVIATION_M)
  {CC1125_DCFILT_CFG,        0x04}, // DC compensation filter algo enabled, 32-sample BW filter
  {CC1125_PREAMBLE_CFG1,     0x18}, // 4-byte preamble (preamble minimum size = 32 bits)
  {CC1125_FREQ_IF_CFG,       0x00}, // f_IF = f_xosc*FREQ_IF/2^15 [kHz], FREQ_IF is two's complement
  {CC1125_IQIC,              0x00}, // disable IQ compensation, IQIC coeffs disabled,
                                    // IQIC block = 8 samples
  {CC1125_CHAN_BW,           0x01}, // RX filter bw = 200 kHz
  {CC1125_MDMCFG0,           0x05}, // filter enabled, BW = 5.7 Hz - 250.0 Hz
                                    // transparent data filter disabled
  {CC1125_SYMBOL_RATE2,      0xA9},
  {CC1125_SYMBOL_RATE1,      0x99},
  {CC1125_SYMBOL_RATE0,      0x9A}, // symbol rate = 100 ksps (with 4-GFSK, 200 kbit/s)
  {CC1125_AGC_REF,           0x3C}, // AGC_REFERENCE = 10*log10(RX Filter BW)-106-RSSI Offset,
  {CC1125_AGC_CS_THR,        0xEC},
  {CC1125_AGC_CFG3,          0x83},
  {CC1125_AGC_CFG2,          0x60},
  {CC1125_AGC_CFG1,          0xA9},
  {CC1125_AGC_CFG0,          0xC0},
  {CC1125_FIFO_CFG,          0x00}, // autoflush of rx fifo on crc error is disabled
  {CC1125_FS_CFG,            0x14}, // out of lock detector enabled, 410.0 - 480.0 MHz band
  {CC1125_PKT_CFG0,          0x00}, // fixed packet length (FIXME: testing only!)
  /* {CC1125_PKT_CFG0,          0x20}, // Variable packet length mode. Packet length configured by the */
  /*                                   // first byte received after sync word */
  {CC1125_PA_CFG0,           0x02}, // ASK/OOK depth, [dBm]
  /* {CC1125_PKT_LEN,           0xFF}, // maximum packet length of 256 bytes */
  {CC1125_PKT_LEN,           0x07}, // maximum packet length of 1 byte (FIXME: testing only!)
  {CC1125_IF_MIX_CFG,        0x00}, // unused lmao
  {CC1125_TOC_CFG,           0x0A},
  {CC1125_FREQ2,             0x6C},
  {CC1125_FREQ1,             0x80},
  {CC1125_FS_DIG1,           0x00},
  {CC1125_FS_DIG0,           0x5F},
  {CC1125_FS_CAL0,           0x0E},
  {CC1125_FS_DIVTWO,         0x03},
  {CC1125_FS_DSM0,           0x33},
  {CC1125_FS_DVC0,           0x17},
  {CC1125_FS_PFD,            0x50},
  {CC1125_FS_PRE,            0x6E},
  {CC1125_FS_REG_DIV_CML,    0x14},
  {CC1125_FS_SPARE,          0xAC},
  {CC1125_XOSC5,             0x0E},
  {CC1125_XOSC3,             0xC7},
  {CC1125_XOSC1,             0x07},
};

void cc1125_write_byte(uint16_t addr, uint8_t data) {
  spi_ss_low();
  // check extended byte
  if ((addr & 0xFF00) == 0x2F00) {
    spi_read_write_byte(0x2F);
  }

  spi_read_write_byte(addr & 0xFF);
  spi_read_write_byte(data);
  spi_ss_high();
}

uint8_t cc1125_read_byte(uint16_t addr) {
  uint8_t ret = 0;

  spi_ss_low();
  // check extended byte, then use extended command strobe
  if ((addr & 0xFF00) == 0x2F00) {
    spi_read_write_byte(0x80 | 0x2F);
    spi_read_write_byte(addr & 0xFF);
    ret = spi_read_write_byte(0x00);
  } else { // else, just use read address
    spi_read_write_byte(0x80 | (addr & 0xFF));
    ret = spi_read_write_byte(0x00);
  }
  spi_ss_high();

  return ret;
}

void cc1125_command_strobe(uint8_t command) {
  if ((0x30 <= command) && (command <= 0x3D)) {
    spi_ss_low();
    spi_read_write_byte(command);
    spi_ss_high();
  } else {
    pr_str(USART_1, "command strobe failed: "); pr_hex(USART_1, command); pr_nl(USART_1);

    error_catch();
  }
}

// configure radio using the preferred settings (generated from SmartRF Studio)
void cc1125_config_radio(void) {
  spi_read_write_byte(0x3D);
  for (uint8_t i = 0; i < sizeof(preferredSettings)/sizeof(registerSetting_t); i++) {
    cc1125_write_byte(preferredSettings[i].addr,
                      preferredSettings[i].data);

    uint8_t check = cc1125_read_byte(preferredSettings[i].addr);

    if (check != preferredSettings[i].data) {
      pr_hex(USART_1, preferredSettings[i].addr); pr_str(USART_1, " ");
      pr_hex(USART_1, preferredSettings[i].data); pr_str(USART_1, " ");
      pr_hex(USART_1, check); pr_nl(USART_1);

      pr_str(USART_1, "config failed."); pr_nl(USART_1);
      error_catch();
    }
  }
}