
#include "radio_cc1125.h"
#include "spi.h"
#include "printf.h"

typedef struct registerSetting_t {
  uint16_t addr;
  uint8_t data;
} registerSetting_t;

/******************************************************************************
 * VARIABLES
 */  
// Address config = No address check 
// Packet length = 255 
// Modulation format = 2-GFSK 
// PA ramping = true 
// Packet length mode = Variable 
// Bit rate = 0.6 
// Deviation = 1.499176 
// Packet bit length = 0 
// Performance mode = High Performance 
// Carrier frequency = 470.000000 
// RX filter BW = 7.812500 
// Manchester enable = false 
// Symbol rate = 0.6 
// TX power = 15 
// Device address = 0 
// Whitening = false 
static const registerSetting_t preferredSettings470[]= {
  {CC112X_IOCFG3,            0xB0},
  {CC112X_IOCFG2,            0x06},
  {CC112X_IOCFG1,            0xB0},
  {CC112X_IOCFG0,            0x40},
  {CC112X_SYNC_CFG1,         0x08},
  {CC112X_DEVIATION_M,       0x89},
  {CC112X_MODCFG_DEV_E,      0x09},
  {CC112X_DCFILT_CFG,        0x1C},
  {CC112X_IQIC,              0xC6},
  {CC112X_CHAN_BW,           0x50},
  {CC112X_SYMBOL_RATE2,      0x33},
  {CC112X_AGC_REF,           0x20},
  {CC112X_AGC_CS_THR,        0x19},
  {CC112X_AGC_CFG1,          0xA9},
  {CC112X_AGC_CFG0,          0xCF},
  {CC112X_FIFO_CFG,          0x00},
  {CC112X_SETTLING_CFG,      0x03},
  {CC112X_FS_CFG,            0x14},
  {CC112X_PKT_CFG0,          0x20},
  {CC112X_PA_CFG0,           0x7E},
  {CC112X_PKT_LEN,           0xFF},
  {CC112X_IF_MIX_CFG,        0x00},
  {CC112X_FREQOFF_CFG,       0x30},
  {CC112X_FREQ2,             0x75},
  {CC112X_FREQ1,             0x80},
  {CC112X_FS_DIG1,           0x00},
  {CC112X_FS_DIG0,           0x5F},
  {CC112X_FS_CAL1,           0x40},
  {CC112X_FS_CAL0,           0x0E},
  {CC112X_FS_DIVTWO,         0x03},
  {CC112X_FS_DSM0,           0x33},
  {CC112X_FS_DVC0,           0x17},
  {CC112X_FS_PFD,            0x50},
  {CC112X_FS_PRE,            0x6E},
  {CC112X_FS_REG_DIV_CML,    0x14},
  {CC112X_FS_SPARE,          0xAC},
  {CC112X_FS_VCO0,           0xB4},
  {CC112X_LNA,               0x03},
  {CC112X_XOSC5,             0x0E},
  {CC112X_XOSC1,             0x03},
};

// command strobe from 0x30 to 0x3D
void cc1125_send_command_strobe(uint8_t cmd) {
  printf("in cc1125_command_strobe %x\r\n", cmd);
  /* spi_write_byte(cmd); */
};

void cc1125_send_byte(uint16_t addr, uint8_t data) {
  printf("in cc1125_send_byte %x, %x\r\n", addr, data);
  // if extended register space address
  if (addr >> 8 == 0x2F) {
    spi_write_byte(0x2F);
  }
  spi_write_byte(addr & 0xFF);

  spi_write_byte(data);
}

uint8_t cc1125_read_reg(uint16_t addr) {
  printf("in cc1125_read_reg %x\r\n", addr);
  if (addr >> 8 == 0x2F) {
    spi_write_byte(0x2F);
  }
  spi_write_byte(addr & 0xFF);

  return spi_read_byte();
}

void cc1125_config_regs(void) {
  printf("in cc1125_config_regs\r\n");
  /* cc1125_send_command_strobe(CC112X_SRES); */
  cc1125_send_command_strobe(0x5A);
  /* for(uint16_t i = 0; */
  /*     /\* i < (sizeof(preferredSettings470)/sizeof(registerSetting_t)); *\/ */
  /*     i < 40; */
  /*     i++) { */
  /*   cc1125_send_byte(preferredSettings470[i].addr, */
  /*                    preferredSettings470[i].data); */
  /*   printf("wrote byte: %x, %x\r\n", preferredSettings470[i].addr, */
  /*                                    preferredSettings470[i].data); */
  /* } */
}

#define VCDAC_START_OFFSET 2
#define FS_VCO2_INDEX 0
#define FS_VCO4_INDEX 1
#define FS_CHP_INDEX 2
void cc1125_manual_calibrate(void) {
  uint8_t original_fs_cal2;
  uint8_t calResults_for_vcdac_start_high[3];
  uint8_t calResults_for_vcdac_start_mid[3];
  uint8_t marcstate;

  // 1) Set VCO cap-array to 0 (FS_VCO2 = 0x00)
  cc1125_send_byte(CC112X_FS_VCO2, 0x00);

  // 2) Start with high VCDAC (original VCDAC_START + 2):
  original_fs_cal2 = cc1125_read_reg(CC112X_FS_CAL2);
  cc1125_send_byte(CC112X_FS_CAL2, original_fs_cal2 + VCDAC_START_OFFSET);

  // 3) Calibrate and wait for calibration to be done
  //   (radio back in IDLE state)
  cc1125_send_command_strobe(CC112X_SCAL);

  do {
    marcstate = cc1125_read_reg(CC112X_MARCSTATE);
  } while (marcstate != 0x41);

  // 4) Read FS_VCO2, FS_VCO4 and FS_CHP register obtained with 
  //    high VCDAC_START value
  calResults_for_vcdac_start_high[FS_VCO2_INDEX] = cc1125_read_reg(CC112X_FS_VCO2);
  calResults_for_vcdac_start_high[FS_VCO4_INDEX] = cc1125_read_reg(CC112X_FS_VCO4);
  calResults_for_vcdac_start_high[FS_CHP_INDEX] = cc1125_read_reg(CC112X_FS_CHP);

  // 5) Set VCO cap-array to 0 (FS_VCO2 = 0x00)
  cc1125_send_byte(CC112X_FS_VCO2, 0x00);

  // 6) Continue with mid VCDAC (original VCDAC_START):
  cc1125_send_byte(CC112X_FS_CAL2, original_fs_cal2);

  // 7) Calibrate and wait for calibration to be done
  //   (radio back in IDLE state)
  cc1125_send_command_strobe(CC112X_SCAL);

  do {
    marcstate = cc1125_read_reg(CC112X_MARCSTATE);
  } while (marcstate != 0x41);

  // 8) Read FS_VCO2, FS_VCO4 and FS_CHP register obtained 
  //    with mid VCDAC_START value
  calResults_for_vcdac_start_mid[FS_VCO2_INDEX] = cc1125_read_reg(CC112X_FS_VCO2);
  calResults_for_vcdac_start_mid[FS_VCO4_INDEX] = cc1125_read_reg(CC112X_FS_VCO4);
  calResults_for_vcdac_start_mid[FS_CHP_INDEX] = cc1125_read_reg(CC112X_FS_CHP);

  // 9) Write back highest FS_VCO2 and corresponding FS_VCO
  //    and FS_CHP result
  if (calResults_for_vcdac_start_high[FS_VCO2_INDEX] >
      calResults_for_vcdac_start_mid[FS_VCO2_INDEX]) {
    cc1125_send_byte(CC112X_FS_VCO2, calResults_for_vcdac_start_high[FS_VCO2_INDEX]);
    cc1125_send_byte(CC112X_FS_VCO4, calResults_for_vcdac_start_high[FS_VCO4_INDEX]);
    cc1125_send_byte(CC112X_FS_CHP, calResults_for_vcdac_start_high[FS_CHP_INDEX]);
  } else {
    cc1125_send_byte(CC112X_FS_VCO2, calResults_for_vcdac_start_mid[FS_VCO2_INDEX]);
    cc1125_send_byte(CC112X_FS_VCO4, calResults_for_vcdac_start_mid[FS_VCO4_INDEX]);
    cc1125_send_byte(CC112X_FS_CHP, calResults_for_vcdac_start_mid[FS_CHP_INDEX]);
  }
}

/* void runTX(void) { */
/*   uint8 writeByte; */
/*   uint8 readByte; */

/*   // Initialize packet buffer */
/*   uint8 txBuffer[4] = {0}; */

/*   // Configure register settings */
/*   registerConfig((uint8)(*pMenuTable[FREQ_BAND].pValue)); */

/*   // Write custom register settings to radio (force fixed packet length in TX) */
/*   cc112xSpiReadReg(CC112X_PKT_CFG0, &readByte, 1); */
/*   writeByte = (readByte & 0x9F) | 0x00; */
/*   cc112xSpiWriteReg(CC112X_PKT_CFG0, &writeByte, 1); */

/*   // Calibrate radio according to errata */
/*   manualCalibration(); */

/*   // Infinite loop */
/*   while(TRUE) { */
/*     do { */

/*       txBuffer[0] = 0x55; // Dummy byte in packet 1 */
/*       txBuffer[1] = (uint8_t)(++packetCounter >> 8); // Seq. Number in packet 2 */
/*       txBuffer[2] = (uint8_t)(packetCounter);  */
/*       txBuffer[3] = 0x55; */

/*       // Write both packets to TX FIFO */
/*       cc112xSpiWriteTxFifo(txBuffer, sizeof(txBuffer)); */

/*       // Reg. Config for packet 1 */
/*       writeByte = 0x01; cc112xSpiWriteReg(CC112X_PKT_LEN,  &writeByte, 1); */

/*       // SYNC_1 */
/*       writeByte = 0x26; cc112xSpiWriteReg(CC112X_SYNC3, &writeByte, 1); */
/*       writeByte = 0x33; cc112xSpiWriteReg(CC112X_SYNC2, &writeByte, 1); */
/*       writeByte = 0xD9; cc112xSpiWriteReg(CC112X_SYNC1, &writeByte, 1); */
/*       writeByte = 0xCC; cc112xSpiWriteReg(CC112X_SYNC0, &writeByte, 1); */
    
/*       // Strobe TX to send packet 1 */
/*       //--------------------------------------------------------- */
/*       // 0xAA 0xAA 0xAA |  0x26 0x33 0xD9 0xCC | 0x55 | CRC CRC | */
/*       //--------------------------------------------------------- */
/*       trxSpiCmdStrobe(CC112X_STX); */

/*       // Wait for interrupt that packet has been sent. */
/*       // (Assumes the GPIO connected to the radioRxTxISR function is */
/*       // set to GPIOx_CFG = 0x06) */
/*       while(packetSemaphore != ISR_ACTION_REQUIRED); */

/*       // Clear semaphore flag */
/*       packetSemaphore = ISR_IDLE; */
    
/*       // Reg. Config for packet 2 */
/*       writeByte = 0x03; cc112xSpiWriteReg(CC112X_PKT_LEN,  &writeByte, 1); */
       
/*       // SYNC_2 */
/*       writeByte = 0x93; cc112xSpiWriteReg(CC112X_SYNC3, &writeByte, 1); */
/*       writeByte = 0x0B; cc112xSpiWriteReg(CC112X_SYNC2, &writeByte, 1); */
/*       writeByte = 0x51; cc112xSpiWriteReg(CC112X_SYNC1, &writeByte, 1); */
/*       writeByte = 0xDE; cc112xSpiWriteReg(CC112X_SYNC0, &writeByte, 1); */

/*       // Strobe TX to send packet 2 */
/*       //-------------------------------------------------------------------- */
/*       // 0xAA 0xAA 0xAA |  0x93 0x0B 0x51 0xDE | Seq. Seq. 0x55  | CRC CRC | */
/*       //-------------------------------------------------------------------- */
/*       trxSpiCmdStrobe(CC112X_STX); */

/*       // Wait for interrupt that packet has been sent. */
/*       // (Assumes the GPIO connected to the radioRxTxISR function is */
/*       // set to GPIOx_CFG = 0x06) */
/*       while(packetSemaphore != ISR_ACTION_REQUIRED); */

/*       // Clear semaphore flag */
/*       packetSemaphore = ISR_IDLE; */
    
/*       updateLcdTx((int32)packetCounter); */
    
/*     } while (bspKeyPushed(BSP_KEY_ALL) == 0); */

/*   } */
/* } */
