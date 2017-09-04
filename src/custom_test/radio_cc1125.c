// This is a personal academic project. Dear PVS-Studio, please check it.
// PVS-Studio Static Code Analyzer for C, C++ and C#: http://www.viva64.com

#include "spi.h"
#include "log.h"

/*****************************************************************************
//  @file   cc112x_spi.c
//
//  @brief  Implementation file for basic and necessary functions
//          to communicate with CC112X over SPI
//
//  Copyright (C) 2013 Texas Instruments Incorporated - http://www.ti.com/
//
//  Redistribution and use in source and binary forms, with or without
//  modification, are permitted provided that the following conditions
//  are met:
//
//    Redistributions of source code must retain the above copyright
//    notice, this list of conditions and the following disclaimer.
//
//    Redistributions in binary form must reproduce the above copyright
//    notice, this list of conditions and the following disclaimer in the
//    documentation and/or other materials provided with the distribution.
//
//    Neither the name of Texas Instruments Incorporated nor the names of
//    its contributors may be used to endorse or promote products derived
//    from this software without specific prior written permission.
//
//  THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS
//  "AS IS" AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT
//  LIMITED TO, THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR
//  A PARTICULAR PURPOSE ARE DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT
//  OWNER OR CONTRIBUTORS BE LIABLE FOR ANY DIRECT, INDIRECT, INCIDENTAL,
//  SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING, BUT NOT
//  LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES; LOSS OF USE,
//  DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER CAUSED AND ON ANY
//  THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY, OR TORT
//  (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE
//  OF THIS SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.
****************************************************************************/

#include "radio_cc1125.h"

#define RADIO_BURST_ACCESS               0x40
#define RADIO_SINGLE_ACCESS              0x00
#define RADIO_READ_ACCESS                0x80
#define RADIO_WRITE_ACCESS               0x00

/* Bit fields in the chip status byte */
#define STATUS_CHIP_RDYn_BM              0x80
#define STATUS_STATE_BM                  0x70
#define STATUS_FIFO_BYTES_AVAILABLE_BM   0x0F

static rf_status_t trx_8b_reg_access(const uint8_t access_type, const uint8_t addr_byte, uint8_t * data, const uint16_t len);
static rf_status_t trx_16b_reg_access(const uint8_t access_type, const uint8_t ext_addr, const uint8_t reg_addr, uint8_t * data, const uint8_t len);
static void trx_read_write_burst_single(const uint8_t addr, uint8_t * data, const uint16_t len);

void cc1125_spi_select_chip(void) {
  spi_ss_low();
}

void cc1125_spi_deselect_chip(void) {
  spi_ss_high();
}

uint32_t cc1125_spi_write(const uint8_t * buf, const uint32_t len) {
  uint8_t status = 0;
  for (uint32_t i = 0; i < len; i++) {
    status = spi_write_byte(buf[i]);
  }

  return status;
}

uint32_t cc1125_spi_read(uint8_t * buf, const uint32_t len) {
  uint8_t status = 0;

    for (uint32_t i = 1; i < len; i++) {
      buf[i] = spi_read_byte();
    }

    return status;
}

uint8_t cc1125_spi_write_read_byte(const uint8_t byte) {
  return spi_write_byte(byte);
}

/******************************************************************************
 * @fn      cc1125_get_tx_status
 *
 * @brief   This function transmits a No Operation Strobe (SNOP) to get the
 *          status of the radio and the number of free bytes in the TX FIFO.
 *
 *          Status byte:
 *
 *  ---------------------------------------------------------------------------
 *  |          |            |                                                 |
 *  | CHIP_RDY | STATE[2:0] | FIFO_BYTES_AVAILABLE (free bytes in the TX FIFO |
 *  |          |            |                                                 |
 *  ---------------------------------------------------------------------------
 *
 *
 * input parameters
 *
 * @param   none
 *
 * output parameters
 *
 * @return  rf_status_t
 *
 */
rf_status_t cc1125_get_tx_status(void)
{
  return cc1125_spi_cmd_strobe(CC1125_SNOP);
}


/******************************************************************************
 *
 *  @fn       cc1125_get_rx_status
 *
 *  @brief
 *            This function transmits a No Operation Strobe (SNOP) with the
 *            read bit set to get the status of the radio and the number of
 *            available bytes in the RXFIFO.
 *
 *            Status byte:
 *
 *  --------------------------------------------------------------------------------
 *  |          |            |                                                      |
 *  | CHIP_RDY | STATE[2:0] | FIFO_BYTES_AVAILABLE (available bytes in the RX FIFO |
 *  |          |            |                                                      |
 *  --------------------------------------------------------------------------------
 *
 *
 * input parameters
 *
 * @param     none
 *
 * output parameters
 *
 * @return    rf_status_t
 *
 */
rf_status_t cc1125_get_rx_status(void)
{
  return cc1125_spi_cmd_strobe(CC1125_SNOP|RADIO_READ_ACCESS);
}


/*******************************************************************************
 * @fn          cc1125_spi_cmd_strobe
 *
 * @brief       Send command strobe to the radio. Returns status byte read
 *              during transfer of command strobe. Validation of provided
 *              is not done. Function assumes chip is ready.
 *
 * input parameters
 *
 * @param       cmd - command strobe
 *
 * output parameters
 *
 * @return      status byte
 */
rf_status_t cc1125_spi_cmd_strobe(const uint8_t cmd)
{
  uint8_t status;

  cc1125_spi_select_chip();

  status = cc1125_spi_write_read_byte(cmd);

  cc1125_spi_deselect_chip();

#if CC1121x_DEBUG == 1
  cc1125_debug(RF_WRITE_CMD, cmd, status);
#endif

  return status;
}


/******************************************************************************
 * @fn          cc1125_spi_read_reg
 *
 * @brief       Read value(s) from config/status/extended radio register(s).
 *              If len  = 1: Reads a single register
 *              if len != 1: Reads len register values in burst mode 
 *
 * input parameters
 *
 * @param       addr - address of first register to read
 * @param       data - pointer to data array where read bytes are saved
 * @param       len  - number of bytes to read
 *
 * output parameters
 *
 * @return      rf_status_t
 */
rf_status_t cc1125_spi_read_reg(const uint16_t addr, uint8_t * data, const uint8_t len)
{
  uint8_t temp_ext;
  uint8_t temp_addr;
  uint8_t rc;

  rc = 0;
  temp_ext  = (uint8_t)(addr >> 8);
  temp_addr = (uint8_t)(addr & 0x00FF);

  /* Checking if this is a FIFO access -> returns chip not ready */
  if (CC1125_SINGLE_TXFIFO <= temp_addr && !temp_ext) {
    rc = STATUS_CHIP_RDYn_BM;
  } else {

    /* Decide what register space is accessed */
    if (!temp_ext) {
      rc = trx_8b_reg_access(RADIO_BURST_ACCESS|RADIO_READ_ACCESS, temp_addr, data, len);
    } else if (temp_ext == 0x2F) {
      rc = trx_16b_reg_access(RADIO_BURST_ACCESS|RADIO_READ_ACCESS, temp_ext, temp_addr, data, len);
    }
  }

#if CC1121x_DEBUG == 1
  cc1125_debug(!temp_ext ? RF_READ_REG_8B : RF_READ_REG_16B, RADIO_BURST_ACCESS|RADIO_READ_ACCESS, rc);
#endif

  return rc;
}


/******************************************************************************
 * @fn          cc1125_spi_write_reg
 *
 * @brief       Write value(s) to config/status/extended radio register(s).
 *              If len  = 1: Writes a single register
 *              if len  > 1: Writes len register values in burst mode 
 *
 * input parameters
 *
 * @param       addr - address of first register to write
 * @param       data - pointer to data array that holds bytes to be written
 * @param       len  - number of bytes to write
 *
 * output parameters
 *
 * @return      rf_status_t
 */
rf_status_t cc1125_spi_write_reg(const uint16_t addr, uint8_t * data, const uint8_t len)
{
  uint8_t temp_ext;
  uint8_t temp_addr;
  uint8_t rc;

  rc = 0;
  temp_ext  = (uint8_t)(addr >> 8);
  temp_addr = (uint8_t)(addr & 0x00FF);
  
  /* Checking if this is a FIFO access - returns chip not ready */
  if (CC1125_SINGLE_TXFIFO <= temp_addr && !temp_ext) {
    rc = STATUS_CHIP_RDYn_BM;
  } else {
    /* Decide what register space is accessed */
    if (!temp_ext) {
      rc = trx_8b_reg_access(RADIO_BURST_ACCESS|RADIO_WRITE_ACCESS, temp_addr, data, len);
    } else if (temp_ext == 0x2F) {
      rc = trx_16b_reg_access(RADIO_BURST_ACCESS|RADIO_WRITE_ACCESS, temp_ext, temp_addr, data, len);
    }
  }

#if CC1121x_DEBUG == 1
  cc1125_debug(!temp_ext ? RF_WRITE_REG_8B : RF_WRITE_REG_16B, RADIO_BURST_ACCESS|RADIO_WRITE_ACCESS, rc);
#endif

  return rc;
}


/*******************************************************************************
 * @fn          cc1125_spi_write_tx_fifo
 *
 * @brief       Write data to radio transmit FIFO.
 *
 * input parameters
 *
 * @param       data - pointer to data array that is written to TX FIFO
 * @param       len  - Length of data array to be written
 *
 * output parameters
 *
 * @return      rf_status_t
 */
rf_status_t cc1125_spi_write_tx_fifo(uint8_t * data, const uint8_t len)
{
  return trx_8b_reg_access(0, CC1125_BURST_TXFIFO, data, len);
}


/*******************************************************************************
 * @fn          cc1125_spi_read_rx_fifo
 *
 * @brief       Reads RX FIFO values to data array
 *
 * input parameters
 *
 * @param       data - pointer to data array where RX FIFO bytes are saved
 * @param       len  - number of bytes to read from the RX FIFO
 *
 * output parameters
 *
 * @return      rf_status_t
 */
rf_status_t cc1125_spi_read_rx_fifo(uint8_t * data, const uint8_t len)
{
  return trx_8b_reg_access(0, CC1125_BURST_RXFIFO, data, len);
}


/*******************************************************************************
 * @fn          trx_8b_reg_access
 *
 * @brief       This function performs a read or write from/to a 8bit register
 *              address space. The function handles burst and single read/write
 *              as specified in addrByte. Function assumes that chip is ready.
 *
 * input parameters
 *
 * @param       access_type - Specifies if this is a read or write and if it's
 *                            a single or burst access. Bitmask made up of
 *                            RADIO_BURST_ACCESS/RADIO_SINGLE_ACCESS/
 *                            RADIO_WRITE_ACCESS/RADIO_READ_ACCESS.
 * @param       addr_byte - address byte of register.
 * @param       data      - data array
 * @param       len       - Length of array to be read(TX)/written(RX)
 *
 * output parameters
 *
 * @return      chip status
 */
static rf_status_t trx_8b_reg_access(const uint8_t access_type, const uint8_t addr_byte, uint8_t * data, const uint16_t len)
{
  uint8_t status;

  cc1125_spi_select_chip();

  status = cc1125_spi_write_read_byte(access_type|addr_byte);
  trx_read_write_burst_single(access_type|addr_byte, data, len);

  cc1125_spi_deselect_chip();

  return status;
}


/******************************************************************************
 * @fn          trx_16b_reg_access
 *
 * @brief       This function performs a read or write in the extended adress
 *              space of CC1125.
 *
 * input parameters
 *
 * @param       access_type - Specifies if this is a read or write and if it's
 *                            a single or burst access. Bitmask made up of
 *                            RADIO_BURST_ACCESS/RADIO_SINGLE_ACCESS/
 *                            RADIO_WRITE_ACCESS/RADIO_READ_ACCESS.
 * @param       ext_addr - Extended register space address = 0x2F.
 * @param       reg_addr - Register address in the extended address space.
 * @param       data     - Pointer to data array for communication
 * @param       len      - Length of bytes to be read/written from/to radio
 *
 * output parameters
 *
 * @return      rf_status_t
 */
static rf_status_t trx_16b_reg_access(const uint8_t access_type, const uint8_t ext_addr, const uint8_t reg_addr, uint8_t * data, const uint8_t len)
{
  uint8_t status;

  cc1125_spi_select_chip();

  status = cc1125_spi_write_read_byte(access_type|ext_addr);
  cc1125_spi_write(&reg_addr, 1);
  trx_read_write_burst_single(access_type|ext_addr, data, len);

  cc1125_spi_deselect_chip();

  return status;
}

/*******************************************************************************
 * @fn          trx_read_write_burst_single
 *
 * @brief       When the address byte is sent to the SPI slave, the next byte
 *              communicated is the data to be written or read. The address
 *              byte that holds information about read/write -and single/
 *              burst-access is provided to this function.
 *
 *              Depending on these two bits this function will write len bytes to
 *              the radio in burst mode or read len bytes from the radio in burst
 *              mode if the burst bit is set. If the burst bit is not set, only
 *              one data byte is communicated.
 *
 * output parameters
 *
 * @return      void
 */
static void trx_read_write_burst_single(const uint8_t addr, uint8_t * data, const uint16_t len)
{
  if(addr & RADIO_READ_ACCESS) {
    cc1125_spi_read(data, len);
  } else {
    cc1125_spi_write(data, len);
  }
}
