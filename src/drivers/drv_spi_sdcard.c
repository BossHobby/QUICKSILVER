#include "drv_spi_sdcard.h"

#include <string.h>

#include "drv_spi.h"
#include "drv_time.h"
#include "project.h"

#if defined(STM32F4) && defined(USE_SDCARD)

#define SPI_PORT spi_port_defs[SDCARD_SPI_PORT]
#define NSS_PIN gpio_pin_defs[SDCARD_NSS_PIN]

void sdcard_init() {
  spi_init_pins(SDCARD_SPI_PORT, SDCARD_NSS_PIN);

  spi_enable_rcc(SDCARD_SPI_PORT);

  SPI_I2S_DeInit(SPI_PORT.channel);
  SPI_InitTypeDef SPI_InitStructure;
  SPI_InitStructure.SPI_Direction = SPI_Direction_2Lines_FullDuplex;
  SPI_InitStructure.SPI_Mode = SPI_Mode_Master;
  SPI_InitStructure.SPI_DataSize = SPI_DataSize_8b;
  SPI_InitStructure.SPI_CPOL = SPI_CPOL_Low;
  SPI_InitStructure.SPI_CPHA = SPI_CPHA_1Edge;
  SPI_InitStructure.SPI_NSS = SPI_NSS_Soft;
  SPI_InitStructure.SPI_BaudRatePrescaler = SPI_BaudRatePrescaler_256;
  SPI_InitStructure.SPI_FirstBit = SPI_FirstBit_MSB;
  SPI_InitStructure.SPI_CRCPolynomial = 7;
  SPI_Init(SPI_PORT.channel, &SPI_InitStructure);
  SPI_Cmd(SPI_PORT.channel, ENABLE);

  // Dummy read to clear receive buffer
  while (SPI_I2S_GetFlagStatus(SPI_PORT.channel, SPI_I2S_FLAG_TXE) == RESET)
    ;

  SPI_I2S_ReceiveData(SPI_PORT.channel);

  spi_csn_enable(SDCARD_NSS_PIN);
  for (uint8_t i = 0; i < 20; i++) {
    spi_transfer_byte(SDCARD_SPI_PORT, 0xff);
  }
  spi_csn_disable(SDCARD_NSS_PIN);

  spi_dma_init(SDCARD_SPI_PORT);
}

void sdcard_reinit_fast() {
  SPI_Cmd(SPI_PORT.channel, DISABLE);
  SPI_I2S_DeInit(SPI_PORT.channel);
  SPI_InitTypeDef SPI_InitStructure;
  SPI_InitStructure.SPI_Direction = SPI_Direction_2Lines_FullDuplex;
  SPI_InitStructure.SPI_Mode = SPI_Mode_Master;
  SPI_InitStructure.SPI_DataSize = SPI_DataSize_8b;
  SPI_InitStructure.SPI_CPOL = SPI_CPOL_Low;
  SPI_InitStructure.SPI_CPHA = SPI_CPHA_1Edge;
  SPI_InitStructure.SPI_NSS = SPI_NSS_Soft;
  SPI_InitStructure.SPI_BaudRatePrescaler = SPI_BaudRatePrescaler_4;
  SPI_InitStructure.SPI_FirstBit = SPI_FirstBit_MSB;
  SPI_InitStructure.SPI_CRCPolynomial = 7;
  SPI_Init(SPI_PORT.channel, &SPI_InitStructure);
  SPI_Cmd(SPI_PORT.channel, ENABLE);
}

uint8_t sdcard_build_command(uint8_t *buf, const uint8_t cmd, const uint32_t args) {
  buf[0] = 0x40 | cmd;
  buf[1] = args >> 24;
  buf[2] = args >> 16;
  buf[3] = args >> 8;
  buf[4] = args >> 0;

  buf[5] = 0xff;
  buf[6] = 0xff;
  buf[7] = 0xff;
}

uint8_t sdcard_command(const uint8_t cmd, const uint32_t args) {
  spi_transfer_byte(SDCARD_SPI_PORT, 0x40 | cmd);
  spi_transfer_byte(SDCARD_SPI_PORT, args >> 24);
  spi_transfer_byte(SDCARD_SPI_PORT, args >> 16);
  spi_transfer_byte(SDCARD_SPI_PORT, args >> 8);
  spi_transfer_byte(SDCARD_SPI_PORT, args >> 0);

  // we have to send CRC while we are still in SD Bus mode
  switch (cmd) {
  case SDCARD_GO_IDLE:
    spi_transfer_byte(SDCARD_SPI_PORT, 0x95);
    break;
  case SDCARD_IF_COND:
    spi_transfer_byte(SDCARD_SPI_PORT, 0x87);
    break;
  default:
    spi_transfer_byte(SDCARD_SPI_PORT, 0xff);
  }

  spi_transfer_byte(SDCARD_SPI_PORT, 0xff);
  return spi_transfer_byte(SDCARD_SPI_PORT, 0xff);
}

void sdcard_wait_for_value(uint8_t value) {
  while (spi_transfer_byte(SDCARD_SPI_PORT, 0xff) != value)
    ;
}

uint8_t sdcard_wait_for_ready(uint32_t time) {
  for (uint16_t timeout = time; spi_transfer_byte(SDCARD_SPI_PORT, 0xff) == 0x0; timeout--) {
    if (timeout == 0) {
      return 0;
    }
    timer_delay_us(1);
  }
  return 1;
}

uint32_t sdcard_read_response() {
  return (spi_transfer_byte(SDCARD_SPI_PORT, 0xff) << 24) |
         (spi_transfer_byte(SDCARD_SPI_PORT, 0xff) << 16) |
         (spi_transfer_byte(SDCARD_SPI_PORT, 0xff) << 8) |
         (spi_transfer_byte(SDCARD_SPI_PORT, 0xff) << 0);
}

void sdcard_read_data(uint8_t *buf, const uint32_t size) {
  // wait for data token
  sdcard_wait_for_value(0xfe);

  //spi_dma_transfer_bytes(SDCARD_SPI_PORT, buf, size);
  for (uint32_t i = 0; i < size; i++) {
    buf[i] = spi_transfer_byte(SDCARD_SPI_PORT, 0xff);
  }

  // two bytes CRC
  spi_transfer_byte(SDCARD_SPI_PORT, 0xff);
  spi_transfer_byte(SDCARD_SPI_PORT, 0xff);
}

void sdcard_write_data(const uint8_t token, const uint8_t *buf, const uint32_t size) {
  // start block
  spi_transfer_byte(SDCARD_SPI_PORT, token);

  spi_dma_transfer_bytes(SDCARD_SPI_PORT, (uint8_t *)buf, size);

  // two bytes CRC
  spi_transfer_byte(SDCARD_SPI_PORT, 0xff);
  spi_transfer_byte(SDCARD_SPI_PORT, 0xff);

  // write response
  spi_transfer_byte(SDCARD_SPI_PORT, 0xff);
}

uint8_t sdcard_detect() {
  spi_csn_enable(SDCARD_NSS_PIN);

  // reset the sdcard
  uint8_t ret = sdcard_command(SDCARD_GO_IDLE, 0);
  if (ret != 0x01) {
    spi_csn_disable(SDCARD_NSS_PIN);
    return 0;
  }

  ret = sdcard_command(SDCARD_IF_COND, 0x1AA);
  if (ret == 0x01) {
    uint32_t voltage_check = sdcard_read_response();
    if (voltage_check != 0x1AA) {
      spi_csn_disable(SDCARD_NSS_PIN);
      return 0;
    }

    for (uint8_t i = 0; i < 10; i++) {
      ret = sdcard_command(SDCARD_APP_CMD, 0);
      ret = sdcard_command(SDCARD_ACMD_OD_COND, (1 << 30)); // we support SDHC too

      if (i == 9) {
        spi_csn_disable(SDCARD_NSS_PIN);
        return 0;
      }

      if (ret == 0x0) {
        // we left the idle state
        break;
      }

      timer_delay_us(20);
    }

  } else {
  }

  sdcard_reinit_fast();

  ret = sdcard_command(SDCARD_CID, 0);
  if (ret != 0x0) {
    spi_csn_disable(SDCARD_NSS_PIN);
    return 0;
  }

  sdcard_cid_t cid;
  sdcard_read_data((uint8_t *)&cid, 16);

  ret = sdcard_command(SDCARD_CSD, 0);
  if (ret != 0x0) {
    spi_csn_disable(SDCARD_NSS_PIN);
    return 0;
  }

  uint8_t csd[16];
  sdcard_read_data(csd, 16);

  spi_csn_disable(SDCARD_NSS_PIN);
  return 1;
}

uint8_t sdcard_read_sectors(uint8_t *buf, uint32_t sector, uint32_t count) {
  spi_csn_enable(SDCARD_NSS_PIN);

  // wait for not busy
  sdcard_wait_for_ready(0x400);

  uint8_t ret = sdcard_command(SDCARD_READ_MULTIPLE_BLOCK, sector);
  if (ret != 0x0) {
    spi_csn_disable(SDCARD_NSS_PIN);
    return 0;
  }

  for (uint32_t i = 0; i < count; i++) {
    sdcard_read_data(&buf[i * 512], 512);
  }

  sdcard_command(SDCARD_STOP_TRANSMISSION, 0);

  spi_csn_disable(SDCARD_NSS_PIN);
  return 1;
}

#define COMMAND_SIZE 1
#define TRAILER_SIZE 3

static uint8_t dma_write_buf[2][512 + COMMAND_SIZE + TRAILER_SIZE];
static uint8_t dma_curr_buf = 0;
static uint8_t dma_data_ready = 0;

uint8_t sdcard_start_write_sector(uint32_t sector) {
  if (dma_data_ready == 1) {
    if (!sdcard_wait_for_ready(8)) {
      spi_csn_disable(SDCARD_NSS_PIN);
      return 0;
    }

    spi_dma_transfer_begin(SDCARD_SPI_PORT, (uint8_t *)dma_write_buf[dma_curr_buf], 512 + COMMAND_SIZE + TRAILER_SIZE);
    dma_curr_buf = !dma_curr_buf;
  }

  // start block
  dma_write_buf[dma_curr_buf][0] = 0xFE;
  return 1;
}

void sdcard_continue_write_sector(const uint32_t offset, const uint8_t *buf, const uint32_t size) {
  memcpy(dma_write_buf[dma_curr_buf] + COMMAND_SIZE + offset, buf, size);
}

uint8_t sdcard_finish_write_sector(uint32_t sector) {
  // two bytes CRC
  dma_write_buf[dma_curr_buf][512 + COMMAND_SIZE + 0] = 0xFF;
  dma_write_buf[dma_curr_buf][512 + COMMAND_SIZE + 1] = 0xFF;

  // write response
  dma_write_buf[dma_curr_buf][512 + COMMAND_SIZE + 2] = 0xFF;

  spi_dma_wait_for_ready(SDCARD_SPI_PORT);
  spi_csn_enable(SDCARD_NSS_PIN);

  if (!sdcard_wait_for_ready(8)) {
    spi_csn_disable(SDCARD_NSS_PIN);
    dma_data_ready = 0;
    return 0;
  }

  sdcard_command(SDCARD_WRITE_BLOCK, sector);
  dma_data_ready = 1;

  return 1;
}

void sdcard_dma_rx_isr() {
  spi_csn_disable(SDCARD_NSS_PIN);
}

uint8_t sdcard_write_sectors(const uint8_t *buf, uint32_t sector, uint32_t count) {
  spi_csn_enable(SDCARD_NSS_PIN);

  // wait for not busy
  if (!sdcard_wait_for_ready(0x400)) {
    spi_csn_disable(SDCARD_NSS_PIN);
    return 0;
  }

  // pre-erase all blocks we are about to write to
  /*
  uint8_t ret = 0x0;
  ret = sdcard_command(SDCARD_APP_CMD, 0);
  ret = sdcard_command(SDCARD_ACMD_SET_WR_BLK_ERASE_COUNT, count);
  if (ret != 0x0) {
    spi_csn_disable(SDCARD_NSS_PIN);
    return 0;
  }
  */

  uint8_t ret = sdcard_command(SDCARD_WRITE_MULTIPLE_BLOCK, sector);
  if (ret != 0x0) {
    spi_csn_disable(SDCARD_NSS_PIN);
    return 0;
  }

  if (!sdcard_wait_for_ready(0x400)) {
    spi_csn_disable(SDCARD_NSS_PIN);
    return 0;
  }

  for (uint32_t i = 0; i < count; i++) {
    sdcard_write_data(0xFC, &buf[i * 512], 512);
    if (!sdcard_wait_for_ready(0x400)) {
      spi_csn_disable(SDCARD_NSS_PIN);
      return 0;
    }
  }

  // stop transmission
  spi_transfer_byte(SDCARD_SPI_PORT, 0xfd);
  spi_transfer_byte(SDCARD_SPI_PORT, 0xff);

  spi_csn_disable(SDCARD_NSS_PIN);
  return 1;
}

#endif