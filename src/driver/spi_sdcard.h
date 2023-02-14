#pragma once

#include <stdint.h>

#include "io/data_flash.h"

#define SDCARD_PAGE_SIZE 512

typedef enum {
  SDCARD_GO_IDLE = 0,
  SDCARD_IF_COND = 8,
  SDCARD_CSD = 9,
  SDCARD_CID = 10,
  SDCARD_OCR = 58,

  SDCARD_STOP_TRANSMISSION = 12,

  SDACARD_SET_BLOCK_LEN = 16,

  SDCARD_READ_BLOCK = 17,
  SDCARD_READ_MULTIPLE_BLOCK = 18,

  SDCARD_WRITE_BLOCK = 24,
  SDCARD_WRITE_MULTIPLE_BLOCK = 25,

  SDCARD_APP_CMD = 55,

  SDCARD_ACMD_SET_WR_BLK_ERASE_COUNT = 23,
  SDCARD_ACMD_OD_COND = 41,
} sdcard_commands_t;

typedef enum {
  SDCARD_R1_IDLE = 1,
  SDCARD_R1_ERASE_RESET = 2,
  SDCARD_R1_ILLEGAL_COMMAND = 4,
  SDCARD_R1_COM_CRC_ERROR = 8,
  SDCARD_R1_ERASE_SEQUENCE_ERROR = 16,
  SDCARD_R1_ADDRESS_ERROR = 32,
  SDCARD_R1_PARAMETER_ERROR = 64,
} sdcard_r1_t;

typedef struct {
  uint8_t manufacturer_id;
  uint16_t application_id;
  uint8_t name[5];
  uint8_t revision;
  uint32_t serial;
  uint16_t manufacturing_date;
  uint8_t crc;
} __attribute((packed)) sdcard_cid_t;

typedef struct {
  uint32_t CSD_STRUCTURE_VER : 2;

  union {
    struct {
      uint8_t TAAC : 8;
      uint8_t NSAC : 8;
      uint8_t TRAN_SPEED : 8;
      uint16_t CCC : 12;
      uint8_t READ_BL_LEN : 4;
      uint8_t READ_BL_PARTIAL : 1;
      uint8_t WRITE_BLK_MISALIGN : 1;
      uint8_t READ_BLK_MISALIGN : 1;
      uint8_t DSR_IMP : 1;
      uint16_t C_SIZE : 12;
      uint8_t VDD_R_CURR_MIN : 3;
      uint8_t VDD_R_CURR_MAX : 3;
      uint8_t VDD_W_CURR_MIN : 3;
      uint8_t VDD_W_CURR_MAX : 3;
      uint8_t C_SIZE_MULT : 3;
      uint8_t ERASE_BLK_EN : 1;
      uint8_t SECTOR_SIZE : 7;
      uint8_t WP_GRP_SIZE : 7;
      uint8_t WP_GRP_ENABLE : 1;
      uint8_t R2W_FACTOR : 3;
      uint8_t WRITE_BL_LEN : 4;
      uint8_t WRITE_BL_PARTIAL : 1;
      uint8_t FILE_FORMAT_GRP : 1;
      uint8_t COPY : 1;
      uint8_t PERM_WRITE_PROTECT : 1;
      uint8_t TMP_WRITE_PROTECT : 1;
      uint8_t FILE_FORMAT : 2;
      uint8_t CSD_CRC : 8;
    } v1;

    struct {
      uint8_t TAAC : 8;
      uint8_t NSAC : 8;
      uint8_t TRAN_SPEED : 8;
      uint16_t CCC : 12;
      uint8_t READ_BL_LEN : 4;
      uint8_t READ_BL_PARTIAL : 1;
      uint8_t WRITE_BLK_MISALIGN : 1;
      uint8_t READ_BLK_MISALIGN : 1;
      uint8_t DSR_IMP : 1;
      uint32_t C_SIZE : 22;
      uint8_t ERASE_BLK_EN : 1;
      uint8_t SECTOR_SIZE : 7;
      uint8_t WP_GRP_SIZE : 7;
      uint8_t WP_GRP_ENABLE : 1;
      uint8_t R2W_FACTOR : 3;
      uint8_t WRITE_BL_LEN : 4;
      uint8_t WRITE_BL_PARTIAL : 1;
      uint8_t FILE_FORMAT_GRP : 1;
      uint8_t COPY : 1;
      uint8_t PERM_WRITE_PROTECT : 1;
      uint8_t TMP_WRITE_PROTECT : 1;
      uint8_t FILE_FORMAT : 2;
      uint8_t CSD_CRC : 8;
    } v2;
  };
} sdcard_csd_t;

typedef struct {
  uint8_t version;

  uint8_t high_capacity;

  uint32_t ocr;
  sdcard_cid_t cid;
  sdcard_csd_t csd;

} sdcard_info_t;

typedef enum {
  SDCARD_WAIT,
  SDCARD_ERROR,
  SDCARD_IDLE,
} sdcard_status_t;

void sdcard_init();

sdcard_status_t sdcard_update();

void sdcard_get_bounds(data_flash_bounds_t *bounds);

uint8_t sdcard_read_pages(uint8_t *buf, uint32_t page, uint32_t count);
uint8_t sdcard_write_page(uint8_t *buf, uint32_t page);

uint8_t sdcard_write_pages_start(uint32_t page, uint32_t count);
uint8_t sdcard_write_pages_continue(uint8_t *buf);
uint8_t sdcard_write_pages_finish();
