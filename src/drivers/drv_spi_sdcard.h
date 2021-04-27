#pragma once

#include <stdint.h>

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

typedef union {
  uint8_t raw[16];

  struct {
    uint32_t CSD_STRUCTURE_VER : 2;
    uint32_t _RESERVED1 : 6;
    uint32_t TAAC : 8;
    uint32_t NSAC : 8;
    uint32_t TRAN_SPEED : 8;
    uint32_t CCC : 12;
    uint32_t READ_BLOCK_LEN : 4;
    uint32_t READ_BLOCK_PARTIAL_ALLOWED : 1;
    uint32_t WRITE_BLOCK_MISALIGN : 1;
    uint32_t READ_BLOCK_MISALIGN : 1;
    uint32_t DSR_IMPLEMENTED : 1;
    uint32_t _RESERVED2 : 2;
    uint32_t CSIZE : 12;
    uint32_t VDD_READ_CURR_MIN : 3;
    uint32_t VDD_READ_CURR_MAX : 3;
    uint32_t VDD_WRITE_CURR_MIN : 3;
    uint32_t VDD_WRITE_CURR_MAX : 3;
    uint32_t CSIZE_MULT : 3;
    uint32_t ERASE_SINGLE_BLOCK_ALLOWED : 1;
    uint32_t SECTOR_SIZE : 7;
    uint32_t WRITE_PROTECT_GROUP_SIZE : 7;
    uint32_t WRITE_PROTECT_GROUP_ENABLE : 1;
    uint32_t _RESERVED3 : 2;
    uint32_t R2W_FACTOR : 3;
    uint32_t WRITE_BLOCK_LEN : 4;
    uint32_t WRITE_BLOCK_PARTIAL_ALLOWED : 1;
    uint32_t FILE_FORMAT_GROUP : 1;
    uint32_t COPY : 1;
    uint32_t PERMANENT_WRITE_PROTECT : 1;
    uint32_t TEMPORARY_WRITE_PROTECT : 1;
    uint32_t FILE_FORMAT : 2;
    uint32_t _RESERVED4 : 2;
    uint32_t CRC_VAL : 7;
    uint32_t TRAILER : 1;
  } v1;

  struct {
    uint32_t CSD_STRUCTURE_VER : 2;
    uint32_t _RESERVED1 : 6;
    uint32_t TAAC : 8;
    uint32_t NSAC : 8;
    uint32_t TRAN_SPEED : 8;
    uint32_t CCC : 12;
    uint32_t READ_BLOCK_LEN : 4;
    uint32_t READ_BLOCK_PARTIAL_ALLOWED : 1;
    uint32_t WRITE_BLOCK_MISALIGN : 1;
    uint32_t READ_BLOCK_MISALIGN : 1;
    uint32_t DSR_IMPLEMENTED : 1;
    uint32_t _RESERVED2 : 6;
    uint32_t CSIZE : 22;
    uint32_t _RESERVED3 : 1;
    uint32_t ERASE_SINGLE_BLOCK_ALLOWED : 1;
    uint32_t SECTOR_SIZE : 7;
    uint32_t WRITE_PROTECT_GROUP_SIZE : 7;
    uint32_t WRITE_PROTECT_GROUP_ENABLE : 1;
    uint32_t R2W_FACTOR : 3;
    uint32_t WRITE_BLOCK_LEN : 4;
    uint32_t WRITE_BLOCK_PARTIAL_ALLOWED : 1;
    uint32_t _RESERVED4 : 5;
    uint32_t FILE_FORMAT_GROUP : 1;
    uint32_t COPY : 1;
    uint32_t PERMANENT_WRITE_PROTECT : 1;
    uint32_t TEMPORARY_WRITE_PROTECT : 1;
    uint32_t FILE_FORMAT : 2;
    uint32_t _RESERVED5 : 2;
    uint32_t CRC_VAL : 7;
    uint32_t TRAILER : 1;
  } v2;

} sdcard_csd_t;

typedef struct {
  uint8_t version;

  uint32_t block_count;
  uint8_t high_capacity;

  uint32_t ocr;
  sdcard_cid_t cid;
  sdcard_csd_t csd;

} sdcard_info_t;

void sdcard_init();

uint8_t sdcard_update();

uint8_t sdcard_read_pages(uint8_t *buf, uint32_t page, uint32_t count);
uint8_t sdcard_write_page(uint8_t *buf, uint32_t page);

uint8_t sdcard_write_pages_start(uint32_t page, uint32_t count);
uint8_t sdcard_write_pages_continue(uint8_t *buf);
uint8_t sdcard_write_pages_finish();
