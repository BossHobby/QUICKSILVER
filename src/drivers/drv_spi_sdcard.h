#pragma once

#include <stdint.h>

enum sdcard_commands {
  SDCARD_GO_IDLE = 0,
  SDCARD_IF_COND = 8,
  SDCARD_CSD = 9,
  SDCARD_CID = 10,

  SDCARD_STOP_TRANSMISSION = 12,

  SDCARD_READ_BLOCK = 17,
  SDCARD_READ_MULTIPLE_BLOCK = 18,

  SDCARD_WRITE_BLOCK = 24,
  SDCARD_WRITE_MULTIPLE_BLOCK = 25,

  SDCARD_APP_CMD = 55,

  SDCARD_ACMD_SET_WR_BLK_ERASE_COUNT = 23,
  SDCARD_ACMD_OD_COND = 41,
};

typedef struct {
  uint8_t manufacturer_id;
  uint16_t application_id;
  uint8_t name[5];
  uint8_t revision;
  uint32_t serial;
  uint16_t manufacturing_date;
  uint8_t crc;
} __attribute((packed)) sdcard_cid_t;

void sdcard_init();
uint8_t sdcard_command(const uint8_t cmd, const uint32_t args);
uint8_t sdcard_detect();

uint8_t sdcard_read_sectors(uint8_t *buf, uint32_t sector, uint32_t count);

uint8_t sdcard_start_write_sector(uint32_t sector);
void sdcard_continue_write_sector(const uint32_t offset, const uint8_t *buf, const uint32_t size);
uint8_t sdcard_finish_write_sector(uint32_t sector);

uint8_t sdcard_write_sectors(const uint8_t *buf, uint32_t sector, uint32_t count);