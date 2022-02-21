#pragma once

#include <stdbool.h>
#include <stdint.h>

#define OSD_TXN_BUFFER 128
#define OSD_TXN_MAX 16

typedef struct {
  uint8_t attr;

  uint8_t x;
  uint8_t y;

  uint8_t offset;
  uint8_t size;
} osd_segment_t;

typedef struct {
  uint8_t buffer[OSD_TXN_BUFFER];

  osd_segment_t segments[OSD_TXN_MAX];
  uint8_t segment_count;
} osd_transaction_t;

typedef enum {
  OSD_SYS_NONE,
  OSD_SYS_PAL,
  OSD_SYS_NTSC,
} osd_system_t;

typedef enum {
  OSD_DEVICE_NONE,
  OSD_DEVICE_MAX7456,
  OSD_DEVICE_HDZERO
} osd_device_t;

typedef enum {
  OSD_ATTR_TEXT,
  OSD_ATTR_INVERT = (0x1 << 1),
  OSD_ATTR_BLINK = (0x1 << 2),
} osd_text_attr_t;

void osd_device_init(osd_device_t dev);
void osd_intro();

bool osd_is_ready();

void osd_clear();
uint8_t osd_clear_async();

uint8_t osd_check_system();

osd_transaction_t *osd_txn_init();

void osd_txn_start(uint8_t attr, uint8_t x, uint8_t y);

void osd_txn_write_data(const uint8_t *buffer, uint8_t size);
void osd_txn_write_str(const char *buffer);
void osd_txn_write_char(const char val);
void osd_txn_write_uint(uint32_t val, uint8_t width);
void osd_txn_write_int(int32_t val, uint8_t width);
void osd_txn_write_float(float val, uint8_t width, uint8_t precision);

void osd_txn_submit(osd_transaction_t *txn);