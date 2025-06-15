#pragma once

#include <stdbool.h>
#include <stdint.h>

#define MAX7456_COLS 30
#define MAX7456_ROWS 16

#define DISPLAYPORT_ROWS 18
#define DISPLAYPORT_COLS 50

#define HD_ROWS DISPLAYPORT_ROWS
#define HD_COLS DISPLAYPORT_COLS

#define SD_COLS MAX7456_COLS
#define SD_ROWS MAX7456_ROWS

#define OSD_TXN_BUFFER 128
#define OSD_TXN_MAX 16

typedef struct {
  uint8_t attr;

  uint8_t x;
  uint8_t y;

  uint8_t offset;
} osd_segment_t;

typedef struct {
  uint8_t val;
  uint8_t attr;
} osd_char_t;

typedef enum {
  OSD_SYS_NONE,
  OSD_SYS_PAL,
  OSD_SYS_NTSC,
  OSD_SYS_HD,
} osd_system_t;

typedef enum {
  OSD_DEVICE_NONE,
  OSD_DEVICE_MAX7456,
  OSD_DEVICE_DISPLAYPORT,
  OSD_DEVICE_SIMULATOR,
} osd_device_t;

typedef enum {
  OSD_ATTR_TEXT,
  OSD_ATTR_INVERT = (0x1 << 1),
  OSD_ATTR_BLINK = (0x1 << 2),
} osd_text_attr_t;

void osd_device_init();
void osd_intro();

bool osd_is_ready();
bool osd_update();

void osd_clear();
bool osd_clear_async();

void osd_display_refresh();

osd_system_t osd_check_system();

void osd_start(uint8_t attr, uint8_t x, uint8_t y);

void osd_write_data(const uint8_t *buffer, uint8_t size);
void osd_write_str(const char *buffer);
void osd_write_char(const char val);
void osd_write_uint(uint32_t val, uint8_t width);
void osd_write_int(int32_t val, uint8_t width);
void osd_write_float(float val, uint8_t width, uint8_t precision);