#pragma once

#include <stdbool.h>
#include <stdint.h>

#include <FreeRTOS.h>
#include <cbor.h>

#define MAX7456_COLS 30
#define MAX7456_ROWS 16

// DisplayPort canvas used until the goggle announces its own via
// MSP_SET_OSD_CANVAS. DJI O3/O4 never announce and use this grid natively.
static constexpr uint8_t DISPLAYPORT_COLS = 53;
static constexpr uint8_t DISPLAYPORT_ROWS = 20;

// Largest canvas we render, Betaflight's limit. Columns must fit the 64 bit
// dirty mask.
static constexpr uint8_t HD_COLS_MAX = 63;
static constexpr uint8_t HD_ROWS_MAX = 31;

// SD layouts use the PAL grid; a screen with fewer rows, NTSC, moves rows
// past 12 up by two.
#define SD_COLS MAX7456_COLS
#define SD_ROWS MAX7456_ROWS
static constexpr uint8_t SD_ROWS_NTSC = 13;

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

typedef struct {
  // camera signal on analog, goggles polling on DisplayPort
  uint8_t connected;
  uint8_t cols;
  uint8_t rows;
} osd_screen_t;

#define OSD_SCREEN_MEMBERS   \
  START_STRUCT(osd_screen_t) \
  MEMBER(connected, uint8_t) \
  MEMBER(cols, uint8_t)      \
  MEMBER(rows, uint8_t)      \
  END_STRUCT()

// Written by the OSD thread, read by the configurator.
extern osd_screen_t osd_screen;

static inline bool osd_is_hd() {
  return osd_screen.cols > SD_COLS;
}

// Rows the layout is made for; an SD screen may show fewer.
static inline uint8_t osd_layout_rows() {
  return osd_is_hd() ? osd_screen.rows : SD_ROWS;
}

void osd_device_init();
void osd_intro();

// Tighten the caller's deadline for periodic transport/display work.
bool osd_is_ready(TickType_t *wait = nullptr);
bool osd_update(TickType_t *wait = nullptr);
void osd_notify_from_isr(void *);

void osd_clear();
bool osd_clear_async();

void osd_display_refresh();

void osd_check_screen();

void osd_start(uint8_t attr, uint8_t x, uint8_t y);

void osd_write_data(const uint8_t *buffer, uint8_t size);
void osd_write_str(const char *buffer);
void osd_write_char(const char val);
void osd_write_uint(uint32_t val, uint8_t width);
void osd_write_int(int32_t val, uint8_t width);
void osd_write_float(float val, uint8_t width, uint8_t precision);

cbor_result_t cbor_encode_osd_screen_t(cbor_value_t *enc, const osd_screen_t *o);
