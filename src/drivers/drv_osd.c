#include "drv_osd.h"

#include <string.h>

#include "util.h"

static osd_transaction_t osd_txn;

osd_transaction_t *osd_txn_init() {
  osd_txn.segment_count = 0;
  return &osd_txn;
}

void osd_txn_start(uint8_t attr, uint8_t x, uint8_t y) {
  uint8_t offset = 0;
  if (osd_txn.segment_count) {
    osd_segment_t *last_seg = &osd_txn.segments[osd_txn.segment_count - 1];
    offset = last_seg->offset + last_seg->size;
  }

  osd_segment_t *seg = &osd_txn.segments[osd_txn.segment_count];
  seg->attr = attr;
  seg->x = x;
  seg->y = y;
  seg->offset = offset;
  seg->size = 0;

  osd_txn.segment_count++;
}

void osd_txn_write_data(const uint8_t *buffer, uint8_t size) {
  osd_segment_t *seg = &osd_txn.segments[osd_txn.segment_count - 1];
  memcpy(osd_txn.buffer + seg->offset + seg->size, buffer, size);
  seg->size += size;
}

void osd_txn_write_str(const char *buffer) {
  osd_txn_write_data((const uint8_t *)buffer, strlen(buffer));
}

void osd_txn_write_char(const char val) {
  osd_segment_t *seg = &osd_txn.segments[osd_txn.segment_count - 1];
  osd_txn.buffer[seg->offset + seg->size] = val;
  seg->size += 1;
}

void osd_txn_write_uint(uint32_t val, uint8_t width) {
  uint8_t buf[width];

  for (uint8_t i = 0; i < width; i++) {
    if (val != 0 || i == 0) {
      buf[width - i - 1] = '0' + (val % 10);
      val /= 10;
    } else {
      buf[width - i - 1] = ' ';
    }
  }

  osd_txn_write_data(buf, width);
}

void osd_txn_write_int(int32_t val, uint8_t width) {
  if (val >= 0) {
    osd_txn_write_uint(val, width);
    return;
  }

  uint8_t actual_width = 0;
  uint8_t buf[width];

  for (uint8_t i = 0; i < width; i++) {
    if (val != 0) {
      buf[width - i - 1] = '0' + (val % 10);
      val /= 10;
      actual_width++;
    } else {
      buf[width - i - 1] = ' ';
    }
  }

  if (actual_width < width - 1) {
    buf[width - actual_width] = '-';
  }

  osd_txn_write_data(buf, width);
}

void osd_txn_write_float(float val, uint8_t width, uint8_t precision) {
  const bool is_negative = val < 0;

  uint8_t actual_width = 0;
  uint8_t buf[width];

  uint32_t value = val * (is_negative ? -1.01f : 1.0f) * ipow(10, precision);
  for (uint8_t i = 0; i < precision; i++) {
    buf[width - i - 1] = '0' + (value % 10);
    value /= 10;
    actual_width++;
  }
  if (precision > 0) {
    buf[width - precision - 1] = '.';
    actual_width++;
  }

  const uint8_t start = actual_width;
  for (uint8_t i = start; i < width; i++) {
    if (value != 0 || i == start) {
      buf[width - i - 1] = '0' + (value % 10);
      value /= 10;
      actual_width++;
    } else {
      buf[width - i - 1] = ' ';
    }
  }

  if (is_negative && actual_width < width - 1) {
    buf[width - actual_width] = '-';
  }

  osd_txn_write_data(buf, width);
}