#include "drv_osd.h"

#include <string.h>

#include "drv_serial_hdzero.h"
#include "drv_spi_max7456.h"
#include "profile.h"
#include "util/util.h"

#define MAX_ROWS HDZERO_ROWS
#define MAX_COLS HDZERO_COLS

#define MAX_DISPLAY_SIZE (HDZERO_COLS * HDZERO_ROWS)

static osd_transaction_t osd_txn;
static osd_device_t osd_device = OSD_DEVICE_NONE;

static osd_char_t display[MAX_DISPLAY_SIZE];
static bool display_row_dirty[HDZERO_ROWS];
static bool display_dirty = false;

static uint8_t cols = HDZERO_COLS;
static uint8_t rows = HDZERO_ROWS;

void osd_device_init() {
  if (profile.serial.hdzero != USART_PORT_INVALID) {
    osd_device = OSD_DEVICE_HDZERO;
    cols = HDZERO_COLS;
    rows = HDZERO_ROWS;
    hdzero_init();
  } else {
    osd_device = OSD_DEVICE_MAX7456;
    cols = MAX7456_COLS;
    rows = MAX7456_ROWS;
    max7456_init();
  }
}

void osd_intro() {
  switch (osd_device) {
  case OSD_DEVICE_MAX7456:
    max7456_intro();
    break;

  case OSD_DEVICE_HDZERO:
    hdzero_intro();
    break;

  default:
    break;
  }
}

void osd_clear() {
  while (!osd_clear_async())
    ;
}

uint8_t osd_clear_async() {
  uint8_t is_done = 0;

  switch (osd_device) {
  case OSD_DEVICE_MAX7456:
    is_done = max7456_clear_async();
    break;

  case OSD_DEVICE_HDZERO:
    is_done = hdzero_clear_async();
    break;

  default:
    is_done = 1;
    break;
  }

  if (is_done) {
    memset(display, 0, MAX_DISPLAY_SIZE * sizeof(osd_char_t));
    memset(display_row_dirty, 0, MAX_ROWS * sizeof(bool));
    display_dirty = false;
  }

  return is_done;
}

osd_system_t osd_check_system() {
  switch (osd_device) {
  case OSD_DEVICE_MAX7456:
    cols = MAX7456_COLS;
    rows = MAX7456_ROWS;
    return max7456_check_system();

  case OSD_DEVICE_HDZERO:
    cols = HDZERO_COLS;
    rows = HDZERO_ROWS;
    return hdzero_check_system();

  default:
    return 0;
  }
}

osd_transaction_t *osd_txn_init() {
  osd_txn.segment_count = 0;
  return &osd_txn;
}

void osd_txn_start(uint8_t attr, uint8_t x, uint8_t y) {
  osd_segment_t *seg = &osd_txn.segments[osd_txn.segment_count];
  seg->attr = attr;
  seg->x = x;
  seg->y = y;
  seg->offset = 0;
  seg->size = 0;

  osd_txn.segment_count++;
}

void osd_txn_write_data(const uint8_t *buffer, uint8_t size) {
  osd_segment_t *seg = &osd_txn.segments[osd_txn.segment_count - 1];
  seg->size += size;

  const uint16_t offset = seg->y * cols + seg->x + seg->offset;
  for (uint8_t i = 0; i < size; i++) {
    if (display[offset + i].val == buffer[i] && display[offset + i].attr == seg->attr) {
      continue;
    }

    display[offset + i].dirty = 1;
    display[offset + i].attr = seg->attr;
    display[offset + i].val = buffer[i];

    display_row_dirty[seg->y] = true;
    display_dirty = true;
  }

  seg->offset += size;
}

void osd_txn_write_char(const char val) {
  osd_segment_t *seg = &osd_txn.segments[osd_txn.segment_count - 1];
  seg->size += 1;

  const uint16_t offset = seg->y * cols + seg->x + seg->offset;
  if (display[offset].val == val && display[offset].attr == seg->attr) {
    return;
  }

  display[offset].dirty = 1;
  display[offset].attr = seg->attr;
  display[offset].val = val;

  display_row_dirty[seg->y] = true;
  display_dirty = true;

  seg->offset += 1;
}

void osd_txn_submit(osd_transaction_t *txn) {
}

static bool osd_can_fit(uint8_t size) {
  switch (osd_device) {
  case OSD_DEVICE_MAX7456:
    return max7456_can_fit(size);

  case OSD_DEVICE_HDZERO:
    return hdzero_can_fit(size);

  default:
    return false;
  }
}

static bool osd_push_string(uint8_t attr, uint8_t x, uint8_t y, const uint8_t *data, uint8_t size) {
  switch (osd_device) {
  case OSD_DEVICE_MAX7456:
    return max7456_push_string(attr, x, y, data, size);

  case OSD_DEVICE_HDZERO:
    return hdzero_push_string(attr, x, y, data, size);

  default:
    return false;
  }
}

static bool osd_flush() {
  switch (osd_device) {
  case OSD_DEVICE_MAX7456:
    return max7456_flush();

  case OSD_DEVICE_HDZERO:
    return hdzero_flush();

  default:
    return false;
  }
}

static bool osd_update_display() {
  static uint8_t row = 0;

  while (row < rows) {
    if (!display_row_dirty[row]) {
      row++;
      continue;
    }

    uint8_t string[cols];
    uint8_t attr = 0;
    uint8_t start = cols;
    uint8_t size = 0;

    volatile bool row_done = true;
    for (uint8_t col = 0; col < cols; col++) {
      osd_char_t *entry = &display[row * cols + col];

      if (size && (!entry->dirty || entry->attr != attr || col == (cols - 1) || row_done == false)) {
        osd_push_string(attr, start, row, string, size);
        size = 0;
        start = cols;

        if (!row_done) {
          break;
        }
      }

      if (!entry->dirty) {
        continue;
      }

      if (col < start) {
        start = col;
        attr = entry->attr;
      }

      if (!osd_can_fit(size + 1)) {
        row_done = false;
        continue;
      }

      string[size] = entry->val;
      entry->dirty = 0;
      size++;
    }

    if (row_done) {
      display_row_dirty[row] = false;
      row++;
    }
    return false;
  }

  if (row == rows) {
    if (osd_flush()) {
      row++;
    }
    return false;
  }

  row = 0;
  return true;
}

bool osd_is_ready() {
  switch (osd_device) {
  case OSD_DEVICE_MAX7456:
    return max7456_is_ready();

  case OSD_DEVICE_HDZERO:
    return hdzero_is_ready();

  default:
    return false;
  }
}

bool osd_update() {
  if (display_dirty) {
    if (osd_update_display()) {
      display_dirty = false;
    }
    return false;
  }
  return true;
}

void osd_txn_write_str(const char *buffer) {
  osd_txn_write_data((const uint8_t *)buffer, strlen(buffer));
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

  val = -val;

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
    buf[actual_width + 1] = '-';
  }

  osd_txn_write_data(buf, width);
}

void osd_txn_write_float(float val, uint8_t width, uint8_t precision) {
  const bool is_negative = val < 0;

  uint8_t actual_width = 0;
  uint8_t buf[width];

  uint32_t value = val * (is_negative ? -1.0f : 1.0f) * ipow(10, precision);
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
