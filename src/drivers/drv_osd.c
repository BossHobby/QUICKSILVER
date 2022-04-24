#include "drv_osd.h"

#include <string.h>

#include "drv_serial_hdzero.h"
#include "drv_spi_max7456.h"
#include "profile.h"
#include "util/util.h"

static osd_transaction_t osd_txn;
static osd_device_t osd_device = OSD_DEVICE_NONE;

void osd_device_init() {
  if (profile.serial.hdzero != USART_PORT_INVALID) {
    osd_device = OSD_DEVICE_HDZERO;
    hdzero_init();
  } else {
    osd_device = OSD_DEVICE_MAX7456;
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
  switch (osd_device) {
  case OSD_DEVICE_MAX7456:
    return max7456_clear_async();

  case OSD_DEVICE_HDZERO:
    return hdzero_clear_async();

  default:
    return 0;
  }
}

osd_system_t osd_check_system() {
  switch (osd_device) {
  case OSD_DEVICE_MAX7456:
    return max7456_check_system();

  case OSD_DEVICE_HDZERO:
    return hdzero_check_system();

  default:
    return 0;
  }
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

  switch (osd_device) {
  case OSD_DEVICE_MAX7456:
    max7456_txn_start(&osd_txn, attr, x, y);
    break;

  case OSD_DEVICE_HDZERO:
    hdzero_txn_start(&osd_txn, attr, x, y);
    break;

  default:
    break;
  }
}

void osd_txn_write_data(const uint8_t *buffer, uint8_t size) {
  osd_segment_t *seg = &osd_txn.segments[osd_txn.segment_count - 1];
  seg->size += size;

  switch (osd_device) {
  case OSD_DEVICE_MAX7456:
    max7456_txn_write_data(&osd_txn, buffer, size);
    break;

  case OSD_DEVICE_HDZERO:
    hdzero_txn_write_data(&osd_txn, buffer, size);
    break;

  default:
    break;
  }

  seg->offset += size;
}

void osd_txn_write_str(const char *buffer) {
  osd_txn_write_data((const uint8_t *)buffer, strlen(buffer));
}

void osd_txn_write_char(const char val) {
  osd_segment_t *seg = &osd_txn.segments[osd_txn.segment_count - 1];
  seg->size += 1;

  switch (osd_device) {
  case OSD_DEVICE_MAX7456:
    max7456_txn_write_char(&osd_txn, val);
    break;

  case OSD_DEVICE_HDZERO:
    hdzero_txn_write_char(&osd_txn, val);
    break;

  default:
    break;
  }

  seg->offset += 1;
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

void osd_txn_submit(osd_transaction_t *txn) {
  switch (osd_device) {
  case OSD_DEVICE_MAX7456:
    max7456_txn_submit(txn);
    break;

  case OSD_DEVICE_HDZERO:
    hdzero_txn_submit(txn);
    break;

  default:
    break;
  }
}