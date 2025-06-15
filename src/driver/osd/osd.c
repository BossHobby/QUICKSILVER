#include "driver/osd/osd.h"

#include <string.h>
#include <strings.h>

#include "core/profile.h"
#include "driver/osd/displayport.h"
#include "driver/osd/max7456.h"
#include "io/simulator.h"
#include "util/util.h"

#define MAX_DISPLAY_SIZE (HD_COLS * HD_ROWS)

static osd_segment_t osd_seg;
static osd_device_t osd_device = OSD_DEVICE_NONE;

static osd_char_t display[MAX_DISPLAY_SIZE];
static uint64_t display_dirty_rows[HD_ROWS];
static bool display_has_dirty = false;

static uint8_t cols = HD_COLS;
static uint8_t rows = HD_ROWS;

static bool blink_phase = false;

void osd_device_init() {
#ifdef USE_DIGITAL_VTX
  if (profile.serial.hdzero != SERIAL_PORT_INVALID) {
    target_set_feature(FEATURE_OSD);
    osd_device = OSD_DEVICE_DISPLAYPORT;
    cols = DISPLAYPORT_COLS;
    rows = DISPLAYPORT_ROWS;
    displayport_init();
  }
#endif
#ifdef USE_MAX7456
  else if (target_spi_device_valid(&target.osd) && max7456_init()) {
    target_set_feature(FEATURE_OSD);
    osd_device = OSD_DEVICE_MAX7456;
    cols = MAX7456_COLS;
    rows = MAX7456_ROWS;
  } else
#endif
#ifdef SIMULATOR
  {
    target_set_feature(FEATURE_OSD);
    osd_device = OSD_DEVICE_SIMULATOR;
    cols = DISPLAYPORT_COLS;
    rows = DISPLAYPORT_ROWS;
  }
#else
  {
    target_reset_feature(FEATURE_OSD);
  }
#endif
}

void osd_intro() {
  switch (osd_device) {
#ifdef USE_MAX7456
  case OSD_DEVICE_MAX7456:
    max7456_intro();
    break;
#endif
#ifdef USE_DIGITAL_VTX
  case OSD_DEVICE_DISPLAYPORT:
    displayport_intro();
    break;
#endif
#ifdef SIMULATOR
  case OSD_DEVICE_SIMULATOR:
    simulator_osd_intro();
    break;
#endif
  default:
    break;
  }
}

void osd_clear() {
  while (!osd_clear_async())
    ;
}

static bool osd_device_clear_async() {
  switch (osd_device) {
#ifdef USE_MAX7456
  case OSD_DEVICE_MAX7456:
    if (!max7456_clear_async()) {
      return false;
    }
    break;
#endif
#ifdef USE_DIGITAL_VTX
  case OSD_DEVICE_DISPLAYPORT:
    if (!displayport_clear_async()) {
      return false;
    }
    break;
#endif
#ifdef SIMULATOR
  case OSD_DEVICE_SIMULATOR:
    if (!simulator_osd_clear_async()) {
      return false;
    }
    break;
#endif
  default:
    break;
  }

  return true;
}

static bool osd_mark_dirty() {
  static uint8_t state = 0;

  switch (state) {
  case 0:
    if (!osd_device_clear_async()) {
      return false;
    }
    state++;
    // FALLTHROUGH

  default: {
    // Process rows to mark non-space characters as dirty
    osd_char_t *ptr = display;
    for (uint8_t row = 0; row < rows; row++) {
      uint64_t dirty_mask = 0;
      for (uint8_t col = 0; col < cols; col++, ptr++) {
        if (ptr->val != ' ') {
          dirty_mask |= (1ULL << col);
        }
      }
      display_dirty_rows[row] = dirty_mask;
      if (dirty_mask) {
        display_has_dirty = true;
      }
    }
    state = 0;
    return true;
  }
  }
}

static bool osd_mark_blink_dirty() {
  osd_char_t *ptr = display;
  for (uint8_t row = 0; row < rows; row++) {
    uint64_t blink_mask = 0;
    for (uint8_t col = 0; col < cols; col++, ptr++) {
      if ((ptr->attr & OSD_ATTR_BLINK) && ptr->val != ' ') {
        blink_mask |= (1ULL << col);
      }
    }
    if (blink_mask) {
      display_dirty_rows[row] |= blink_mask;
      display_has_dirty = true;
    }
  }
  return true;
}

bool osd_clear_async() {
  static uint8_t state = 0;

  switch (state) {
  case 0:
    for (uint32_t i = 0; i < (MAX_DISPLAY_SIZE / 2); i++) {
      ((uint32_t *)display)[i] = ((0x20 << 16) | 0x20);
    }
    memset(display_dirty_rows, 0, sizeof(display_dirty_rows));
    display_has_dirty = false;
    state++;
    return false;

  default:
    if (!osd_device_clear_async()) {
      return false;
    }
    state = 0;
    return true;
  }
}

static void osd_display_set(uint8_t x, uint8_t y, uint8_t attr, uint8_t val) {
  if (x >= cols || y >= rows) {
    return;
  }

  const uint16_t offset = y * cols + x;
  osd_char_t *const c = &display[offset];
  if (c->val == val && c->attr == attr) {
    return;
  }

  c->attr = attr;
  c->val = val;

  display_dirty_rows[y] |= (1ULL << x);
  display_has_dirty = true;
}

void osd_display_refresh() {
  for (uint8_t y = 0; y < rows; y++) {
    for (uint8_t x = 0; x < cols; x++) {
      osd_display_set(x, y, 0, ' ');
    }
  }
}

osd_system_t osd_check_system() {
  switch (osd_device) {
#ifdef USE_MAX7456
  case OSD_DEVICE_MAX7456:
    cols = MAX7456_COLS;
    rows = MAX7456_ROWS;
    return max7456_check_system();
#endif
#ifdef USE_DIGITAL_VTX
  case OSD_DEVICE_DISPLAYPORT:
    cols = DISPLAYPORT_COLS;
    rows = DISPLAYPORT_ROWS;
    return displayport_check_system();
#endif
#ifdef SIMULATOR
  case OSD_DEVICE_SIMULATOR:
    cols = DISPLAYPORT_COLS;
    rows = DISPLAYPORT_ROWS;
    return simulator_osd_check_system();
#endif
  default:
    return OSD_SYS_NONE;
  }
}

void osd_start(uint8_t attr, uint8_t x, uint8_t y) {
  osd_seg.attr = attr;
  osd_seg.x = x;
  osd_seg.y = y;
  osd_seg.offset = 0;
}

void osd_write_char(const char val) {
  osd_display_set(osd_seg.x + osd_seg.offset, osd_seg.y, osd_seg.attr, val);
  osd_seg.offset += 1;
}

void osd_write_data(const uint8_t *buffer, uint8_t size) {
  for (uint8_t i = 0; i < size; i++)
    osd_write_char(buffer[i]);
}

static uint32_t osd_can_fit() {
  switch (osd_device) {
#ifdef USE_MAX7456
  case OSD_DEVICE_MAX7456:
    return max7456_can_fit();
#endif
#ifdef USE_DIGITAL_VTX
  case OSD_DEVICE_DISPLAYPORT:
    return displayport_can_fit();
#endif
#ifdef SIMULATOR
  case OSD_DEVICE_SIMULATOR:
    return simulator_osd_can_fit();
#endif
  default:
    return false;
  }
}

static bool osd_push_string(uint8_t attr, uint8_t x, uint8_t y, const uint8_t *data, uint8_t size) {
  if (size == 0) {
    return true;
  }

  switch (osd_device) {
#ifdef USE_MAX7456
  case OSD_DEVICE_MAX7456:
    return max7456_push_string(attr, x, y, data, size);
#endif
#ifdef USE_DIGITAL_VTX
  case OSD_DEVICE_DISPLAYPORT:
    return displayport_push_string(attr, x, y, data, size);
#endif
#ifdef SIMULATOR
  case OSD_DEVICE_SIMULATOR:
    return simulator_osd_push_string(attr, x, y, data, size);
#endif
  default:
    return false;
  }
}

static bool osd_flush() {
  switch (osd_device) {
#ifdef USE_MAX7456
  case OSD_DEVICE_MAX7456:
    return max7456_flush();
#endif
#ifdef USE_DIGITAL_VTX
  case OSD_DEVICE_DISPLAYPORT:
    return displayport_flush();
#endif
#ifdef SIMULATOR
  case OSD_DEVICE_SIMULATOR:
    return simulator_osd_flush();
#endif
  default:
    return false;
  }
}

static bool osd_update_display() {
  static uint8_t row = 0;
  static uint8_t string[DISPLAYPORT_COLS];

  // Early exit if no dirty rows
  if (!display_has_dirty) {
    return true;
  }

  // Process rows until we run out of dirty rows
  while (true) {
    // Find next dirty row
    while (row < rows && display_dirty_rows[row] == 0) {
      row++;
    }

    if (row >= rows) {
      if (osd_flush()) {
        row = 0;
        display_has_dirty = false;
        return true;
      }
      return false;
    }

    const uint16_t row_offset = row * cols;
    osd_char_t *const row_start = &display[row_offset];
    uint64_t dirty_mask = display_dirty_rows[row];

    while (dirty_mask) {
      // Find next dirty bit using builtin
      uint8_t col = __builtin_ctzll(dirty_mask);

      // Check if we can send more data
      uint32_t max_chars = osd_can_fit();
      if (max_chars == 0) {
        // No space available, update the dirty mask and stop processing
        display_dirty_rows[row] = dirty_mask;
        return false;
      }

      // Found dirty character, build string with same attribute
      const uint8_t start = col;
      const uint8_t attr = row_start[col].attr;
      uint8_t size = 0;

      const bool do_blink = osd_device == OSD_DEVICE_DISPLAYPORT && (attr & OSD_ATTR_BLINK) && !blink_phase;

      // Build string of consecutive chars with same attribute
      while (col < cols && size < max_chars && (dirty_mask & (1ULL << col)) && row_start[col].attr == attr) {
        string[size++] = do_blink ? ' ' : row_start[col].val;
        dirty_mask &= ~(1ULL << col); // Clear this bit
        col++;
      }

      if (!osd_push_string(attr, start, row, string, size)) {
        // Push failed - restore dirty mask to current state and return
        display_dirty_rows[row] = dirty_mask;
        return false;
      }
    }

    // All characters in this row processed successfully
    display_dirty_rows[row] = 0;
    row++;
  }

  return false;
}

bool osd_is_ready() {
  switch (osd_device) {
#ifdef USE_MAX7456
  case OSD_DEVICE_MAX7456:
    return max7456_is_ready();
#endif
#ifdef USE_DIGITAL_VTX
  case OSD_DEVICE_DISPLAYPORT:
    return displayport_is_ready();
#endif
#ifdef SIMULATOR
  case OSD_DEVICE_SIMULATOR:
    return simulator_osd_is_ready();
#endif
  default:
    return false;
  }
}

bool osd_update() {
  if (display_has_dirty) {
    if (osd_update_display()) {
      display_has_dirty = false;
    }
    return false;
  }

  // Periodic refresh for displayport devices
  bool can_render = true;

  if (osd_device == OSD_DEVICE_DISPLAYPORT) {
    const uint32_t now = time_millis();

    static uint32_t last_blink = 0;
    if (now - last_blink > 250) {
      osd_mark_blink_dirty();
      blink_phase = !blink_phase;
      last_blink = now;
      can_render = false;
    }

    static uint32_t last_redraw = 0;
    if (now - last_redraw > 5000) {
      if (osd_mark_dirty()) {
        last_redraw = now;
        can_render = false;
      }
    }
  }

  return can_render;
}

void osd_write_str(const char *buffer) {
  osd_write_data((const uint8_t *)buffer, strlen(buffer));
}

void osd_write_uint(uint32_t val, uint8_t width) {
  uint8_t buf[width];

  for (uint8_t i = 0; i < width; i++) {
    if (val != 0 || i == 0) {
      buf[width - i - 1] = '0' + (val % 10);
      val /= 10;
    } else {
      buf[width - i - 1] = ' ';
    }
  }

  osd_write_data(buf, width);
}

void osd_write_int(int32_t val, uint8_t width) {
  if (val >= 0) {
    osd_write_uint(val, width);
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

  osd_write_data(buf, width);
}

void osd_write_float(float val, uint8_t width, uint8_t precision) {
  const bool is_negative = val < 0;

  // reserve one for the dot
  uint8_t decimals = 1;

  uint32_t value = val;
  while (value) {
    value /= 10;
    decimals++;
  }
  if (is_negative) {
    decimals++;
  }
  if (decimals > width) {
    precision = 0;
  } else {
    precision = constrain(width - decimals, 0, precision);
  }

  uint8_t buf[width];
  uint8_t actual_width = 0;
  value = val * (is_negative ? -1.0f : 1.0f) * ipow(10, precision);
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

  osd_write_data(buf, width);
}
