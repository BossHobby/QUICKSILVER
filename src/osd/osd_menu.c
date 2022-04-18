#include "osd_menu.h"

#include <string.h>

#include "drv_osd.h"
#include "osd_render.h"

#define SCREEN_COLS 32

typedef struct {
  bool had_change;

  uint8_t onscreen_elements;
  uint8_t rendered_elements;
  uint8_t active_elements;
  uint8_t grid_elements;
} osd_menu_state_t;

static osd_menu_state_t menu_state;

extern osd_system_t osd_system;

void osd_menu_start() {
  if (osd_state.screen_phase == 0) {
    if (osd_clear_async()) {
      osd_state.screen_phase++;
    }
  }
  if (osd_state.screen_phase == 1) {
    // re-render elements
    menu_state.rendered_elements = 0;
    menu_state.had_change = false;
    osd_state.selection_max = 1;
  }

  menu_state.onscreen_elements = 0;
  menu_state.active_elements = 0;
  menu_state.grid_elements = 0;
}

static void osd_menu_had_change() {
  menu_state.had_change = true;

  osd_state.selection_increase = 0;
  osd_state.selection_decrease = 0;
  osd_state.screen_phase = 1;
}

bool osd_menu_finish() {
  if (osd_state.screen_phase == 0 || menu_state.had_change) {
    return false;
  }
  if (osd_state.screen_phase < menu_state.onscreen_elements) {
    osd_state.screen_phase++;
    return false;
  }

  osd_state.cursor_min = 0;
  osd_state.cursor_max = menu_state.active_elements - 1;

  menu_state.grid_elements = 0;

  return osd_state.screen_phase >= menu_state.onscreen_elements;
}

static bool should_render_element() {
  menu_state.grid_elements = 0;
  menu_state.onscreen_elements++;

  if (menu_state.onscreen_elements != (menu_state.rendered_elements + 1)) {
    // our current element does not yet have its rendering turn
    return false;
  }

  if (osd_state.screen_phase != (menu_state.rendered_elements + 1)) {
    // not the correct screen phase
    return false;
  }

  return true;
}

static bool should_render_active_element() {
  menu_state.active_elements++;
  return should_render_element();
}

static bool should_render_grid_element() {
  menu_state.grid_elements++;

  if (menu_state.onscreen_elements != menu_state.rendered_elements) {
    // our current element does not yet have its rendering turn
    return false;
  }

  if (osd_state.screen_phase != menu_state.rendered_elements) {
    // not the correct screen phase
    return false;
  }

  if (osd_state.cursor == menu_state.active_elements) {
    osd_state.selection_max = menu_state.grid_elements;
  }

  return true;
}

static bool is_element_selected() {
  return osd_state.selection == menu_state.grid_elements && osd_state.cursor == menu_state.active_elements;
}

static bool has_adjust() {
  return osd_state.selection_increase || osd_state.selection_decrease;
}

int32_t osd_menu_adjust_int(int32_t val, const int32_t delta, const int32_t min, const int32_t max) {
  if (osd_state.selection_increase) {
    osd_state.selection_increase = 0;
    osd_menu_had_change();

    val += delta;
    if (val > max) {
      val = max;
    }
  }
  if (osd_state.selection_decrease) {
    osd_state.selection_decrease = 0;
    osd_menu_had_change();

    val -= delta;
    if (val < min) {
      val = min;
    }
  }

  return val;
}

float osd_menu_adjust_float(float val, const float delta, const float min, const float max) {
  const float rounded = (int)(val * 100.0f + (val <= 0 ? -0.5f : 0.5f));

  if (osd_state.selection_increase) {
    osd_state.selection_increase = 0;
    osd_menu_had_change();

    val = (rounded + (100.0f * delta)) / 100.0f;
    if (val > max) {
      val = max;
    }
  }
  if (osd_state.selection_decrease) {
    osd_state.selection_decrease = 0;
    osd_menu_had_change();

    val = (rounded - (100.0f * delta)) / 100.0f;
    if (val < min) {
      val = min;
    }
  }

  return val;
}

void osd_menu_adjust_str(char *str) {
  if (osd_state.selection_increase) {
    osd_state.selection_increase = 0;
    osd_menu_had_change();

    str[osd_state.selection - 1]++;
  }

  if (osd_state.selection_decrease) {
    osd_state.selection_decrease = 0;
    osd_menu_had_change();

    str[osd_state.selection - 1]--;
  }
}

vec3_t osd_menu_adjust_vec3(vec3_t val, const float delta, const float min, const float max) {
  if (osd_state.selection < 1 || osd_state.selection > 3) {
    return val;
  }

  val.axis[osd_state.selection - 1] = osd_menu_adjust_float(val.axis[osd_state.selection - 1], delta, min, max);

  return val;
}

void osd_menu_header(const char *text) {
  if (!should_render_element()) {
    return;
  }

  const uint8_t len = strlen(text);
  const uint8_t x = (SCREEN_COLS - len - 1) / 2;

  osd_transaction_t *txn = osd_txn_init();
  osd_txn_start(OSD_ATTR_INVERT, x, 1);
  osd_txn_write_data((const uint8_t *)text, len);
  osd_txn_submit(txn);

  menu_state.rendered_elements++;
}

void osd_menu_highlight(uint8_t x, uint8_t y, const char *text) {
  if (!should_render_element()) {
    return;
  }

  osd_transaction_t *txn = osd_txn_init();
  osd_txn_start(OSD_ATTR_INVERT, x, y);
  osd_txn_write_str(text);
  osd_txn_submit(txn);

  menu_state.rendered_elements++;
}

void osd_menu_label(uint8_t x, uint8_t y, const char *text) {
  if (!should_render_element()) {
    return;
  }

  osd_transaction_t *txn = osd_txn_init();
  osd_txn_start(OSD_ATTR_TEXT, x, y);
  osd_txn_write_str(text);
  osd_txn_submit(txn);

  menu_state.rendered_elements++;
}

bool osd_menu_button(uint8_t x, uint8_t y, const char *text) {
  if (!should_render_active_element()) {
    return osd_state.cursor == menu_state.active_elements && osd_state.selection == 1;
  }

  osd_transaction_t *txn = osd_txn_init();

  const bool is_selected = is_element_selected();
  if (osd_system == OSD_SYS_HD) {
    if (is_selected) {
      osd_txn_start(OSD_ATTR_INVERT, x - 1, y);
      osd_txn_write_char('>');
    } else {
      osd_txn_start(OSD_ATTR_TEXT, x - 1, y);
      osd_txn_write_char(' ');
    }
  } else {
    if (is_selected) {
      osd_txn_start(OSD_ATTR_INVERT, x, y);
    } else {
      osd_txn_start(OSD_ATTR_TEXT, x, y);
    }
  }

  osd_txn_write_str(text);
  osd_txn_submit(txn);

  menu_state.rendered_elements++;

  return is_selected && osd_state.selection == 1;
}

void osd_menu_select(uint8_t x, uint8_t y, const char *text) {
  if (!should_render_active_element()) {
    return;
  }

  osd_transaction_t *txn = osd_txn_init();

  const bool is_selected = is_element_selected();
  if (osd_system == OSD_SYS_HD) {
    if (is_selected) {
      osd_txn_start(OSD_ATTR_INVERT, x - 1, y);
      osd_txn_write_char('>');
    } else {
      osd_txn_start(OSD_ATTR_TEXT, x - 1, y);
      osd_txn_write_char(' ');
    }
  } else {
    if (is_selected) {
      osd_txn_start(OSD_ATTR_INVERT, x, y);
    } else {
      osd_txn_start(OSD_ATTR_TEXT, x, y);
    }
  }

  osd_txn_write_str(text);
  osd_txn_submit(txn);

  menu_state.rendered_elements++;
}

bool osd_menu_select_enum(uint8_t x, uint8_t y, const uint8_t val, const char **labels) {
  if (!should_render_grid_element()) {
    return is_element_selected() && has_adjust();
  }

  osd_transaction_t *txn = osd_txn_init();

  const bool is_selected = is_element_selected();
  if (osd_system == OSD_SYS_HD) {
    if (is_selected) {
      osd_txn_start(OSD_ATTR_INVERT, x - 1, y);
      osd_txn_write_char('>');
    } else {
      osd_txn_start(OSD_ATTR_TEXT, x - 1, y);
      osd_txn_write_char(' ');
    }
  } else {
    if (is_selected) {
      osd_txn_start(OSD_ATTR_INVERT, x, y);
    } else {
      osd_txn_start(OSD_ATTR_TEXT, x, y);
    }
  }

  osd_txn_write_str(labels[val]);
  osd_txn_submit(txn);

  return is_selected && has_adjust();
}

bool osd_menu_select_int(uint8_t x, uint8_t y, const int32_t val, uint8_t width) {
  if (!should_render_grid_element()) {
    return is_element_selected() && has_adjust();
  }

  osd_transaction_t *txn = osd_txn_init();

  const bool is_selected = is_element_selected();
  if (osd_system == OSD_SYS_HD) {
    if (is_selected) {
      osd_txn_start(OSD_ATTR_INVERT, x, y);
      osd_txn_write_char('>');
    } else {
      osd_txn_start(OSD_ATTR_TEXT, x, y);
      osd_txn_write_char(' ');
    }
    osd_txn_write_int(val, width - 1);
  } else {
    if (is_selected) {
      osd_txn_start(OSD_ATTR_INVERT, x, y);
    } else {
      osd_txn_start(OSD_ATTR_TEXT, x, y);
    }
    osd_txn_write_int(val, width);
  }

  osd_txn_submit(txn);

  return is_selected && has_adjust();
}

bool osd_menu_select_str(uint8_t x, uint8_t y, const char *str) {
  // limit selection to 20
  menu_state.grid_elements = 19;

  if (!should_render_grid_element()) {
    return osd_state.cursor == menu_state.active_elements && osd_state.selection >= 1 && has_adjust();
  }

  osd_transaction_t *txn = osd_txn_init();

  const bool is_elemet_selected = osd_state.cursor == menu_state.active_elements && osd_state.selection >= 1;

  if (is_elemet_selected) {
    const uint8_t len = strlen(str);
    const uint8_t offset = osd_state.selection - 1;

    if (offset > 0) {
      osd_txn_start(OSD_ATTR_TEXT, x, y);
      osd_txn_write_data((const uint8_t *)str, offset);
    }

    osd_txn_start(OSD_ATTR_INVERT, x + offset, y);
    osd_txn_write_char(str[offset]);

    if ((len - offset) > 0) {
      osd_txn_start(OSD_ATTR_TEXT, x + offset + 1, y);
      osd_txn_write_data((const uint8_t *)str + offset + 1, (len - offset - 1));
    }
  } else {
    osd_txn_start(OSD_ATTR_TEXT, x, y);
    osd_txn_write_str(str);
  }

  osd_txn_submit(txn);

  return osd_state.cursor == menu_state.active_elements && osd_state.selection >= 1 && has_adjust();
}

bool osd_menu_select_float(uint8_t x, uint8_t y, const float val, uint8_t width, uint8_t precision) {
  if (!should_render_grid_element()) {
    return is_element_selected() && has_adjust();
  }

  osd_transaction_t *txn = osd_txn_init();

  const bool is_selected = is_element_selected();
  if (osd_system == OSD_SYS_HD) {
    if (is_selected) {
      osd_txn_start(OSD_ATTR_INVERT, x, y);
      osd_txn_write_char('>');
    } else {
      osd_txn_start(OSD_ATTR_TEXT, x, y);
      osd_txn_write_char(' ');
    }
    osd_txn_write_float(val, width - 1, precision);
  } else {
    if (is_selected) {
      osd_txn_start(OSD_ATTR_INVERT, x, y);
    } else {
      osd_txn_start(OSD_ATTR_TEXT, x, y);
    }
    osd_txn_write_float(val, width, precision);
  }

  osd_txn_submit(txn);

  return is_selected && has_adjust();
}

bool osd_menu_select_vec3(uint8_t x, uint8_t y, const vec3_t val, uint8_t width, uint8_t precision) {
  if (osd_menu_select_float(x, y, val.roll, width, precision)) {
    return true;
  }
  if (osd_menu_select_float(x + 1 * width, y, val.pitch, width, precision)) {
    return true;
  }
  if (osd_menu_select_float(x + 2 * width, y, val.yaw, width, precision)) {
    return true;
  }
  return false;
}

void osd_menu_select_save_and_exit(uint8_t x, uint8_t y) {
  if (osd_menu_button(x, y, "SAVE AND EXIT")) {
    osd_save_exit();
  }
}

void osd_menu_select_screen(uint8_t x, uint8_t y, const char *text, osd_screens_t screen) {
  if (osd_menu_button(x, y, text)) {
    osd_push_screen(screen);
  }
}

void osd_menu_select_enum_adjust(uint8_t x, uint8_t y, const char *text, uint8_t select_x, uint8_t *val, const char **labels, const int32_t min, const int32_t max) {
  osd_menu_select(x, y, text);
  if (osd_menu_select_enum(select_x, y, *val, labels)) {
    *val = osd_menu_adjust_int(*val, 1, min, max);
  }
}