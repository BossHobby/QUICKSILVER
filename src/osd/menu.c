#include "osd/menu.h"

#include <string.h>

#include "driver/osd.h"
#include "osd/render.h"
#include "util/util.h"

#define SCREEN_COLS 32

typedef struct {
  uint8_t onscreen_elements;
  uint8_t active_elements;
  uint8_t grid_elements;

  uint8_t scroll_max;
  uint8_t scroll_start;
  uint8_t scroll_elements;
  uint8_t scroll_offset;
} osd_menu_state_t;

static osd_menu_state_t menu_state;

extern osd_system_t osd_system;

void osd_menu_start() {
  menu_state.onscreen_elements = 0;
  menu_state.active_elements = 0;
  menu_state.grid_elements = 0;
  osd_state.selection_max = 1;

  switch (osd_state.screen_phase) {
  case OSD_PHASE_CLEAR:
    if (!osd_clear_async()) {
      break;
    }
    osd_state.screen_phase = OSD_PHASE_RENDER;
    break;

  case OSD_PHASE_REFRESH:
    osd_display_refresh();
    osd_state.screen_phase = OSD_PHASE_RENDER;
    break;

  default:
    break;
  }
}

static void osd_menu_had_change() {
  osd_state.selection_increase = 0;
  osd_state.selection_decrease = 0;

  osd_state.screen_phase = OSD_PHASE_REFRESH;
}

bool osd_menu_finish() {
  if (osd_state.screen_phase == OSD_PHASE_CLEAR) {
    return false;
  }
  if (osd_state.screen_phase == OSD_PHASE_IDLE) {
    return true;
  }

  osd_state.screen_phase++;

  osd_state.cursor_min = 0;
  osd_state.cursor_max = menu_state.active_elements - 1;

  menu_state.grid_elements = 0;

  return false;
}

static bool should_render_element() {
  menu_state.grid_elements = 0;
  menu_state.onscreen_elements++;
  return osd_state.screen_phase == OSD_PHASE_RENDER;
}

static bool should_render_active_element(int8_t y) {
  menu_state.grid_elements = 0;
  menu_state.onscreen_elements++;
  menu_state.active_elements++;

  if (y == OSD_AUTO) {
    menu_state.scroll_elements++;

    const uint8_t scroll_cursor = osd_state.cursor - (menu_state.active_elements - menu_state.scroll_elements);
    const uint8_t scroll_page = ((scroll_cursor - 1) / menu_state.scroll_max) * menu_state.scroll_max;
    if (menu_state.scroll_elements <= scroll_page || menu_state.scroll_elements > (scroll_page + menu_state.scroll_max)) {
      return false;
    }

    menu_state.scroll_offset++;
  }

  return osd_state.screen_phase == OSD_PHASE_RENDER;
}

static bool should_render_grid_element(int8_t y) {
  menu_state.grid_elements++;

  if (y == OSD_AUTO) {
    const uint8_t scroll_cursor = osd_state.cursor - (menu_state.active_elements - menu_state.scroll_elements);
    const uint8_t scroll_page = ((scroll_cursor - 1) / menu_state.scroll_max) * menu_state.scroll_max;
    if (menu_state.scroll_elements <= scroll_page || menu_state.scroll_elements > (scroll_page + menu_state.scroll_max)) {
      return false;
    }
  }

  if (osd_state.cursor == menu_state.active_elements) {
    osd_state.selection_max = menu_state.grid_elements;
  }

  return osd_state.screen_phase == OSD_PHASE_RENDER;
}

static bool is_element_selected() {
  return osd_state.selection == menu_state.grid_elements && osd_state.cursor == menu_state.active_elements;
}

static bool has_adjust() {
  return osd_state.selection_increase || osd_state.selection_decrease;
}

static uint8_t get_y_value(int8_t y) {
  switch (y) {
  case OSD_AUTO:
    return menu_state.scroll_offset;

  case OSD_END:
    return 14;

  default:
    return y;
  }
}

void osd_menu_scroll_start(uint8_t x, uint8_t y, uint8_t max) {
  menu_state.scroll_max = max;
  menu_state.scroll_start = y;
  menu_state.scroll_elements = 0;
  menu_state.scroll_offset = y;

  const uint8_t scroll_cursor = osd_state.cursor - (menu_state.active_elements - menu_state.scroll_elements);
  const uint8_t scroll_page = ((scroll_cursor - 1) / menu_state.scroll_max);
  if (scroll_page > 0 && should_render_element()) {
    osd_start(OSD_ATTR_TEXT, x - 1, y);
    osd_write_char(0x75); // UP ARROW
  }
}

void osd_menu_scroll_finish(uint8_t x) {
  const uint8_t scroll_cursor = osd_state.cursor - (menu_state.active_elements - menu_state.scroll_elements);
  const uint8_t scroll_page = ((scroll_cursor - 1) / menu_state.scroll_max);
  const uint8_t scroll_page_max = ((menu_state.scroll_elements - 1) / menu_state.scroll_max);
  if (scroll_page != scroll_page_max && should_render_element()) {
    osd_start(OSD_ATTR_TEXT, x - 1, menu_state.scroll_start + menu_state.scroll_max + 1);
    osd_write_char(0x76); // DOWN ARROW
  }

  menu_state.scroll_max = 0;
  menu_state.scroll_start = 0;
  menu_state.scroll_elements = 0;
  menu_state.scroll_offset = 0;
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

  osd_start(OSD_ATTR_INVERT, x, 1);
  osd_write_data((const uint8_t *)text, len);
}

void osd_menu_highlight(int8_t x, int8_t y, const char *text) {
  if (!should_render_element()) {
    return;
  }

  osd_start(OSD_ATTR_INVERT, x, y);
  osd_write_str(text);
}

bool osd_menu_label_start(int8_t x, int8_t y) {
  if (!should_render_element()) {
    return false;
  }

  osd_start(OSD_ATTR_TEXT, x, y);
  return true;
}

void osd_menu_label(int8_t x, int8_t y, const char *text) {
  if (!osd_menu_label_start(x, y)) {
    return;
  }

  osd_write_str(text);
}

bool osd_menu_button(int8_t x, int8_t y, const char *text) {
  if (!should_render_active_element(y)) {
    return osd_state.cursor == menu_state.active_elements && osd_state.selection == 1;
  }

  y = get_y_value(y);

  const bool is_selected = is_element_selected();
  if (osd_system == OSD_SYS_HD) {
    if (is_selected) {
      osd_start(OSD_ATTR_INVERT, x - 1, y);
      osd_write_char('>');
    } else {
      osd_start(OSD_ATTR_TEXT, x - 1, y);
      osd_write_char(' ');
    }
  } else {
    if (is_selected) {
      osd_start(OSD_ATTR_INVERT, x, y);
    } else {
      osd_start(OSD_ATTR_TEXT, x, y);
    }
  }

  osd_write_str(text);

  return osd_state.cursor == menu_state.active_elements && osd_state.selection == 1;
}

void osd_menu_select(int8_t x, int8_t y, const char *text) {
  if (!should_render_active_element(y)) {
    return;
  }

  y = get_y_value(y);

  const bool is_selected = is_element_selected();
  if (osd_system == OSD_SYS_HD) {
    if (is_selected) {
      osd_start(OSD_ATTR_INVERT, x - 1, y);
      osd_write_char('>');
    } else {
      osd_start(OSD_ATTR_TEXT, x - 1, y);
      osd_write_char(' ');
    }
  } else {
    if (is_selected) {
      osd_start(OSD_ATTR_INVERT, x, y);
    } else {
      osd_start(OSD_ATTR_TEXT, x, y);
    }
  }

  osd_write_str(text);
}

bool osd_menu_select_enum(int8_t x, int8_t y, const uint8_t val, const char **labels) {
  if (!should_render_grid_element(y)) {
    return is_element_selected() && has_adjust();
  }

  y = get_y_value(y);

  const bool is_selected = is_element_selected();
  if (osd_system == OSD_SYS_HD) {
    if (is_selected) {
      osd_start(OSD_ATTR_INVERT, x - 1, y);
      osd_write_char('>');
    } else {
      osd_start(OSD_ATTR_TEXT, x - 1, y);
      osd_write_char(' ');
    }
  } else {
    if (is_selected) {
      osd_start(OSD_ATTR_INVERT, x, y);
    } else {
      osd_start(OSD_ATTR_TEXT, x, y);
    }
  }

  osd_write_str(labels[val]);

  return is_selected && has_adjust();
}

bool osd_menu_select_int(int8_t x, int8_t y, const int32_t val, uint8_t width) {
  if (!should_render_grid_element(y)) {
    return is_element_selected() && has_adjust();
  }

  y = get_y_value(y);

  const bool is_selected = is_element_selected();
  if (osd_system == OSD_SYS_HD) {
    if (is_selected) {
      osd_start(OSD_ATTR_INVERT, x, y);
      osd_write_char('>');
    } else {
      osd_start(OSD_ATTR_TEXT, x, y);
      osd_write_char(' ');
    }
    osd_write_int(val, width - 1);
  } else {
    if (is_selected) {
      osd_start(OSD_ATTR_INVERT, x, y);
    } else {
      osd_start(OSD_ATTR_TEXT, x, y);
    }
    osd_write_int(val, width);
  }

  return is_selected && has_adjust();
}

bool osd_menu_select_str(int8_t x, int8_t y, const char *str) {
  // limit selection to 20
  menu_state.grid_elements = 19;

  if (!should_render_grid_element(y)) {
    return osd_state.cursor == menu_state.active_elements && osd_state.selection >= 1 && has_adjust();
  }

  y = get_y_value(y);

  const bool is_elemet_selected = osd_state.cursor == menu_state.active_elements && osd_state.selection >= 1;

  if (is_elemet_selected) {
    const uint8_t len = strlen(str);
    const uint8_t offset = osd_state.selection - 1;

    if (offset > 0) {
      osd_start(OSD_ATTR_TEXT, x, y);
      osd_write_data((const uint8_t *)str, offset);
    }

    osd_start(OSD_ATTR_INVERT, x + offset, y);
    osd_write_char(str[offset]);

    if ((len - offset) > 0) {
      osd_start(OSD_ATTR_TEXT, x + offset + 1, y);
      osd_write_data((const uint8_t *)str + offset + 1, (len - offset - 1));
    }
  } else {
    osd_start(OSD_ATTR_TEXT, x, y);
    osd_write_str(str);
  }

  return osd_state.cursor == menu_state.active_elements && osd_state.selection >= 1 && has_adjust();
}

bool osd_menu_select_float(int8_t x, int8_t y, const float val, uint8_t width, uint8_t precision) {
  if (!should_render_grid_element(y)) {
    return is_element_selected() && has_adjust();
  }

  y = get_y_value(y);

  const bool is_selected = is_element_selected();
  if (osd_system == OSD_SYS_HD) {
    if (is_selected) {
      osd_start(OSD_ATTR_INVERT, x, y);
      osd_write_char('>');
    } else {
      osd_start(OSD_ATTR_TEXT, x, y);
      osd_write_char(' ');
    }
    osd_write_float(val, width - 1, precision);
  } else {
    if (is_selected) {
      osd_start(OSD_ATTR_INVERT, x, y);
    } else {
      osd_start(OSD_ATTR_TEXT, x, y);
    }
    osd_write_float(val, width, precision);
  }

  return is_selected && has_adjust();
}

bool osd_menu_select_vec3(int8_t x, int8_t y, const vec3_t val, uint8_t width, uint8_t precision) {
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

void osd_menu_select_save_and_exit(int8_t x) {
  if (osd_menu_button(x, OSD_END, "SAVE AND EXIT")) {
    osd_save_exit();
  }
}

void osd_menu_select_exit(int8_t x) {
  if (osd_menu_button(x, OSD_END, "EXIT")) {
    osd_exit();
  }
}

void osd_menu_select_screen(int8_t x, int8_t y, const char *text, osd_screens_t screen) {
  if (osd_menu_button(x, y, text)) {
    osd_push_screen(screen);
  }
}

void osd_menu_select_enum_adjust(int8_t x, int8_t y, const char *text, uint8_t select_x, uint8_t *val, const char **labels, const int32_t min, const int32_t max) {
  osd_menu_select(x, y, text);
  if (osd_menu_select_enum(select_x, y, *val, labels)) {
    *val = osd_menu_adjust_int(*val, 1, min, max);
  }
}