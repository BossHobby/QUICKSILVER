#include "osd_render.h"

#include "control.h"
#include "debug.h"
#include "drv_osd.h"
#include "drv_time.h"
#include "filter.h"
#include "flash.h"
#include "float.h"
#include "osd_adjust.h"
#include "osd_menu.h"
#include "osd_menu_maps.h"
#include "profile.h"
#include "project.h"
#include "rx.h"
#include "stdio.h"
#include "string.h"
#include "util.h"
#include "vtx.h"

#ifdef ENABLE_OSD

typedef enum {
  CLEAR,
  DISARM,
  ARM,
  BOOST_1,
  BOOST_2,
  FAILSAFE,
  THRTL_SFTY,
  ARM_SFTY,
  LOW_BAT,
  MOTOR_TEST,
  TURTL,
  LOOP
} osd_status_labels_t;

typedef struct {
  uint8_t loop_warning;
  uint8_t aux_boost;
  uint8_t aux_motor_test;
  control_flags_t flags;
} osd_sys_status_t;

#define ICON_RSSI 0x1
#define ICON_CELSIUS 0xe
#define ICON_THROTTLE 0x4
#define ICON_AMP 0x9a

#define MAIN_MENU 1
#define SUB_MENU 0

#define HOLD 0
#define TEMP 1

extern profile_t profile;
extern vtx_settings_t vtx_settings;
extern vtx_settings_t vtx_settings_copy;

osd_system_t osd_system = OSD_SYS_NONE;

osd_state_t osd_state = {
    .element = OSD_CALLSIGN,

    .screen = OSD_SCREEN_REGULAR,
    .screen_history_size = 0,
    .screen_phase = 0,

    .cursor = 0,
    .cursor_history_size = 0,

    .selection = 0,
    .selection_increase = 0,
    .selection_decrease = 0,

    .reboot_fc_requested = 0,
};

static uint8_t osd_attr(osd_element_t *el) {
  return el->attribute ? OSD_ATTR_INVERT : OSD_ATTR_TEXT;
}

uint8_t osd_decode(uint32_t element, uint8_t status) {
  switch (status) {
  case ACTIVE:
    return (element & 0x01);
  case ATTRIBUTE:
    if (((element >> 1) & 0x01) == 0x01)
      return OSD_ATTR_TEXT;
    else
      return OSD_ATTR_INVERT;
  case POSITIONX:
    return ((element >> 2) & 0xFF);
  case POSITIONY:
    return ((element >> 10) & 0x0F);
  }
  return 0;
}

const char *get_position_string(int input) {
  static char *respond[] = {" 0", " 1", " 2", " 3", " 4", " 5", " 6", " 7", " 8", " 9", "10", "11", "12", "13", "14", "15", "16", "17", "18", "19", "20", "21", "22", "23", "24", "25", "26", "27", "28", "29", "30", "ERROR"};
  return respond[input];
}

const char *get_decode_element_string(osd_element_t *input, osd_element_attrs_t status) {
  switch (status) {
  case ACTIVE:
    if (input->active)
      return "ACTIVE  ";
    else
      return "INACTIVE";
  case ATTRIBUTE:
    if (osd_attr(input))
      return "NORMAL";
    else
      return "INVERT";
  case POSITIONX:
    return get_position_string(input->pos_x);
  case POSITIONY:
    return get_position_string(input->pos_y);
  }
  return 0;
}

uint32_t *osd_elements() {
  if (osd_system == OSD_SYS_HD) {
    return profile.osd.elements_hd;
  }
  return profile.osd.elements;
}

void osd_display_reset() {
  osd_state.element = OSD_CALLSIGN;

  osd_state.screen = OSD_SCREEN_REGULAR;
  osd_state.screen_phase = 0;
  osd_state.screen_history_size = 0;

  osd_state.cursor = 1;
  osd_state.cursor_history_size = 0;
}

static void osd_update_screen(osd_screens_t screen) {
  osd_state.screen = screen;
  osd_state.screen_phase = 0;
}

osd_screens_t osd_push_screen(osd_screens_t screen) {
  osd_state.screen_history[osd_state.screen_history_size] = osd_state.screen;
  osd_update_screen(screen);
  osd_state.screen_history_size++;

  osd_state.cursor = 1;

  return screen;
}

osd_screens_t osd_pop_screen() {
  if (osd_state.screen_history_size <= 1) {
    // first history entry is always the REGULAR display
    // clear everything off the screen and reset
    osd_update_screen(OSD_SCREEN_CLEAR);
    return OSD_SCREEN_CLEAR;
  }

  const uint32_t screen = osd_state.screen_history[osd_state.screen_history_size - 1];
  osd_update_screen(screen);
  osd_state.screen_history_size--;

  return screen;
}

void osd_handle_input(osd_input_t input) {
  switch (input) {
  case OSD_INPUT_UP:
    if (osd_state.selection) {
      osd_state.selection_increase = 1;
    } else {
      if (osd_state.cursor == 1) {
        osd_state.cursor = osd_state.cursor_max - osd_state.cursor_min + 1;
      } else {
        osd_state.cursor--;
      }
      osd_state.screen_phase = 1;
    }
    break;

  case OSD_INPUT_DOWN:
    if (osd_state.selection) {
      osd_state.selection_decrease = 1;
    } else {
      if (osd_state.cursor == (osd_state.cursor_max - osd_state.cursor_min + 1)) {
        osd_state.cursor = 1;
      } else {
        osd_state.cursor++;
      }
      osd_state.screen_phase = 1;
    }
    break;

  case OSD_INPUT_LEFT:
    if (osd_state.selection) {
      osd_state.selection--;
      osd_state.screen_phase = 1;
    } else {
      osd_pop_cursor();
      osd_pop_screen();
    }
    break;

  case OSD_INPUT_RIGHT:
    osd_state.selection++;
    osd_state.screen_phase = 1;
    break;
  }
}

static bool osd_is_selected(const osd_label_t *labels, const uint8_t size) {
  if (osd_state.screen_phase > 1 && osd_state.screen_phase <= osd_state.cursor_min) {
    return false;
  }

  if (osd_state.cursor == (osd_state.screen_phase - osd_state.cursor_min) && osd_state.selection == 0) {
    return true;
  }

  return false;
}

void osd_update_cursor(const osd_label_t *labels, const uint8_t size) {
  osd_state.cursor_min = size;
  osd_state.cursor_max = 0;

  for (uint32_t i = 0; i < size; i++) {
    if (labels[i].type != OSD_LABEL_ACTIVE) {
      continue;
    }
    if (i < osd_state.cursor_min) {
      osd_state.cursor_min = i;
    }
    if (i > osd_state.cursor_max) {
      osd_state.cursor_max = i;
    }
  }
}

uint8_t grid_selection(uint8_t element, uint8_t row) {
  if (osd_state.selection == element && osd_state.cursor == row) {
    return OSD_ATTR_INVERT;
  } else {
    return OSD_ATTR_TEXT;
  }
}

//************************************************************************************************************************************************************************************
//																					PRINT FUNCTIONS
//************************************************************************************************************************************************************************************

void print_osd_flightmode(osd_element_t *el) {
  const uint8_t flightmode_labels[5][21] = {
      {"   ACRO   "},
      {"  LEVEL   "},
      {" RACEMODE "},
      {" HORIZON  "},
      {"RM HORIZON"},
  };

  uint8_t flightmode;
  if (rx_aux_on(AUX_LEVELMODE)) {
    if (rx_aux_on(AUX_RACEMODE) && rx_aux_on(AUX_HORIZON))
      flightmode = 4;
    if (!rx_aux_on(AUX_RACEMODE) && rx_aux_on(AUX_HORIZON))
      flightmode = 3;
    if (rx_aux_on(AUX_RACEMODE) && !rx_aux_on(AUX_HORIZON))
      flightmode = 2;
    if (!rx_aux_on(AUX_RACEMODE) && !rx_aux_on(AUX_HORIZON))
      flightmode = 1;
  } else {
    flightmode = 0;
  }

  osd_transaction_t *txn = osd_txn_init();
  osd_txn_start(osd_attr(el), el->pos_x, el->pos_y);
  osd_txn_write_data(flightmode_labels[flightmode], 21);
  osd_txn_submit(txn);
}

void print_osd_rssi(osd_element_t *el) {
  static float rx_rssi_filt;
  if (flags.failsafe)
    state.rx_rssi = 0.0f;

  lpf(&rx_rssi_filt, state.rx_rssi, FILTERCALC(state.looptime * 1e6f * 133.0f, 2e6f)); // 2 second filtertime and 15hz refresh rate @4k, 30hz@ 8k loop

  osd_transaction_t *txn = osd_txn_init();
  osd_txn_start(osd_attr(el), el->pos_x, el->pos_y);
  osd_txn_write_uint(rx_rssi_filt - 0.5f, 4);
  osd_txn_write_char(ICON_RSSI);
  osd_txn_submit(txn);
}

void print_osd_armtime(osd_element_t *el) {
  uint32_t time_s = state.armtime;

  // Will only display up to 59:59 as realistically no quad will fly that long (currently).
  // Reset to zero at on reaching 1 hr
  while (time_s >= 3600) {
    time_s -= 3600;
  }

  osd_transaction_t *txn = osd_txn_init();
  osd_txn_start(osd_attr(el), el->pos_x, el->pos_y);

  const uint32_t minutes = time_s / 60;
  osd_txn_write_uint(minutes % 10, 1);
  osd_txn_write_uint(minutes / 10, 1);

  osd_txn_write_char(':');

  const uint32_t seconds = time_s % 60;
  osd_txn_write_uint(seconds % 10, 1);
  osd_txn_write_uint(seconds / 10, 1);

  osd_txn_submit(txn);
}

// print the current vtx settings as Band:Channel:Power
void print_osd_vtx(osd_element_t *el) {
  uint8_t str[5];

  switch (vtx_settings.band) {
  case VTX_BAND_A:
    str[0] = 'A';
    break;
  case VTX_BAND_B:
    str[0] = 'B';
    break;
  case VTX_BAND_E:
    str[0] = 'E';
    break;
  case VTX_BAND_F:
    str[0] = 'F';
    break;
  case VTX_BAND_R:
    str[0] = 'R';
    break;
  default:
    str[0] = 'M';
    break;
  }

  str[1] = ':';
  str[2] = vtx_settings.channel + 49;
  str[3] = ':';

  if (vtx_settings.pit_mode == 1) {
    str[4] = 21; // "pit", probably from Pitch, but we will use it here
  } else {
    if (vtx_settings.power_level == 4)
      str[4] = 36; // "max"
    else
      str[4] = vtx_settings.power_level + 49;
  }

  osd_transaction_t *txn = osd_txn_init();
  osd_txn_start(osd_attr(el), el->pos_x, el->pos_y);
  osd_txn_write_data(str, 5);
  osd_txn_submit(txn);
}

// 3 stage return - 0 = stick and hold, 1 = move on but come back to clear, 2 = status print done
uint8_t print_status(osd_element_t *el, uint8_t persistence, uint8_t label) {
  const uint8_t system_status_labels[12][21] = {
      {"               "},
      {" **DISARMED**  "},
      {"  **ARMED**    "},
      {" STICK BOOST 1 "},
      {" STICK BOOST 2 "},
      {" **FAILSAFE**  "},
      {"THROTTLE SAFETY"},
      {" ARMING SAFETY "},
      {"**LOW BATTERY**"},
      {"**MOTOR TEST** "},
      {"  **TURTLE**   "},
      {" **LOOPTIME**  "},
  };

  static uint8_t last_label;
  static uint8_t delay_counter = 25;
  if (last_label != label) { // critical to reset indexers if a print sequence is interrupted by a new request
    delay_counter = 25;
  }
  last_label = label;

  if (delay_counter == 25) {
    // First run, print the label
    osd_transaction_t *txn = osd_txn_init();
    osd_txn_start(osd_attr(el) | OSD_ATTR_BLINK, el->pos_x, el->pos_y);
    osd_txn_write_data(system_status_labels[label], 21);
    osd_txn_submit(txn);

    delay_counter--;

    return 0;
  }

  if (persistence == HOLD) {
    // label printed and we should hold
    return 2;
  }

  // label printed and its temporary
  if (!delay_counter) {
    // timer is elapsed, print clear label
    osd_transaction_t *txn = osd_txn_init();
    osd_txn_start(osd_attr(el), el->pos_x, el->pos_y);
    osd_txn_write_data(system_status_labels[0], 21);
    osd_txn_submit(txn);

    delay_counter = 25;

    return 2;
  }

  delay_counter--;
  return 1;
}

uint8_t print_osd_system_status(osd_element_t *el) {
  static uint8_t ready = 0;
  uint8_t print_stage = 2; // 0 makes the main osd function stick and non zero lets it pass on

  static osd_sys_status_t last_sys_status;
  static osd_sys_status_t printing = {
      .loop_warning = 0,
      .aux_boost = 0,
      .aux_motor_test = 0,
      .flags = {0},
  };

  extern uint8_t looptime_warning;

  // things happen here
  if (ready) {
    if ((flags.arm_state != last_sys_status.flags.arm_state) || printing.flags.arm_state) {
      last_sys_status.flags.arm_state = flags.arm_state;
      printing.flags.arm_state = 1;
      if (flags.arm_state)
        print_stage = print_status(el, TEMP, ARM);
      else
        print_stage = print_status(el, TEMP, DISARM);
      if (print_stage == 2)
        printing.flags.arm_state = 0;
      return print_stage;
    }
    if (((looptime_warning != last_sys_status.loop_warning) || printing.loop_warning) && (state.looptime_autodetect != LOOPTIME_8K)) { // mute warnings till we are on the edge of 4k->2k
      last_sys_status.loop_warning = looptime_warning;
      printing.loop_warning = 1;
      print_stage = print_status(el, TEMP, LOOP);
      if (print_stage == 2)
        printing.loop_warning = 0;
      return print_stage;
    }
    if (rx_aux_on(AUX_STICK_BOOST_PROFILE) != last_sys_status.aux_boost || printing.aux_boost) {
      last_sys_status.aux_boost = rx_aux_on(AUX_STICK_BOOST_PROFILE);
      printing.aux_boost = 1;
      if (rx_aux_on(AUX_STICK_BOOST_PROFILE))
        print_stage = print_status(el, TEMP, BOOST_2);
      else
        print_stage = print_status(el, TEMP, BOOST_1);
      if (print_stage == 2)
        printing.aux_boost = 0;
      return print_stage;
    }
    if (flags.failsafe != last_sys_status.flags.failsafe || printing.flags.failsafe) {
      last_sys_status.flags.failsafe = flags.failsafe;
      printing.flags.failsafe = 1;
      if (flags.failsafe)
        print_stage = print_status(el, HOLD, FAILSAFE);
      else
        print_stage = print_status(el, HOLD, CLEAR);
      if (print_stage == 2)
        printing.flags.failsafe = 0;
      return print_stage;
    }
    if ((flags.arm_safety && !flags.failsafe) || printing.flags.arm_safety) {
      printing.flags.arm_safety = 1;
      if (flags.arm_safety) {
        print_stage = print_status(el, HOLD, ARM_SFTY);
      } else {
        print_stage = print_status(el, HOLD, CLEAR);
        if (print_stage == 2)
          printing.flags.arm_safety = 0;
      }
      return print_stage;
    }
    if ((flags.throttle_safety && !flags.arm_safety) || printing.flags.throttle_safety) {
      printing.flags.throttle_safety = 1;
      if (flags.throttle_safety) {
        print_stage = print_status(el, HOLD, THRTL_SFTY);
      } else {
        print_stage = print_status(el, HOLD, CLEAR);
        if (print_stage == 2)
          printing.flags.throttle_safety = 0;
      }
      return print_stage;
    }
    if ((flags.lowbatt != last_sys_status.flags.lowbatt && !flags.arm_safety && !flags.throttle_safety && !flags.failsafe) || printing.flags.lowbatt) {
      last_sys_status.flags.lowbatt = flags.lowbatt;
      printing.flags.lowbatt = 1;
      if (flags.lowbatt)
        print_stage = print_status(el, HOLD, LOW_BAT);
      else
        print_stage = print_status(el, HOLD, CLEAR);
      if (print_stage == 2)
        printing.flags.lowbatt = 0;
      return print_stage;
    }
    if ((rx_aux_on(AUX_MOTOR_TEST) && !flags.arm_safety && !flags.throttle_safety && !flags.failsafe) || (printing.aux_motor_test && !flags.arm_safety && !flags.throttle_safety && !flags.failsafe)) {
      printing.aux_motor_test = 1;
      if (rx_aux_on(AUX_MOTOR_TEST)) {
        print_stage = print_status(el, HOLD, MOTOR_TEST);
      } else {
        print_stage = print_status(el, HOLD, CLEAR);
        if (print_stage == 2)
          printing.aux_motor_test = 0;
      }
      return print_stage;
    }
    if ((flags.turtle && !flags.arm_safety && !flags.throttle_safety && !flags.failsafe) || (printing.flags.turtle && !flags.arm_safety && !flags.throttle_safety && !flags.failsafe)) {
      printing.flags.turtle = 1;
      if (flags.turtle) {
        print_stage = print_status(el, HOLD, TURTL);
      } else {
        print_stage = print_status(el, HOLD, CLEAR);
        if (print_stage == 2)
          printing.flags.turtle = 0;
      }
      return print_stage;
    }
  }
  if (ready == 0) {
    last_sys_status.loop_warning = looptime_warning;
    last_sys_status.aux_boost = rx_aux_on(AUX_STICK_BOOST_PROFILE);
    last_sys_status.aux_motor_test = rx_aux_on(AUX_MOTOR_TEST);
    last_sys_status.flags = flags;
    ready = 1;
  }
  return ready;
}

void print_osd_callsign_adjustable(uint8_t string_element_qty, uint8_t data_element_qty, const uint8_t grid[data_element_qty][2], const uint8_t print_position[data_element_qty][2]) {
  if (osd_state.screen_phase <= string_element_qty)
    return;
  if (osd_state.screen_phase > string_element_qty + data_element_qty)
    return;
  static uint8_t skip_loop = 0;
  if (osd_state.screen_phase == string_element_qty + 1 && skip_loop == 0) { // skip a loop to prevent dma collision with previous print function
    skip_loop++;
    return;
  }
  skip_loop = 0;
  uint8_t index = osd_state.screen_phase - string_element_qty - 1;

  osd_transaction_t *txn = osd_txn_init();
  osd_txn_start(grid_selection(grid[index][0], grid[index][1]), print_position[index][0], print_position[index][1]);
  osd_txn_write_char(profile.osd.callsign[index]);
  osd_txn_submit(txn);

  osd_state.screen_phase++;
}

void print_osd_menu_strings(const osd_label_t *labels, const uint8_t size) {
  if (osd_state.screen_phase > size) {
    return;
  }
  if (osd_state.screen_phase == 0) {
    if (osd_clear_async()) {
      osd_update_cursor(labels, size);
      osd_state.screen_phase++;
    }
    return;
  }

  const osd_label_t *label = &labels[osd_state.screen_phase - 1];

  osd_transaction_t *txn = osd_txn_init();

  switch (label->type) {
  case OSD_LABEL_HEADER:
    osd_txn_start(OSD_ATTR_INVERT, label->pos[0], label->pos[1]);
    break;

  case OSD_LABEL_INACTIVE:
    osd_txn_start(OSD_ATTR_TEXT, label->pos[0], label->pos[1]);
    break;

  case OSD_LABEL_ACTIVE: {
    const bool is_selected = osd_is_selected(labels, size);
    if (osd_system == OSD_SYS_HD) {
      if (is_selected) {
        osd_txn_start(OSD_ATTR_INVERT, label->pos[0] - 1, label->pos[1]);
        osd_txn_write_char('>');
      } else {
        osd_txn_start(OSD_ATTR_TEXT, label->pos[0] - 1, label->pos[1]);
        osd_txn_write_char(' ');
      }
    } else {
      if (is_selected) {
        osd_txn_start(OSD_ATTR_INVERT, label->pos[0], label->pos[1]);
      } else {
        osd_txn_start(OSD_ATTR_TEXT, label->pos[0], label->pos[1]);
      }
    }

    break;
  }
  }

  osd_txn_write_str(label->text);
  osd_txn_submit(txn);

  osd_state.screen_phase++;
}

void print_osd_adjustable_enums(uint8_t string_element_qty, uint8_t data_element_qty, const char data_to_print[21], const uint8_t grid[data_element_qty][2], const uint8_t print_position[data_element_qty][2]) {
  if (osd_state.screen_phase <= string_element_qty)
    return;
  if (osd_state.screen_phase > string_element_qty + data_element_qty)
    return;
  static uint8_t skip_loop = 0;
  if (osd_state.screen_phase == string_element_qty + 1 && skip_loop == 0) { // skip a loop to prevent dma collision with previous print function
    skip_loop++;
    return;
  }
  skip_loop = 0;
  uint8_t index = osd_state.screen_phase - string_element_qty - 1;

  osd_transaction_t *txn = osd_txn_init();
  osd_txn_start(grid_selection(grid[index][0], grid[index][1]), print_position[index][0], print_position[index][1]);
  osd_txn_write_str(data_to_print);
  osd_txn_submit(txn);

  osd_state.screen_phase++;
}

void print_osd_adjustable_vectors(uint8_t menu_type, uint8_t string_element_qty, uint8_t data_element_qty, vec3_t *pointer, const uint8_t data_index[data_element_qty][2], const uint8_t grid[data_element_qty][2], const uint8_t print_position[data_element_qty][2]) {
  if (osd_state.screen_phase <= string_element_qty)
    return;
  if (osd_state.screen_phase > string_element_qty + data_element_qty)
    return;
  static uint8_t skip_loop = 0;
  if (osd_state.screen_phase == string_element_qty + 1 && skip_loop == 0) { // skip a loop to prevent dma collision with previous print function
    skip_loop++;
    return;
  }
  skip_loop = 0;

  uint8_t index = osd_state.screen_phase - string_element_qty - 1;

  osd_transaction_t *txn = osd_txn_init();
  osd_txn_start(grid_selection(grid[index][0], grid[index][1]), print_position[index][0], print_position[index][1]);

  switch (menu_type) {
  case BF_PIDS:
    osd_txn_write_uint(pointer->axis[data_index[index][1]], 4);
    break;
  case SW_RATES:
    if (index < 3)
      osd_txn_write_uint(pointer->axis[data_index[index][1]], 4);
    else
      osd_txn_write_float(pointer->axis[data_index[index][1]] + FLT_EPSILON, 5, 2);
    break;
  case ROUNDED:
    osd_txn_write_float(pointer->axis[data_index[index][1]] + FLT_EPSILON, 5, 2);
    break;
  }
  osd_txn_submit(txn);

  osd_state.screen_phase++;
}

void print_osd_adjustable_float(uint8_t string_element_qty, uint8_t data_element_qty, float *pointer[], const uint8_t grid[data_element_qty][2], const uint8_t print_position[data_element_qty][2], uint8_t precision) {
  if (osd_state.screen_phase <= string_element_qty)
    return;
  if (osd_state.screen_phase > string_element_qty + data_element_qty)
    return;
  static uint8_t skip_loop = 0;
  if (osd_state.screen_phase == string_element_qty + 1 && skip_loop == 0) { // skip a loop to prevent dma collision with previous print function
    skip_loop++;
    return;
  }
  skip_loop = 0;

  uint8_t index = osd_state.screen_phase - string_element_qty - 1;

  osd_transaction_t *txn = osd_txn_init();
  osd_txn_start(grid_selection(grid[index][0], grid[index][1]), print_position[index][0], print_position[index][1]);
  osd_txn_write_float(*pointer[index] + FLT_EPSILON, 4, precision);
  osd_txn_submit(txn);

  osd_state.screen_phase++;
}

//************************************************************************************************************************************************************************************
//************************************************************************************************************************************************************************************
//																				MAIN OSD DISPLAY FUNCTION
//************************************************************************************************************************************************************************************
//************************************************************************************************************************************************************************************
void osd_init() {
  osd_device_init();

  // print the splash screen
  osd_intro();
}

static void osd_display_regular() {
  osd_element_t *el = (osd_element_t *)(osd_elements() + osd_state.element);
  if (osd_state.element < OSD_ELEMENT_MAX && !el->active) {
    osd_state.element++;
    return;
  }

  switch (osd_state.element) {
  case OSD_CALLSIGN: {
    osd_transaction_t *txn = osd_txn_init();
    osd_txn_start(osd_attr(el), el->pos_x, el->pos_y);
    osd_txn_write_str((const char *)profile.osd.callsign);
    osd_txn_submit(txn);
    osd_state.element++;
    break;
  }

  case OSD_FUELGAUGE_VOLTS: {
    osd_transaction_t *txn = osd_txn_init();
    if (!flags.lowbatt) {
      osd_txn_start(osd_attr(el), el->pos_x, el->pos_y);
    } else {
      osd_txn_start(OSD_ATTR_BLINK | OSD_ATTR_INVERT, el->pos_x, el->pos_y);
    }
    osd_txn_write_uint(state.lipo_cell_count, 1);
    osd_txn_write_char('S');

    osd_txn_start(osd_attr(el), el->pos_x + 3, el->pos_y);
    osd_txn_write_float(state.vbatt_comp, 4, 1);
    osd_txn_write_char('V');

    osd_txn_submit(txn);
    osd_state.element++;
    break;
  }

  case OSD_FILTERED_VOLTS: {
    osd_transaction_t *txn = osd_txn_init();
    if (!flags.lowbatt) {
      osd_txn_start(osd_attr(el), el->pos_x, el->pos_y);
    } else {
      osd_txn_start(OSD_ATTR_BLINK | OSD_ATTR_INVERT, el->pos_x, el->pos_y);
    }
    osd_txn_write_uint(state.lipo_cell_count, 1);
    osd_txn_write_char('S');

    osd_txn_start(osd_attr(el), el->pos_x + 3, el->pos_y);
    osd_txn_write_float(state.vbattfilt_corr, 4, 1);
    osd_txn_write_char('V');

    osd_txn_submit(txn);
    osd_state.element++;
    break;
  }

  case OSD_GYRO_TEMP: {
    osd_transaction_t *txn = osd_txn_init();
    osd_txn_start(osd_attr(el), el->pos_x, el->pos_y);
    osd_txn_write_uint(state.gyro_temp, 4);
    osd_txn_write_char(ICON_CELSIUS);
    osd_txn_submit(txn);
    osd_state.element++;
    break;
  }

  case OSD_FLIGHT_MODE: {
    print_osd_flightmode(el);
    osd_state.element++;
    break;
  }

  case OSD_RSSI: {
    print_osd_rssi(el);
    osd_state.element++;
    break;
  }

  case OSD_STOPWATCH: {
    print_osd_armtime(el);
    osd_state.element++;
    break;
  }

  case OSD_SYSTEM_STATUS: {
    if (print_osd_system_status(el))
      osd_state.element++;
    break;
  }

  case OSD_THROTTLE: {
    osd_transaction_t *txn = osd_txn_init();
    osd_txn_start(osd_attr(el), el->pos_x, el->pos_y);
    osd_txn_write_uint(state.throttle * 100.0f, 4);
    osd_txn_write_char(ICON_THROTTLE);
    osd_txn_submit(txn);
    osd_state.element++;
    break;
  }

  case OSD_VTX_CHANNEL: {
    print_osd_vtx(el);
    osd_state.element++;
    break;
  }

  case OSD_CURRENT_DRAW: {
    osd_transaction_t *txn = osd_txn_init();
    osd_txn_start(osd_attr(el), el->pos_x, el->pos_y);
    osd_txn_write_float(state.ibat_filtered / 1000.0f, 4, 2);
    osd_txn_write_char(ICON_AMP);
    osd_txn_submit(txn);
    osd_state.element++;
    break;
  }

  case OSD_ELEMENT_MAX: {
    // end of regular display - display_trigger counter sticks here till it wraps
    static uint8_t display_trigger = 0;
    display_trigger++;
    if (display_trigger == 0)
      osd_state.element = OSD_CALLSIGN;
    break;
  }
  }
}

void osd_display_rate_menu() {
  osd_menu_start();

  switch (profile_current_rates()->mode) {
  case RATE_MODE_SILVERWARE:
    osd_menu_header("SILVERWARE RATES");
    break;
  case RATE_MODE_BETAFLIGHT:
    osd_menu_header("BETAFLIGHT RATES");
    break;
  case RATE_MODE_ACTUAL:
    osd_menu_header("ACTUAL RATES");
    break;
  }

  const char *mode_labels[] = {
      "SILVERWARE",
      "BETAFLIGHT",
      "ACTUAL",
  };
  osd_menu_select(2, 3, "MODE");
  if (osd_menu_select_enum(14, 3, profile_current_rates()->mode, mode_labels)) {
    profile_current_rates()->mode = osd_menu_adjust_int(profile_current_rates()->mode, 1, 0, RATE_MODE_ACTUAL);
  }

  osd_menu_label(14, 5, "ROLL");
  osd_menu_label(19, 5, "PITCH");
  osd_menu_label(25, 5, "YAW");

  vec3_t *rate = profile_current_rates()->rate;
  switch (profile_current_rates()->mode) {
  case RATE_MODE_SILVERWARE: {
    osd_menu_select(2, 6, "RATE");
    if (osd_menu_select_vec3(13, 6, rate[SILVERWARE_MAX_RATE], 5, 0)) {
      rate[SILVERWARE_MAX_RATE] = osd_menu_adjust_vec3(rate[SILVERWARE_MAX_RATE], 10, 0.0, 1800.0);
    }

    osd_menu_select(2, 7, "ACRO EXPO");
    if (osd_menu_select_vec3(13, 7, rate[SILVERWARE_ACRO_EXPO], 5, 2)) {
      rate[SILVERWARE_ACRO_EXPO] = osd_menu_adjust_vec3(rate[SILVERWARE_ACRO_EXPO], 0.01, 0.0, 0.99);
    }

    osd_menu_select(2, 8, "ANGLE EXPO");
    if (osd_menu_select_vec3(13, 8, rate[SILVERWARE_ANGLE_EXPO], 5, 2)) {
      rate[SILVERWARE_ANGLE_EXPO] = osd_menu_adjust_vec3(rate[SILVERWARE_ANGLE_EXPO], 0.01, 0.0, 0.99);
    }
    break;
  }
  case RATE_MODE_BETAFLIGHT: {
    osd_menu_select(2, 6, "RC RATE");
    if (osd_menu_select_vec3(13, 6, rate[BETAFLIGHT_RC_RATE], 5, 2)) {
      rate[BETAFLIGHT_RC_RATE] = osd_menu_adjust_vec3(rate[BETAFLIGHT_RC_RATE], 0.01, 0.0, 3.0);
    }

    osd_menu_select(2, 7, "SUPER RATE");
    if (osd_menu_select_vec3(13, 7, rate[BETAFLIGHT_SUPER_RATE], 5, 2)) {
      rate[BETAFLIGHT_SUPER_RATE] = osd_menu_adjust_vec3(rate[BETAFLIGHT_SUPER_RATE], 0.01, 0.0, 3.0);
    }

    osd_menu_select(2, 8, "EXPO");
    if (osd_menu_select_vec3(13, 8, rate[BETAFLIGHT_EXPO], 5, 2)) {
      rate[BETAFLIGHT_EXPO] = osd_menu_adjust_vec3(rate[BETAFLIGHT_EXPO], 0.01, 0.0, 0.99);
    }
    break;
  }
  case RATE_MODE_ACTUAL: {
    osd_menu_select(2, 6, "CENTER");
    if (osd_menu_select_vec3(13, 6, rate[ACTUAL_CENTER_SENSITIVITY], 5, 2)) {
      rate[ACTUAL_CENTER_SENSITIVITY] = osd_menu_adjust_vec3(rate[ACTUAL_CENTER_SENSITIVITY], 1.0, 0.0, 500);
    }

    osd_menu_select(2, 7, "MAX RATE");
    if (osd_menu_select_vec3(13, 7, rate[ACTUAL_MAX_RATE], 5, 2)) {
      rate[ACTUAL_MAX_RATE] = osd_menu_adjust_vec3(rate[ACTUAL_MAX_RATE], 10, 0.0, 1800);
    }

    osd_menu_select(2, 8, "EXPO");
    if (osd_menu_select_vec3(13, 8, rate[ACTUAL_EXPO], 5, 2)) {
      rate[ACTUAL_EXPO] = osd_menu_adjust_vec3(rate[ACTUAL_EXPO], 0.01, 0.0, 0.99);
    }
    break;
  }
  }

  osd_menu_select_save_and_exit(2, 14);

  osd_menu_finish();
}

void osd_display() {
  if (!osd_is_ready()) {
    return;
  }

  // check if the system changed
  const osd_system_t sys = osd_check_system();
  if (sys != osd_system) {
    // sytem has changed, reset osd state
    osd_system = sys;
    osd_display_reset();
    return;
  }
  switch (osd_state.screen) {
  case OSD_SCREEN_CLEAR:
    if (osd_clear_async())
      osd_display_reset();
    break;

  case OSD_SCREEN_REGULAR:
    osd_display_regular();
    break;

    //**********************************************************************************************************************************************************************************************
    //																				OSD MENUS BELOW THIS POINT
    //**********************************************************************************************************************************************************************************************

  case OSD_SCREEN_MAIN_MENU: {
    osd_menu_start();
    osd_menu_header("MENU");

    osd_menu_select_screen(7, 3, "VTX", OSD_SCREEN_VTX);
    osd_menu_select_screen(7, 4, "PIDS", OSD_SCREEN_PID_PROFILE);
    osd_menu_select_screen(7, 5, "FILTERS", OSD_SCREEN_FILTERS);
    osd_menu_select_screen(7, 6, "RATES", OSD_SCREEN_RATE_PROFILES);
    osd_menu_select_screen(7, 7, "FLIGHT MODES", OSD_SCREEN_FLIGHT_MODES);
    osd_menu_select_screen(7, 8, "OSD ELEMENTS", OSD_SCREEN_ELEMENTS);
    osd_menu_select_screen(7, 9, "SPECIAL FEATURES", OSD_SCREEN_SPECIAL_FEATURES);
    osd_menu_select_screen(7, 10, "RC LINK", OSD_SCREEN_RC_LINK);

    osd_menu_select_save_and_exit(7, 11);
    osd_menu_finish();
    break;
  }

  case OSD_SCREEN_PID_PROFILE:
    osd_menu_start();
    osd_menu_header("PID PROFILES");

    if (osd_menu_button(7, 4, "PID PROFILE 1")) {
      profile.pid.pid_profile = PID_PROFILE_1;
      osd_push_cursor();
      osd_push_screen(OSD_SCREEN_PID);
    }

    if (osd_menu_button(7, 5, "PID PROFILE 2")) {
      profile.pid.pid_profile = PID_PROFILE_2;
      osd_push_cursor();
      osd_push_screen(OSD_SCREEN_PID);
    }

    osd_menu_finish();
    break;

  case OSD_SCREEN_PID:
    osd_menu_start();

    if (profile.pid.pid_profile == PID_PROFILE_1)
      osd_menu_header("PID PROFILE 1");
    else
      osd_menu_header("PID PROFILE 2");

    osd_menu_label(10, 4, "ROLL");
    osd_menu_label(16, 4, "PITCH");
    osd_menu_label(23, 4, "YAW");

    pid_rate_t *rates = profile_current_pid_rates();

    osd_menu_select(4, 6, "KP");
    if (osd_menu_select_vec3(8, 6, rates->kp, 6, 0)) {
      rates->kp = osd_menu_adjust_vec3(rates->kp, 1, 0.0, 400.0);
    }

    osd_menu_select(4, 7, "KI");
    if (osd_menu_select_vec3(8, 7, rates->ki, 6, 0)) {
      rates->ki = osd_menu_adjust_vec3(rates->ki, 1, 0.0, 100.0);
    }

    osd_menu_select(4, 8, "KD");
    if (osd_menu_select_vec3(8, 8, rates->kd, 6, 0)) {
      rates->kd = osd_menu_adjust_vec3(rates->kd, 1, 0.0, 120.0);
    }

    osd_menu_select_save_and_exit(7, 11);
    osd_menu_finish();
    break;

  case OSD_SCREEN_RATE_PROFILES:
    osd_menu_start();
    osd_menu_header("RATE PROFILES");

    if (osd_menu_button(7, 4, "PROFILE 1")) {
      profile.rate.profile = STICK_RATE_PROFILE_1;
      osd_push_cursor();
      osd_push_screen(OSD_SCREEN_RATES);
    }

    if (osd_menu_button(7, 5, "PROFILE 2")) {
      profile.rate.profile = STICK_RATE_PROFILE_2;
      osd_push_cursor();
      osd_push_screen(OSD_SCREEN_RATES);
    }

    osd_menu_finish();
    break;

  case OSD_SCREEN_RATES:
    osd_display_rate_menu();
    break;

  case OSD_SCREEN_FILTERS:
    osd_menu_start();
    osd_menu_header("FILTERS");

    osd_menu_select_screen(7, 4, "GYRO", OSD_SCREEN_GYRO_FILTER);
    osd_menu_select_screen(7, 5, "D-TERM", OSD_SCREEN_DTERM_FILTER);

    osd_menu_finish();
    break;

  case OSD_SCREEN_GYRO_FILTER: {
    osd_menu_start();
    osd_menu_header("GYRO FILTERS");

    const char *filter_type_labels[] = {
        "NONE",
        " PT1",
        " PT2",
    };

    osd_menu_select(4, 4, "PASS 1 TYPE");
    if (osd_menu_select_enum(18, 4, profile.filter.gyro[0].type, filter_type_labels)) {
      profile.filter.gyro[0].type = osd_menu_adjust_int(profile.filter.gyro[0].type, 1, 0, FILTER_LP2_PT1);
      osd_state.reboot_fc_requested = 1;
    }

    osd_menu_select(4, 5, "PASS 1 FREQ");
    if (osd_menu_select_float(18, 5, profile.filter.gyro[0].cutoff_freq, 4, 0)) {
      profile.filter.gyro[0].cutoff_freq = osd_menu_adjust_float(profile.filter.gyro[0].cutoff_freq, 10, 50, 500);
    }

    osd_menu_select(4, 6, "PASS 2 TYPE");
    if (osd_menu_select_enum(18, 6, profile.filter.gyro[1].type, filter_type_labels)) {
      profile.filter.gyro[1].type = osd_menu_adjust_int(profile.filter.gyro[1].type, 1, 0, FILTER_LP2_PT1);
      osd_state.reboot_fc_requested = 1;
    }

    osd_menu_select(4, 7, "PASS 2 FREQ");
    if (osd_menu_select_float(18, 7, profile.filter.gyro[1].cutoff_freq, 4, 0)) {
      profile.filter.gyro[1].cutoff_freq = osd_menu_adjust_float(profile.filter.gyro[1].cutoff_freq, 10, 50, 500);
    }

    osd_menu_select_save_and_exit(4, 14);
    osd_menu_finish();
    break;
  }

  case OSD_SCREEN_DTERM_FILTER:
    osd_menu_start();
    osd_menu_header("D-TERM FILTERS");

    const char *filter_type_labels[] = {
        "NONE",
        " PT1",
        " PT2",
    };

    osd_menu_select(4, 3, "PASS 1 TYPE");
    if (osd_menu_select_enum(18, 3, profile.filter.dterm[0].type, filter_type_labels)) {
      profile.filter.dterm[0].type = osd_menu_adjust_int(profile.filter.dterm[0].type, 1, 0, FILTER_LP2_PT1);
      osd_state.reboot_fc_requested = 1;
    }

    osd_menu_select(4, 4, "PASS 1 FREQ");
    if (osd_menu_select_float(18, 4, profile.filter.dterm[0].cutoff_freq, 4, 0)) {
      profile.filter.dterm[0].cutoff_freq = osd_menu_adjust_float(profile.filter.dterm[0].cutoff_freq, 10, 50, 500);
    }

    osd_menu_select(4, 5, "PASS 2 TYPE");
    if (osd_menu_select_enum(18, 5, profile.filter.dterm[1].type, filter_type_labels)) {
      profile.filter.dterm[1].type = osd_menu_adjust_int(profile.filter.dterm[1].type, 1, 0, FILTER_LP2_PT1);
      osd_state.reboot_fc_requested = 1;
    }

    osd_menu_select(4, 6, "PASS 2 FREQ");
    if (osd_menu_select_float(18, 6, profile.filter.dterm[1].cutoff_freq, 4, 0)) {
      profile.filter.dterm[1].cutoff_freq = osd_menu_adjust_float(profile.filter.dterm[1].cutoff_freq, 10, 50, 500);
    }

    osd_menu_select(4, 7, "DYNAMIC");
    if (osd_menu_select_enum(18, 7, profile.filter.dterm_dynamic_enable, filter_type_labels)) {
      profile.filter.dterm_dynamic_enable = osd_menu_adjust_int(profile.filter.dterm_dynamic_enable, 1, 0, FILTER_LP_PT1);
      osd_state.reboot_fc_requested = 1;
    }

    osd_menu_select(4, 8, "FREQ MIN");
    if (osd_menu_select_float(18, 8, profile.filter.dterm_dynamic_min, 4, 0)) {
      profile.filter.dterm_dynamic_min = osd_menu_adjust_float(profile.filter.dterm_dynamic_min, 10, 50, 500);
    }

    osd_menu_select(4, 9, "FREQ MAX");
    if (osd_menu_select_float(18, 9, profile.filter.dterm_dynamic_max, 4, 0)) {
      profile.filter.dterm_dynamic_max = osd_menu_adjust_float(profile.filter.dterm_dynamic_max, 10, 50, 500);
    }

    osd_menu_select_save_and_exit(4, 14);
    osd_menu_finish();
    break;

  case OSD_SCREEN_FLIGHT_MODES:
    print_osd_menu_strings(flight_modes_labels, flight_modes_labels_size);
    print_osd_adjustable_enums(12, 10, get_aux_status(profile.receiver.aux[flight_modes_aux_items[osd_state.screen_phase - 13]]), flight_modes_grid, flight_modes_data_positions);
    if (osd_state.screen_phase == 23)
      osd_enum_adjust(flight_modes_ptr, 10, flight_modes_aux_limits);
    break;

  case OSD_SCREEN_ELEMENTS:
    print_osd_menu_strings(osd_elements_menu_labels, osd_elements_menu_labels_size);
    if (osd_state.screen_phase == 6)
      osd_select_menu_item(4, osd_elements_map, SUB_MENU);
    break;

  case OSD_SCREEN_VTX:
    if (vtx_settings.detected) {
      populate_vtx_buffer_once();
      print_osd_menu_strings(vtx_labels, vtx_labels_size);
      // print the buffer and not the actual status
      print_osd_adjustable_enums(6, 4, get_vtx_status(osd_state.screen_phase - 7), vtx_grid, vtx_data_positions);
      if (osd_state.screen_phase == 11)
        // adjust the buffer and not the actual settings
        osd_enum_adjust(vtx_ptr, 4, vtx_limits);
      // save & exit needs a function to write the buffer to the actual settings
    } else {
      print_osd_menu_strings(vtx_na_labels, vtx_na_labels_size);
      if (osd_state.selection)
        osd_state.selection = 0;
    }
    break;

  case OSD_SCREEN_SPECIAL_FEATURES:
    print_osd_menu_strings(special_features_labels, special_features_labels_size);
    if (osd_state.screen_phase == 9)
      osd_select_menu_item(7, special_features_map, SUB_MENU);
    break;

  case OSD_SCREEN_STICK_BOOST:
    osd_menu_start();
    osd_menu_header("STICK BOOST PROFILES");

    if (osd_menu_button(7, 4, "AUX OFF PROFILE 1")) {
      profile.pid.stick_profile = STICK_PROFILE_OFF;
      osd_push_cursor();
      osd_push_screen(OSD_SCREEN_STICK_BOOST_ADJUST);
    }

    if (osd_menu_button(7, 5, "AUX ON  PROFILE 2")) {
      profile.pid.stick_profile = STICK_PROFILE_ON;
      osd_push_cursor();
      osd_push_screen(OSD_SCREEN_STICK_BOOST_ADJUST);
    }

    osd_menu_finish();
    break;

  case OSD_SCREEN_STICK_BOOST_ADJUST: {
    osd_menu_start();

    if (profile.pid.stick_profile == STICK_PROFILE_OFF)
      osd_menu_header("BOOST PROFILE 1");
    else
      osd_menu_header("BOOST PROFILE 2");

    osd_menu_label(14, 4, "ROLL");
    osd_menu_label(19, 4, "PITCH");
    osd_menu_label(25, 4, "YAW");

    stick_rate_t *rates = &profile.pid.stick_rates[profile.pid.stick_profile];

    osd_menu_select(2, 6, "ACCELERATOR");
    if (osd_menu_select_vec3(13, 6, rates->accelerator, 5, 2)) {
      rates->accelerator = osd_menu_adjust_vec3(rates->accelerator, 0.01, 0.0, 3.0);
    }

    osd_menu_select(2, 7, "TRANSITION");
    if (osd_menu_select_vec3(13, 7, rates->transition, 5, 2)) {
      rates->transition = osd_menu_adjust_vec3(rates->transition, 0.01, -1.0, -1.0);
    }

    osd_menu_select_save_and_exit(7, 11);
    osd_menu_finish();
    break;
  }

  case OSD_SCREEN_ELEMENTS_ADD_REMOVE:
    print_osd_menu_strings(osd_display_labels, osd_display_labels_size);
    print_osd_adjustable_enums(12, 10, get_decode_element_string((osd_element_t *)(osd_elements() + osd_elements_active_items[osd_state.screen_phase - 13]), ACTIVE), osd_display_grid, osd_display_data_positions);
    if (osd_state.screen_phase == 23)
      osd_encoded_adjust(osd_elements() + osd_elements_active_items[osd_state.cursor - 1], 10, 1, ACTIVE);
    break;

  case OSD_SCREEN_ELEMENTS_POSITION:
    print_osd_menu_strings(osd_position_labels, osd_position_labels_size);
    print_osd_adjustable_enums(14, 20, get_decode_element_string((osd_element_t *)(osd_elements() + osd_position_active_items[osd_state.screen_phase - 15]), osd_position_index[osd_state.screen_phase - 15]), osd_position_grid, osd_position_data_positions);
    if (osd_state.screen_phase == 35 && osd_state.selection > 0)
      osd_encoded_adjust(osd_elements() + osd_elements_active_items[osd_state.cursor - 1], 10, 2, osd_state.selection + 1);
    break;

  case OSD_SCREEN_ELEMENTS_STYLE:
    print_osd_menu_strings(osd_text_style, osd_text_style_size);
    print_osd_adjustable_enums(12, 10, get_decode_element_string((osd_element_t *)(osd_elements() + osd_elements_active_items[osd_state.screen_phase - 13]), ATTRIBUTE), osd_display_grid, osd_display_data_positions);
    if (osd_state.screen_phase == 23)
      osd_encoded_adjust(osd_elements() + osd_elements_active_items[osd_state.cursor - 1], 10, 1, ATTRIBUTE);
    break;

  case OSD_SCREEN_CALLSIGN:
    print_osd_menu_strings(osd_callsign_edit_labels, osd_callsign_edit_labels_size);
    print_osd_callsign_adjustable(23, 20, osd_callsign_grid, osd_callsign_edit_data_positions);
    if (osd_state.screen_phase == 44)
      osd_encoded_adjust_callsign();
    break;

  case OSD_SCREEN_LOWBAT:
    print_osd_menu_strings(lowbatt_labels, lowbatt_labels_size);
    print_osd_adjustable_float(3, 1, low_batt_ptr, lowbatt_grid, lowbatt_data_positions, 1);
    if (osd_state.screen_phase == 5)
      osd_float_adjust(low_batt_ptr, 1, 1, lowbatt_adjust_limits, 0.1);
    break;

  case OSD_SCREEN_LEVEL_MODE:
    print_osd_menu_strings(level_submenu_labels, level_submenu_labels_size);
    if (osd_state.screen_phase == 4)
      osd_select_menu_item(2, level_submenu_map, SUB_MENU);
    break;

  case OSD_SCREEN_MOTOR_BOOST:
    print_osd_menu_strings(motor_boost_labels, motor_boost_labels_size);
    if (osd_state.screen_phase == 4)
      osd_select_menu_item(2, motor_boost_map, SUB_MENU);
    break;

  case OSD_SCREEN_DIGITAL_IDLE:
    print_osd_menu_strings(motoridle_labels, motoridle_labels_size);
    print_osd_adjustable_float(3, 1, motoridle_ptr, motoridle_grid, motoridle_data_positions, 1);
    if (osd_state.screen_phase == 5)
      osd_float_adjust(motoridle_ptr, 1, 1, motoridle_adjust_limits, 0.1);
    break;

  case OSD_SCREEN_LEVEL_MAX_ANGLE:
    print_osd_menu_strings(maxangle_labels, maxangle_labels_size);
    print_osd_adjustable_float(3, 1, level_maxangle_ptr, maxangle_grid, maxangle_data_positions, 0);
    if (osd_state.screen_phase == 5)
      osd_float_adjust(level_maxangle_ptr, 1, 1, maxangle_adjust_limits, 1.0);
    break;

  case OSD_SCREEN_LEVEL_STRENGTH:
    print_osd_menu_strings(levelmode_labels, levelmode_labels_size);
    print_osd_adjustable_float(6, 4, level_pid_ptr, levelmode_grid, levelmode_data_positions, 1);
    if (osd_state.screen_phase == 11)
      osd_float_adjust(level_pid_ptr, 2, 2, levelmode_adjust_limits, 0.5);
    break;

  case OSD_SCREEN_TORQUE_BOOST:
    print_osd_menu_strings(torqueboost_labels, torqueboost_labels_size);
    print_osd_adjustable_float(3, 1, torqueboost_ptr, torqueboost_grid, torqueboost_data_positions, 1);
    if (osd_state.screen_phase == 5)
      osd_float_adjust(torqueboost_ptr, 1, 1, torqueboost_adjust_limits, 0.1);
    break;

  case OSD_SCREEN_THROTTLE_BOOST:
    print_osd_menu_strings(throttleboost_labels, throttleboost_labels_size);
    print_osd_adjustable_float(3, 1, throttleboost_ptr, throttleboost_grid, throttleboost_data_positions, 1);
    if (osd_state.screen_phase == 5)
      osd_float_adjust(throttleboost_ptr, 1, 1, throttleboost_adjust_limits, 0.5);
    break;

  case OSD_SCREEN_TURTLE_THROTTLE:
    print_osd_menu_strings(turtlethrottle_labels, turtlethrottle_labels_size);
    print_osd_adjustable_float(3, 1, turtlethrottle_ptr, turtlethrottle_grid, turtlethrottle_data_positions, 0);
    if (osd_state.screen_phase == 5)
      osd_float_adjust(turtlethrottle_ptr, 1, 1, turtlethrottle_adjust_limits, 10.0);
    break;

  case OSD_SCREEN_PID_MODIFIER:
    osd_menu_start();
    osd_menu_header("PID MODIFIERS");

    const char *modifier_state_labels[] = {
        " NONE ",
        "ACTIVE",
    };

    osd_menu_select(2, 4, "P-TERM VOLTAGE COMP");
    if (osd_menu_select_enum(23, 4, profile.voltage.pid_voltage_compensation, modifier_state_labels)) {
      profile.voltage.pid_voltage_compensation = osd_menu_adjust_int(profile.voltage.pid_voltage_compensation, 1, 0, 1);
    }

    osd_menu_select(2, 5, "THROTTLE D ATTENUATE");
    if (osd_menu_select_enum(23, 5, profile.pid.throttle_dterm_attenuation.tda_active, modifier_state_labels)) {
      profile.pid.throttle_dterm_attenuation.tda_active = osd_menu_adjust_int(profile.pid.throttle_dterm_attenuation.tda_active, 1, 0, 1);
    }

    osd_menu_select(2, 6, "TDA BREAKPOINT");
    if (osd_menu_select_float(23, 6, profile.pid.throttle_dterm_attenuation.tda_breakpoint, 4, 1)) {
      profile.pid.throttle_dterm_attenuation.tda_breakpoint = osd_menu_adjust_float(profile.pid.throttle_dterm_attenuation.tda_breakpoint, 0.05, 0, 1);
    }

    osd_menu_select(2, 7, "TDA PERCENT");
    if (osd_menu_select_float(23, 7, profile.pid.throttle_dterm_attenuation.tda_percent, 4, 1)) {
      profile.pid.throttle_dterm_attenuation.tda_percent = osd_menu_adjust_float(profile.pid.throttle_dterm_attenuation.tda_percent, 0.05, 0, 1);
    }

    osd_menu_select_save_and_exit(2, 14);
    osd_menu_finish();
    break;

  case OSD_SCREEN_RC_LINK:
    print_osd_menu_strings(rc_link_labels, rc_link_labels_size);
    if (osd_state.screen_phase == 4)
      osd_select_menu_item(7, rc_link_map, SUB_MENU);
    break;

  case OSD_SCREEN_RSSI:
    print_osd_menu_strings(rssi_menu_labels, rssi_menu_labels_size);
    print_osd_adjustable_enums(4, 2, get_rssi_source_status(osd_state.screen_phase - 5), rssi_source_data_grid, rssi_source_data_positions);
    if (osd_state.screen_phase == 7)
      osd_enum_adjust(rssi_source_ptr, 2, rssi_source_limits);
    break;

  case OSD_SCREEN_STICK_WIZARD: // stick wizard select menu
    print_osd_menu_strings(stick_wizard_labels_1, stick_wizard_labels_1_size);
    if (osd_state.screen_phase == 4) {
      if (osd_state.selection) {
        request_stick_calibration_wizard();
        osd_push_screen(OSD_SCREEN_STICK_WIZARD_CALIBRATION);
        osd_state.selection = 0;
      }
    }
    break;

  case OSD_SCREEN_STICK_WIZARD_CALIBRATION: // 5 sec to calibrate
    print_osd_menu_strings(stick_wizard_labels_2, stick_wizard_labels_2_size);
    if (osd_state.screen_phase == 4) {
      if (state.stick_calibration_wizard == WAIT_FOR_CONFIRM) {
        osd_push_screen(OSD_SCREEN_STICK_CONFIRM);
      }
    }
    break;

  case OSD_SCREEN_STICK_CONFIRM: // 5 sec to test / confirm calibration
    print_osd_menu_strings(stick_wizard_labels_3, stick_wizard_labels_3_size);
    if (osd_state.screen_phase == 4) {
      if ((state.stick_calibration_wizard == CALIBRATION_SUCCESS) || (state.stick_calibration_wizard == TIMEOUT)) {
        osd_push_screen(OSD_SCREEN_STICK_RESULT);
      }
    }
    break;

  case OSD_SCREEN_STICK_RESULT: // results of calibration
    if (state.stick_calibration_wizard == CALIBRATION_SUCCESS) {
      print_osd_menu_strings(stick_wizard_labels_4, stick_wizard_labels_4_size); // osd_state.screen_phase will be 4 after this
    }
    if (state.stick_calibration_wizard == TIMEOUT) {
      print_osd_menu_strings(stick_wizard_labels_5, stick_wizard_labels_5_size); // osd_state.screen_phase will be 4 after this
    }
    if (osd_state.selection > 0)
      osd_state.selection = 0;
    break;
  }

  if (osd_state.screen != OSD_SCREEN_REGULAR && rx_aux_on(AUX_ARMING)) {
    flags.arm_safety = 1;
  }
}

#endif
