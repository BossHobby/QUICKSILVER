#include "osd/render.h"

#include <float.h>
#include <stdio.h>
#include <string.h>

#include "core/debug.h"
#include "core/flash.h"
#include "core/looptime.h"
#include "core/profile.h"
#include "core/project.h"
#include "driver/osd.h"
#include "driver/time.h"
#include "flight/control.h"
#include "flight/filter.h"
#include "io/blackbox_device.h"
#include "io/vtx.h"
#include "osd/menu.h"
#include "rx/rx.h"
#include "util/util.h"

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
#define ICON_DOWN 0x76

#define HOLD 0
#define TEMP 1

#define HD_ROWS 18
#define HD_COLS 50

extern profile_t profile;

static vtx_settings_t vtx_settings_copy;
static uint8_t vtx_buffer_populated = 0;

osd_system_t osd_system = OSD_SYS_NONE;

osd_state_t osd_state = {
    .element = OSD_CALLSIGN,

    .screen = OSD_SCREEN_REGULAR,
    .screen_history_size = 0,
    .screen_phase = 0,

    .cursor = 0,
    .cursor_history_size = 0,

    .selection = 0,
    .selection_max = 1,
    .selection_increase = 0,
    .selection_decrease = 0,

    .reboot_fc_requested = 0,
};

static const char *osd_element_labels[] = {
    "CALLSIGN",
    "CELL COUNT",
    "FUELGAUGE VOLTS",
    "FILTERED VOLTS",
    "GYRO TEMP",
    "FLIGHT MODE",
    "RSSI",
    "STOPWATCH",
    "SYSTEM STATUS",
    "THROTTLE",
    "VTX",
    "CURRENT DRAW",
};

static const char *aux_channel_labels[] = {
    "CHANNEL 5  ",
    "CHANNEL 6  ",
    "CHANNEL 7  ",
    "CHANNEL 8  ",
    "CHANNEL 9  ",
    "CHANNEL 10 ",
    "CHANNEL 11 ",
    "CHANNEL 12 ",
    "CHANNEL 13 ",
    "CHANNEL 14 ",
    "CHANNEL 15 ",
    "CHANNEL 16 ",
    "ALWAYS OFF ",
    "ALWAYS ON  ",
    "GESTURE AUX",
};

#pragma GCC diagnostic ignored "-Wmissing-braces"

static const vec3_t rate_defaults[RATE_MODE_ACTUAL + 1][3] = {
    {
        {MAX_RATE, MAX_RATE, MAX_RATEYAW},
        {ACRO_EXPO_ROLL, ACRO_EXPO_PITCH, ACRO_EXPO_YAW},
        {ANGLE_EXPO_ROLL, ANGLE_EXPO_PITCH, ANGLE_EXPO_YAW},
    },
    {
        {BF_RC_RATE_ROLL, BF_RC_RATE_PITCH, BF_RC_RATE_YAW},
        {BF_SUPER_RATE_ROLL, BF_SUPER_RATE_PITCH, BF_SUPER_RATE_YAW},
        {BF_EXPO_ROLL, BF_EXPO_PITCH, BF_EXPO_YAW},
    },
    {
        {ACTUAL_CENTER_SENS_ROLL, ACTUAL_CENTER_SENS_PITCH, ACTUAL_CENTER_SENS_YAW},
        {ACTUAL_MAX_RATE_ROLL, ACTUAL_MAX_RATE_PITCH, ACTUAL_MAX_RATE_YAW},
        {ACTUAL_EXPO_ROLL, ACTUAL_EXPO_PITCH, ACTUAL_EXPO_YAW},
    },
};

#pragma GCC diagnostic pop

static uint8_t osd_attr(osd_element_t *el) {
  return el->attribute ? OSD_ATTR_INVERT : OSD_ATTR_TEXT;
}

static uint32_t *osd_elements() {
  if (osd_system == OSD_SYS_HD) {
    return profile.osd.elements_hd;
  }
  return profile.osd.elements;
}

void osd_display_reset() {
  osd_state.element = OSD_CALLSIGN;

  osd_state.screen = OSD_SCREEN_REGULAR;
  osd_state.screen_phase = OSD_PHASE_CLEAR;
  osd_state.screen_history_size = 0;

  osd_state.cursor = 1;
  osd_state.cursor_history_size = 0;

  osd_state.selection = 0;
  osd_state.selection_max = 1;
}

static void osd_update_screen(osd_screens_t screen) {
  osd_state.screen = screen;
  osd_state.screen_phase = OSD_PHASE_CLEAR;
}

osd_screens_t osd_push_screen_replace(osd_screens_t screen) {
  osd_state.selection = 0;
  osd_state.selection_max = 1;

  osd_update_screen(screen);
  osd_state.cursor = 1;

  return screen;
}

osd_screens_t osd_push_screen(osd_screens_t screen) {
  osd_state.cursor_history[osd_state.cursor_history_size] = osd_state.cursor;
  osd_state.cursor_history_size++;

  osd_state.screen_history[osd_state.screen_history_size] = osd_state.screen;
  osd_state.screen_history_size++;

  return osd_push_screen_replace(screen);
}

osd_screens_t osd_pop_screen() {
  if (osd_state.cursor_history_size) {
    osd_state.cursor = osd_state.cursor_history[osd_state.cursor_history_size - 1];
    osd_state.cursor_history_size--;
  }

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
      osd_state.screen_phase = OSD_PHASE_REFRESH;
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
      osd_state.screen_phase = OSD_PHASE_REFRESH;
    }
    break;

  case OSD_INPUT_LEFT:
    if (osd_state.selection) {
      osd_state.selection--;
      osd_state.screen_phase = OSD_PHASE_REFRESH;
    } else {
      osd_pop_screen();
    }
    break;

  case OSD_INPUT_RIGHT:
    osd_state.selection++;
    if (osd_state.selection > osd_state.selection_max) {
      osd_state.selection = osd_state.selection_max;
    }
    osd_state.screen_phase = OSD_PHASE_REFRESH;
    break;
  }
}

void osd_exit() {
  osd_state.selection = 0;
  osd_state.cursor = 1;
  osd_state.cursor_history_size = 0;
  osd_state.screen = OSD_SCREEN_CLEAR;
}

void osd_save_exit() {
  osd_exit();

  // check if vtx settings need to be updated
  if (vtx_buffer_populated) {
    vtx_set(&vtx_settings_copy);
  }

  // check for fc reboot request
  extern int pid_gestures_used;
  extern int ledcommand;

  pid_gestures_used = 0;
  ledcommand = 1;

  flash_save();
  flash_load();

  // reset flash numbers for pids
  extern int number_of_increments[3][3];
  for (int i = 0; i < 3; i++)
    for (int j = 0; j < 3; j++)
      number_of_increments[i][j] = 0;

  // reset loop time - maybe not necessary cause it gets reset in the next screen clear
  looptime_reset();

  if (osd_state.reboot_fc_requested)
    NVIC_SystemReset();
}

static void print_osd_flightmode(osd_element_t *el) {
  const uint8_t flightmode_labels[5][10] = {
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

  osd_start(osd_attr(el), el->pos_x, el->pos_y);
  osd_write_data(flightmode_labels[flightmode], 10);
}

static void print_osd_rssi(osd_element_t *el) {
  static float rx_rssi_filt;
  if (flags.failsafe)
    state.rx_rssi = 0.0f;

  lpf(&rx_rssi_filt, state.rx_rssi, FILTERCALC(state.looptime * 1e6f * 133.0f, 2e6f)); // 2 second filtertime and 15hz refresh rate @4k, 30hz@ 8k loop

  osd_start(osd_attr(el), el->pos_x, el->pos_y);
  osd_write_uint(rx_rssi_filt - 0.5f, 4);
  osd_write_char(ICON_RSSI);
}

static void print_osd_armtime(osd_element_t *el) {
  uint32_t time_s = state.armtime;

  // Will only display up to 59:59 as realistically no quad will fly that long (currently).
  // Reset to zero at on reaching 1 hr
  while (time_s >= 3600) {
    time_s -= 3600;
  }

  osd_start(osd_attr(el), el->pos_x, el->pos_y);

  const uint32_t minutes = time_s / 60;
  osd_write_uint(minutes / 10, 1);
  osd_write_uint(minutes % 10, 1);

  osd_write_char(':');

  const uint32_t seconds = time_s % 60;
  osd_write_uint(seconds / 10, 1);
  osd_write_uint(seconds % 10, 1);
}

// print the current vtx settings as Band:Channel:Power
static void print_osd_vtx(osd_element_t *el) {
  osd_start(osd_attr(el), el->pos_x, el->pos_y);

  switch (vtx_settings.band) {
  case VTX_BAND_A:
    osd_write_char('A');
    break;
  case VTX_BAND_B:
    osd_write_char('B');
    break;
  case VTX_BAND_E:
    osd_write_char('E');
    break;
  case VTX_BAND_F:
    osd_write_char('F');
    break;
  case VTX_BAND_R:
    osd_write_char('R');
    break;
  case VTX_BAND_L:
    osd_write_char('L');
    break;
  default:
    osd_write_char('M');
    break;
  }

  osd_write_char(':');
  osd_write_char(vtx_settings.channel + 49);
  osd_write_char(':');

  if (vtx_settings.pit_mode == 1) {
    osd_write_char(21); // "pit", probably from Pitch, but we will use it here
  } else {
    if (vtx_settings.power_level == 4)
      osd_write_char(36); // "max"
    else
      osd_write_char(vtx_settings.power_level + 49);
  }
}

// 3 stage return - 0 = stick and hold, 1 = move on but come back to clear, 2 = status print done
static uint8_t print_status(osd_element_t *el, uint8_t persistence, uint8_t label) {
  const char *default_system_status_labels[12] = {
      "                     ",
      " **DISARMED**        ",
      "  **ARMED**          ",
      " STICK BOOST 1       ",
      " STICK BOOST 2       ",
      " **FAILSAFE**        ",
      "THROTTLE SAFETY      ",
      " ARMING SAFETY       ",
      "**LOW BATTERY**      ",
      "**MOTOR TEST**       ",
      "  **TURTLE**         ",
      " **LOOPTIME**        ",
  };
  const char *guac_system_status_labels[12] = {
      "                     ",
      "    **GAME OVER**    ",
      " **HERE WE GO AGAIN**",
      "     STICK BOOST 1   ",
      "     STICK BOOST 2   ",
      " **404 RX NOT FOUND**",
      "    **DANGER ZONE**  ",
      "   **ARMING SAFETY** ",
      "CONSTRUCT MORE PYLONS",
      "    **MOTOR TEST**   ",
      "    \x60THIS SIDE UP\x60   ",
      "     **LOOPTIME**    ",
  };

  const char **labels = profile.osd.guac_mode ? guac_system_status_labels : default_system_status_labels;

  static uint8_t last_label;
  static uint8_t delay_counter = 25;
  if (last_label != label) { // critical to reset indexers if a print sequence is interrupted by a new request
    delay_counter = 25;
  }
  last_label = label;

  if (delay_counter == 25) {
    // First run, print the label
    osd_start(osd_attr(el) | OSD_ATTR_BLINK, el->pos_x, el->pos_y);
    osd_write_data((uint8_t *)labels[label], 21);

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
    osd_start(osd_attr(el), el->pos_x, el->pos_y);
    osd_write_data((uint8_t *)labels[0], 21);

    delay_counter = 25;

    return 2;
  }

  delay_counter--;
  return 1;
}

static uint8_t print_osd_system_status(osd_element_t *el) {
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

void osd_init() {
  osd_device_init();
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
    osd_start(osd_attr(el), el->pos_x, el->pos_y);
    osd_write_str((const char *)profile.osd.callsign);

    osd_state.element++;
    break;
  }

  case OSD_CELL_COUNT: {
    if (!flags.lowbatt) {
      osd_start(osd_attr(el), el->pos_x, el->pos_y);
    } else {
      osd_start(OSD_ATTR_BLINK | OSD_ATTR_INVERT, el->pos_x, el->pos_y);
    }
    osd_write_uint(state.lipo_cell_count, 1);
    osd_write_char('S');

    osd_state.element++;
    break;
  }

  case OSD_FUELGAUGE_VOLTS: {
    osd_start(osd_attr(el), el->pos_x, el->pos_y);
    osd_write_float(state.vbat_compensated_cell_avg, 4, 1);
    osd_write_char('V');

    osd_state.element++;
    break;
  }

  case OSD_FILTERED_VOLTS: {
    osd_start(osd_attr(el), el->pos_x, el->pos_y);
    osd_write_float(state.vbat_cell_avg, 4, 1);
    osd_write_char('V');

    osd_state.element++;
    break;
  }

  case OSD_GYRO_TEMP: {
    osd_start(osd_attr(el), el->pos_x, el->pos_y);
    osd_write_int(state.gyro_temp, 4);
    osd_write_char(ICON_CELSIUS);

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
    osd_start(osd_attr(el), el->pos_x, el->pos_y);
    const float throttle = state.rx_filtered.throttle * 100.0f;
    if (profile.osd.guac_mode && throttle > 99.0f) {
      osd_write_str("GUAC");
    } else {
      osd_write_uint(throttle, 4);
    }
    osd_write_char(ICON_THROTTLE);

    osd_state.element++;
    break;
  }

  case OSD_VTX_CHANNEL: {
    print_osd_vtx(el);
    osd_state.element++;
    break;
  }

  case OSD_CURRENT_DRAW: {
    osd_start(osd_attr(el), el->pos_x, el->pos_y);
    osd_write_float(state.ibat_filtered / 1000.0f, 4, 2);
    osd_write_char(ICON_AMP);

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
    static vec3_t rate_backup[RATE_MODE_ACTUAL + 1][3];
    static bool rate_backup_set[RATE_MODE_ACTUAL + 1];

    const uint8_t current_mode = profile_current_rates()->mode;
    const uint8_t new_mode = osd_menu_adjust_int(current_mode, 1, RATE_MODE_SILVERWARE, RATE_MODE_ACTUAL);

    memcpy(rate_backup[current_mode], profile_current_rates()->rate, 3 * sizeof(vec3_t));
    rate_backup_set[current_mode] = true;

    if (rate_backup_set[new_mode]) {
      memcpy(profile_current_rates()->rate, rate_backup[new_mode], 3 * sizeof(vec3_t));
    } else {
      memcpy(profile_current_rates()->rate, rate_defaults[new_mode], 3 * sizeof(vec3_t));
    }

    profile_current_rates()->mode = new_mode;
    osd_state.screen_phase = OSD_PHASE_CLEAR;
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
    if (osd_menu_select_vec3(13, 6, rate[ACTUAL_CENTER_SENSITIVITY], 5, 0)) {
      rate[ACTUAL_CENTER_SENSITIVITY] = osd_menu_adjust_vec3(rate[ACTUAL_CENTER_SENSITIVITY], 10, 0.0, 500);
    }

    osd_menu_select(2, 7, "MAX RATE");
    if (osd_menu_select_vec3(13, 7, rate[ACTUAL_MAX_RATE], 5, 0)) {
      rate[ACTUAL_MAX_RATE] = osd_menu_adjust_vec3(rate[ACTUAL_MAX_RATE], 10, 0.0, 1800);
    }

    osd_menu_select(2, 8, "EXPO");
    if (osd_menu_select_vec3(13, 8, rate[ACTUAL_EXPO], 5, 2)) {
      rate[ACTUAL_EXPO] = osd_menu_adjust_vec3(rate[ACTUAL_EXPO], 0.01, 0.0, 0.99);
    }
    break;
  }
  }

  osd_menu_select_save_and_exit(2);
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

  if (!osd_update()) {
    return;
  }

  static bool did_just_arm = false;
  if (flags.arm_switch) {
    if (!did_just_arm) {
      while (osd_pop_screen() != OSD_SCREEN_CLEAR)
        ;
      did_just_arm = true;
      return;
    }
  } else {
    did_just_arm = false;
  }

  switch (osd_state.screen) {
  case OSD_SCREEN_CLEAR:
    if (osd_clear_async())
      osd_display_reset();
    break;

  case OSD_SCREEN_REGULAR:
    osd_display_regular();
    break;

  case OSD_SCREEN_MAIN_MENU: {
    osd_menu_start();
    osd_menu_header("MENU");

    osd_menu_scroll_start(7, 2, 7);
    {
      // PAGE 1
      osd_menu_select_screen(7, OSD_AUTO, "VTX", OSD_SCREEN_VTX);
      osd_menu_select_screen(7, OSD_AUTO, "PIDS", OSD_SCREEN_PID_PROFILE);
      osd_menu_select_screen(7, OSD_AUTO, "FILTERS", OSD_SCREEN_FILTERS);
      osd_menu_select_screen(7, OSD_AUTO, "RATES", OSD_SCREEN_RATE_PROFILES);
      osd_menu_select_screen(7, OSD_AUTO, "FLIGHT MODES", OSD_SCREEN_FLIGHT_MODES);
      osd_menu_select_screen(7, OSD_AUTO, "RC LINK", OSD_SCREEN_RC_LINK);
      osd_menu_select_screen(7, OSD_AUTO, "OSD ELEMENTS", OSD_SCREEN_ELEMENTS);

      // PAGE 2
      osd_menu_select_screen(7, OSD_AUTO, "PID MODIFIERS", OSD_SCREEN_PID_MODIFIER);
      osd_menu_select_screen(7, OSD_AUTO, "LEVEL MODE", OSD_SCREEN_LEVEL_MODE);
      osd_menu_select_screen(7, OSD_AUTO, "MOTOR SETTINGS", OSD_SCREEN_MOTOR_SETTINGS);
      osd_menu_select_screen(7, OSD_AUTO, "THROTTLE SETTINGS", OSD_SCREEN_THROTTLE_SETTINGS);
      osd_menu_select_screen(7, OSD_AUTO, "SPECIAL FEATURES", OSD_SCREEN_SPECIAL_FEATURES);
      osd_menu_select_screen(7, OSD_AUTO, "BLACKBOX", OSD_SCREEN_BLACKBOX);
    }
    osd_menu_scroll_finish(7);

    osd_menu_select_save_and_exit(7);
    osd_menu_finish();
    break;
  }

  case OSD_SCREEN_PID_PROFILE:
    osd_menu_start();
    osd_menu_header("PID PROFILES");

    if (profile.pid.pid_profile == PID_PROFILE_1)
      osd_menu_label(7, 3, "ACTIVE: PROFILE 1");
    else
      osd_menu_label(7, 3, "ACTIVE: PROFILE 2");

    if (osd_menu_button(7, 5, "PID PROFILE 1")) {
      profile.pid.pid_profile = PID_PROFILE_1;
      osd_push_screen(OSD_SCREEN_PID);
    }

    if (osd_menu_button(7, 6, "PID PROFILE 2")) {
      profile.pid.pid_profile = PID_PROFILE_2;
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

    osd_menu_select_save_and_exit(7);
    osd_menu_finish();
    break;

  case OSD_SCREEN_RATE_PROFILES:
    osd_menu_start();
    osd_menu_header("RATE PROFILES");

    if (profile.rate.profile == STICK_RATE_PROFILE_1)
      osd_menu_label(7, 3, "ACTIVE: PROFILE 1");
    else
      osd_menu_label(7, 3, "ACTIVE: PROFILE 2");

    if (osd_menu_button(7, 5, "PROFILE 1")) {
      profile.rate.profile = STICK_RATE_PROFILE_1;
      osd_push_screen(OSD_SCREEN_RATES);
    }

    if (osd_menu_button(7, 6, "PROFILE 2")) {
      profile.rate.profile = STICK_RATE_PROFILE_2;
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
        " PT3",
    };

    osd_menu_select(4, 4, "PASS 1 TYPE");
    if (osd_menu_select_enum(18, 4, profile.filter.gyro[0].type, filter_type_labels)) {
      profile.filter.gyro[0].type = osd_menu_adjust_int(profile.filter.gyro[0].type, 1, 0, FILTER_LP_PT3);
      osd_state.reboot_fc_requested = 1;
    }

    osd_menu_select(4, 5, "PASS 1 FREQ");
    if (osd_menu_select_float(18, 5, profile.filter.gyro[0].cutoff_freq, 4, 0)) {
      profile.filter.gyro[0].cutoff_freq = osd_menu_adjust_float(profile.filter.gyro[0].cutoff_freq, 10, 50, 500);
    }

    osd_menu_select(4, 6, "PASS 2 TYPE");
    if (osd_menu_select_enum(18, 6, profile.filter.gyro[1].type, filter_type_labels)) {
      profile.filter.gyro[1].type = osd_menu_adjust_int(profile.filter.gyro[1].type, 1, 0, FILTER_LP_PT3);
      osd_state.reboot_fc_requested = 1;
    }

    osd_menu_select(4, 7, "PASS 2 FREQ");
    if (osd_menu_select_float(18, 7, profile.filter.gyro[1].cutoff_freq, 4, 0)) {
      profile.filter.gyro[1].cutoff_freq = osd_menu_adjust_float(profile.filter.gyro[1].cutoff_freq, 10, 50, 500);
    }

    osd_menu_select_save_and_exit(4);
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
      profile.filter.dterm[0].type = osd_menu_adjust_int(profile.filter.dterm[0].type, 1, 0, FILTER_LP_PT2);
      osd_state.reboot_fc_requested = 1;
    }

    osd_menu_select(4, 4, "PASS 1 FREQ");
    if (osd_menu_select_float(18, 4, profile.filter.dterm[0].cutoff_freq, 4, 0)) {
      profile.filter.dterm[0].cutoff_freq = osd_menu_adjust_float(profile.filter.dterm[0].cutoff_freq, 10, 50, 500);
    }

    osd_menu_select(4, 5, "PASS 2 TYPE");
    if (osd_menu_select_enum(18, 5, profile.filter.dterm[1].type, filter_type_labels)) {
      profile.filter.dterm[1].type = osd_menu_adjust_int(profile.filter.dterm[1].type, 1, 0, FILTER_LP_PT2);
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

    osd_menu_select_save_and_exit(4);
    osd_menu_finish();
    break;

  case OSD_SCREEN_FLIGHT_MODES: {
    osd_menu_start();
    osd_menu_header("FLIGHT MODES");

    osd_menu_scroll_start(4, 2, 7);
    {
      // PAGE 1
      osd_menu_select_enum_adjust(4, OSD_AUTO, "ARMING", 17, &profile.receiver.aux[AUX_ARMING], aux_channel_labels, AUX_CHANNEL_0, AUX_CHANNEL_11);
      osd_menu_select_enum_adjust(4, OSD_AUTO, "PREARM", 17, &profile.receiver.aux[AUX_PREARM], aux_channel_labels, AUX_CHANNEL_0, AUX_CHANNEL_GESTURE);
      osd_menu_select_enum_adjust(4, OSD_AUTO, "IDLE UP", 17, &profile.receiver.aux[AUX_IDLE_UP], aux_channel_labels, AUX_CHANNEL_0, AUX_CHANNEL_GESTURE);
      osd_menu_select_enum_adjust(4, OSD_AUTO, "LEVELMODE", 17, &profile.receiver.aux[AUX_LEVELMODE], aux_channel_labels, AUX_CHANNEL_0, AUX_CHANNEL_GESTURE);
      osd_menu_select_enum_adjust(4, OSD_AUTO, "RACEMODE", 17, &profile.receiver.aux[AUX_RACEMODE], aux_channel_labels, AUX_CHANNEL_0, AUX_CHANNEL_GESTURE);
      osd_menu_select_enum_adjust(4, OSD_AUTO, "HORIZON", 17, &profile.receiver.aux[AUX_HORIZON], aux_channel_labels, AUX_CHANNEL_0, AUX_CHANNEL_GESTURE);
      osd_menu_select_enum_adjust(4, OSD_AUTO, "STICK BOOST", 17, &profile.receiver.aux[AUX_STICK_BOOST_PROFILE], aux_channel_labels, AUX_CHANNEL_0, AUX_CHANNEL_GESTURE);

      // PAGE 2
      osd_menu_select_enum_adjust(4, OSD_AUTO, "RATE", 17, &profile.receiver.aux[AUX_RATE_PROFILE], aux_channel_labels, AUX_CHANNEL_0, AUX_CHANNEL_GESTURE);
      osd_menu_select_enum_adjust(4, OSD_AUTO, "BUZZER", 17, &profile.receiver.aux[AUX_BUZZER_ENABLE], aux_channel_labels, AUX_CHANNEL_0, AUX_CHANNEL_GESTURE);
      osd_menu_select_enum_adjust(4, OSD_AUTO, "TURTLE", 17, &profile.receiver.aux[AUX_TURTLE], aux_channel_labels, AUX_CHANNEL_0, AUX_CHANNEL_GESTURE);
      osd_menu_select_enum_adjust(4, OSD_AUTO, "MOTOR TEST", 17, &profile.receiver.aux[AUX_MOTOR_TEST], aux_channel_labels, AUX_CHANNEL_0, AUX_CHANNEL_GESTURE);
      osd_menu_select_enum_adjust(4, OSD_AUTO, "FPV SWITCH", 17, &profile.receiver.aux[AUX_FPV_SWITCH], aux_channel_labels, AUX_CHANNEL_0, AUX_CHANNEL_GESTURE);
      osd_menu_select_enum_adjust(4, OSD_AUTO, "BLACKBOX", 17, &profile.receiver.aux[AUX_BLACKBOX], aux_channel_labels, AUX_CHANNEL_0, AUX_CHANNEL_GESTURE);
    }
    osd_menu_scroll_finish(4);

    osd_menu_select_save_and_exit(4);
    osd_menu_finish();
    break;
  }

  case OSD_SCREEN_VTX:
    if (vtx_settings.detected) {
      static char power_level_labels_terminated[VTX_POWER_LEVEL_MAX][4];
      if (!vtx_buffer_populated) {
        vtx_settings_copy = vtx_settings;
        for (uint8_t i = 0; i < VTX_POWER_LEVEL_MAX; i++) {
          memcpy(power_level_labels_terminated[i], vtx_settings.power_table.labels[i], 3);
          power_level_labels_terminated[i][3] = 0;
        }
        vtx_buffer_populated = 1;
      }

      osd_menu_start();
      osd_menu_header("VTX CONTROLS");

      const char *band_labels[] = {"A", "B", "E", "F", "R", "L"};
      osd_menu_select_enum_adjust(4, 4, "BAND", 20, &vtx_settings_copy.band, band_labels, VTX_BAND_A, VTX_BAND_MAX - 1);

      const char *channel_labels[] = {"1", "2", "3", "4", "5", "6", "7", "8"};
      osd_menu_select_enum_adjust(4, 5, "CHANNEL", 20, &vtx_settings_copy.channel, channel_labels, VTX_CHANNEL_1, VTX_CHANNEL_8);

      // this ugly AF
      const char *power_level_labels[] = {
          power_level_labels_terminated[0],
          power_level_labels_terminated[1],
          power_level_labels_terminated[2],
          power_level_labels_terminated[3],
          power_level_labels_terminated[4],
      };
      osd_menu_select_enum_adjust(4, 6, "POWER LEVEL", 20, &vtx_settings_copy.power_level, power_level_labels, VTX_POWER_LEVEL_1, VTX_POWER_LEVEL_MAX - 1);

      const char *pit_mode_labels[] = {"OFF", "ON ", "N/A"};
      osd_menu_select(4, 7, "PITMODE");
      if (osd_menu_select_enum(20, 7, vtx_settings_copy.pit_mode, pit_mode_labels) && vtx_settings_copy.pit_mode != VTX_PIT_MODE_NO_SUPPORT) {
        vtx_settings_copy.pit_mode = osd_menu_adjust_int(vtx_settings_copy.pit_mode, 1, VTX_PIT_MODE_OFF, VTX_PIT_MODE_ON);
      }

      osd_menu_select_save_and_exit(4);
      osd_menu_finish();
    } else {
      osd_menu_start();
      osd_menu_header("VTX CONTROLS");

      osd_menu_label(7, 4, "SMART AUDIO");
      osd_menu_label(7, 5, "NOT CONFIGURED");

      osd_menu_finish();

      if (osd_state.selection) {
        osd_state.selection = 0;
      }
    }
    break;

  case OSD_SCREEN_SPECIAL_FEATURES:
    osd_menu_start();
    osd_menu_header("SPECIAL FEATURES");

    osd_menu_select_screen(7, 3, "STICK BOOST", OSD_SCREEN_STICK_BOOST);
    osd_menu_select_screen(7, 4, "MOTOR BOOST", OSD_SCREEN_MOTOR_BOOST);
    osd_menu_select_screen(7, 5, "LOW BATTERY", OSD_SCREEN_LOWBAT);

    osd_menu_finish();
    break;

  case OSD_SCREEN_STICK_BOOST:
    osd_menu_start();
    osd_menu_header("STICK BOOST PROFILES");

    if (osd_menu_button(7, 4, "AUX OFF PROFILE 1")) {
      profile.pid.stick_profile = STICK_PROFILE_OFF;
      osd_push_screen(OSD_SCREEN_STICK_BOOST_ADJUST);
    }

    if (osd_menu_button(7, 5, "AUX ON  PROFILE 2")) {
      profile.pid.stick_profile = STICK_PROFILE_ON;
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

    osd_menu_select_save_and_exit(7);
    osd_menu_finish();
    break;
  }

  case OSD_SCREEN_ELEMENTS:
    osd_menu_start();
    osd_menu_header("OSD ELEMENTS");

    osd_menu_select_screen(7, 4, "ADD OR REMOVE", OSD_SCREEN_ELEMENTS_ADD_REMOVE);
    osd_menu_select_screen(7, 5, "EDIT POSITIONS", OSD_SCREEN_ELEMENTS_POSITION);
    osd_menu_select_screen(7, 6, "EDIT TEXT STYLE", OSD_SCREEN_ELEMENTS_STYLE);
    osd_menu_select_screen(7, 7, "EDIT CALLSIGN", OSD_SCREEN_CALLSIGN);

    osd_menu_finish();
    break;

  case OSD_SCREEN_ELEMENTS_ADD_REMOVE: {
    osd_menu_start();
    osd_menu_header("OSD DISPLAY ITEMS");

    const char *active_labels[] = {
        "INACTIVE",
        "ACTIVE  ",
    };

    osd_menu_scroll_start(4, 2, 7);
    for (uint8_t i = 0; i < OSD_ELEMENT_MAX; i++) {
      osd_element_t *el = (osd_element_t *)(osd_elements() + i);

      osd_menu_select(4, OSD_AUTO, osd_element_labels[i]);
      if (osd_menu_select_enum(20, OSD_AUTO, el->active, active_labels)) {
        el->active = osd_menu_adjust_int(el->active, 1, 0, 1);
      }
    }
    osd_menu_scroll_finish(4);

    osd_menu_select_save_and_exit(4);
    osd_menu_finish();
    break;
  }

  case OSD_SCREEN_ELEMENTS_POSITION:
    osd_menu_start();

    osd_menu_highlight(1, 1, "OSD POSITIONS");
    osd_menu_label(18, 1, "ADJ X");
    osd_menu_label(24, 1, "ADJ Y");

    osd_menu_scroll_start(3, 2, 7);
    for (uint8_t i = 0; i < OSD_ELEMENT_MAX; i++) {
      osd_element_t *el = (osd_element_t *)(osd_elements() + i);

      osd_menu_select(3, OSD_AUTO, osd_element_labels[i]);
      if (osd_menu_select_int(20, OSD_AUTO, el->pos_x, 3)) {
        el->pos_x = osd_menu_adjust_int(el->pos_x, 1, 0, osd_system == OSD_SYS_HD ? HD_COLS : 30);
      }
      if (osd_menu_select_int(26, OSD_AUTO, el->pos_y, 3)) {
        el->pos_y = osd_menu_adjust_int(el->pos_y, 1, 0, osd_system == OSD_SYS_HD ? HD_ROWS : 15);
      }
    }
    osd_menu_scroll_finish(3);

    osd_menu_select_save_and_exit(4);
    osd_menu_finish();
    break;

  case OSD_SCREEN_ELEMENTS_STYLE:
    osd_menu_start();
    osd_menu_header("OSD TEXT STYLE");

    const char *attr_labels[] = {
        "NORMAL",
        "INVERT",
    };

    osd_menu_scroll_start(4, 2, 7);
    for (uint8_t i = 0; i < OSD_ELEMENT_MAX; i++) {
      osd_element_t *el = (osd_element_t *)(osd_elements() + i);

      osd_menu_select(4, OSD_AUTO, osd_element_labels[i]);
      if (osd_menu_select_enum(20, OSD_AUTO, el->attribute, attr_labels)) {
        el->attribute = osd_menu_adjust_int(el->attribute, 1, 0, 1);
      }
    }
    osd_menu_scroll_finish(4);

    osd_menu_select_save_and_exit(4);
    osd_menu_finish();
    break;

  case OSD_SCREEN_CALLSIGN:
    osd_menu_start();
    osd_menu_header("CALLSIGN");

    osd_menu_select(1, 5, "EDIT:");
    if (osd_menu_select_str(8, 5, (char *)profile.osd.callsign)) {
      osd_menu_adjust_str((char *)profile.osd.callsign);
    }
    osd_menu_label(8, 6, "-------------------");

    osd_menu_select_save_and_exit(4);
    osd_menu_finish();
    break;

  case OSD_SCREEN_LOWBAT:
    osd_menu_start();
    osd_menu_header("LOW BATTERY");

    osd_menu_select(4, 5, "VOLTS/CELL ALERT");
    if (osd_menu_select_float(21, 5, profile.voltage.vbattlow, 4, 1)) {
      profile.voltage.vbattlow = osd_menu_adjust_float(profile.voltage.vbattlow, 0.1, 0, 4.2);
    }

    osd_menu_select_save_and_exit(4);
    osd_menu_finish();
    break;

  case OSD_SCREEN_LEVEL_MODE:
    osd_menu_start();
    osd_menu_header("LEVEL MODE");

    osd_menu_select_screen(7, 4, "MAX ANGLE", OSD_SCREEN_LEVEL_MAX_ANGLE);
    osd_menu_select_screen(7, 5, "LEVEL STRENGTH", OSD_SCREEN_LEVEL_STRENGTH);

    osd_menu_finish();
    break;

  case OSD_SCREEN_MOTOR_BOOST:
    osd_menu_start();
    osd_menu_header("MOTOR BOOST TYPES");

    osd_menu_select_screen(7, 4, "TORQUE BOOST", OSD_SCREEN_TORQUE_BOOST);
    osd_menu_select_screen(7, 5, "THROTTLE BOOST", OSD_SCREEN_THROTTLE_BOOST);

    osd_menu_finish();
    break;

  case OSD_SCREEN_MOTOR_SETTINGS:
    osd_menu_start();
    osd_menu_header("MOTOR SETTINGS");

    osd_menu_select(4, 5, "DIGITAL IDLE %");
    if (osd_menu_select_float(22, 5, profile.motor.digital_idle, 4, 1)) {
      profile.motor.digital_idle = osd_menu_adjust_float(profile.motor.digital_idle, 0.1, 0, 25.0);
    }

    osd_menu_select(4, 6, "TURTLE THROTTLE %");
    if (osd_menu_select_float(22, 6, profile.motor.turtle_throttle_percent, 4, 0)) {
      profile.motor.turtle_throttle_percent = osd_menu_adjust_float(profile.motor.turtle_throttle_percent, 1, 0, 100);
    }

    osd_menu_select(4, 7, "MOTOR LIMIT %");
    if (osd_menu_select_float(22, 7, profile.motor.motor_limit, 4, 0)) {
      profile.motor.motor_limit = osd_menu_adjust_float(profile.motor.motor_limit, 1, 0, 100);
    }

    osd_menu_select_save_and_exit(4);
    osd_menu_finish();
    break;

  case OSD_SCREEN_THROTTLE_SETTINGS:
    osd_menu_start();
    osd_menu_header("THROTTLE SETTINGS");

    osd_menu_select(4, 5, "THROTTLE MID");
    if (osd_menu_select_float(22, 5, profile.rate.throttle_mid, 4, 2)) {
      profile.rate.throttle_mid = osd_menu_adjust_float(profile.rate.throttle_mid, 0.01, 0, 1);
    }

    osd_menu_select(4, 6, "THROTTLE EXPO");
    if (osd_menu_select_float(22, 6, profile.rate.throttle_expo, 4, 2)) {
      profile.rate.throttle_expo = osd_menu_adjust_float(profile.rate.throttle_expo, 0.01, 0, 1);
    }

    osd_menu_select_save_and_exit(4);
    osd_menu_finish();
    break;

  case OSD_SCREEN_LEVEL_MAX_ANGLE:
    osd_menu_start();
    osd_menu_header("LEVEL MODE");

    osd_menu_select(1, 5, "MAX ANGLE DEGREES");
    if (osd_menu_select_float(19, 5, profile.rate.level_max_angle, 5, 1)) {
      profile.rate.level_max_angle = osd_menu_adjust_float(profile.rate.level_max_angle, 1, 0, 85.0);
    }

    osd_menu_select_save_and_exit(4);
    osd_menu_finish();
    break;

  case OSD_SCREEN_LEVEL_STRENGTH:
    osd_menu_start();
    osd_menu_header("LEVEL MODE");

    osd_menu_label(20, 5, "KP");
    osd_menu_label(26, 5, "KD");

    osd_menu_select(1, 6, "SM ANGLE STREN");
    if (osd_menu_select_float(18, 6, profile.pid.small_angle.kp, 6, 2)) {
      profile.pid.small_angle.kp = osd_menu_adjust_float(profile.pid.small_angle.kp, 0.5, 0, 20.0);
    }
    if (osd_menu_select_float(24, 6, profile.pid.small_angle.kd, 6, 2)) {
      profile.pid.small_angle.kd = osd_menu_adjust_float(profile.pid.small_angle.kd, 0.5, 0, 10.0);
    }

    osd_menu_select(1, 7, "LRG ANGLE STREN");
    if (osd_menu_select_float(18, 7, profile.pid.big_angle.kp, 6, 2)) {
      profile.pid.big_angle.kp = osd_menu_adjust_float(profile.pid.big_angle.kp, 0.5, 0, 20.0);
    }
    if (osd_menu_select_float(24, 7, profile.pid.big_angle.kd, 6, 2)) {
      profile.pid.big_angle.kd = osd_menu_adjust_float(profile.pid.big_angle.kd, 0.5, 0, 10.0);
    }

    osd_menu_select_save_and_exit(1);
    osd_menu_finish();
    break;

  case OSD_SCREEN_TORQUE_BOOST:
    osd_menu_start();
    osd_menu_header("TORQUE BOOST");

    osd_menu_select(4, 5, "MOTOR TORQUE BOOST");
    if (osd_menu_select_float(22, 5, profile.motor.torque_boost, 4, 1)) {
      profile.motor.torque_boost = osd_menu_adjust_float(profile.motor.torque_boost, 0.1, 0, 3.0);
    }

    osd_menu_select_save_and_exit(4);
    osd_menu_finish();
    break;

  case OSD_SCREEN_THROTTLE_BOOST:
    osd_menu_start();
    osd_menu_header("THROTTLE BOOST");

    osd_menu_select(2, 5, "MOTOR THROTTLE BOOST");
    if (osd_menu_select_float(22, 5, profile.motor.throttle_boost, 4, 1)) {
      profile.motor.throttle_boost = osd_menu_adjust_float(profile.motor.throttle_boost, 0.5, 0, 10.0);
    }

    osd_menu_select_save_and_exit(4);
    osd_menu_finish();
    break;

  case OSD_SCREEN_PID_MODIFIER:
    osd_menu_start();
    osd_menu_header("PID MODIFIERS");

    const char *modifier_state_labels[] = {
        " NONE ",
        "ACTIVE",
    };

    osd_menu_select_enum_adjust(2, 4, "P-TERM VOLTAGE COMP", 23, &profile.voltage.pid_voltage_compensation, modifier_state_labels, 0, 1);
    osd_menu_select_enum_adjust(2, 5, "THROTTLE D ATTENUATE", 23, &profile.pid.throttle_dterm_attenuation.tda_active, modifier_state_labels, 0, 1);

    osd_menu_select(2, 6, "TDA BREAKPOINT");
    if (osd_menu_select_float(23, 6, profile.pid.throttle_dterm_attenuation.tda_breakpoint, 4, 1)) {
      profile.pid.throttle_dterm_attenuation.tda_breakpoint = osd_menu_adjust_float(profile.pid.throttle_dterm_attenuation.tda_breakpoint, 0.05, 0, 1);
    }

    osd_menu_select(2, 7, "TDA PERCENT");
    if (osd_menu_select_float(23, 7, profile.pid.throttle_dterm_attenuation.tda_percent, 4, 1)) {
      profile.pid.throttle_dterm_attenuation.tda_percent = osd_menu_adjust_float(profile.pid.throttle_dterm_attenuation.tda_percent, 0.05, 0, 1);
    }

    osd_menu_select_save_and_exit(2);
    osd_menu_finish();
    break;

  case OSD_SCREEN_RC_LINK:
    osd_menu_start();
    osd_menu_header("RC LINK");

    osd_menu_select_screen(7, 4, "STICK CALIBRATION", OSD_SCREEN_STICK_WIZARD);
    osd_menu_select_screen(7, 5, "RSSI SOURCE", OSD_SCREEN_RSSI);

    osd_menu_finish();
    break;

  case OSD_SCREEN_RSSI: {
    osd_menu_start();
    osd_menu_header("RSSI SOURCE");

    const char *rssi_source_labels[] = {
        "PACKET RATE",
        "CHANNEL",
        "DIRECT",
    };

    osd_menu_select_enum_adjust(4, 4, "RSSI FROM", 16, &profile.receiver.lqi_source, rssi_source_labels, RX_LQI_SOURCE_PACKET_RATE, RX_LQI_SOURCE_DIRECT);
    if (profile.receiver.lqi_source == RX_LQI_SOURCE_CHANNEL) {
      osd_menu_select_enum_adjust(4, 5, "SELECT AUX", 16, &profile.receiver.aux[AUX_RSSI], aux_channel_labels, AUX_CHANNEL_0, AUX_CHANNEL_11);
    }

    osd_menu_select_save_and_exit(4);
    osd_menu_finish();
    break;
  }

  case OSD_SCREEN_BLACKBOX: {
    static uint8_t reset_state = 0;

    switch (reset_state) {
    default:
    case 0:
      osd_menu_start();
      osd_menu_header("BLACKBOX");

      osd_menu_select_screen(4, 3, "PRESETS", OSD_SCREEN_BLACKBOX_PRESETS);

      if (osd_menu_label_start(4, 5)) {
        osd_write_int(blackbox_device_header.file_num, 3);
        osd_write_str(" FILES");
      }

      if (osd_menu_label_start(4, 6)) {
        const uint32_t usage = (float)(blackbox_device_usage()) / (float)(blackbox_bounds.total_size) * 100;

        osd_write_int(usage, 3);
        osd_write_str("% USAGE");
      }

      if (osd_menu_button(4, 13, "RESET")) {
        osd_push_screen_replace(OSD_SCREEN_BLACKBOX);
        reset_state = 1;
      }
      osd_menu_select_exit(4);
      osd_menu_finish();
      break;

    case 1:
      osd_menu_start();
      osd_menu_header("BLACKBOX");

      osd_menu_label(9, 5, "RESETTING");
      osd_menu_label(9, 7, "PLEASE WAIT");

      if (osd_menu_finish()) {
        osd_push_screen_replace(OSD_SCREEN_BLACKBOX);
        reset_state = 2;
      }
      break;

    case 2:
      blackbox_device_reset();
      osd_push_screen_replace(OSD_SCREEN_BLACKBOX);
      reset_state = 0;
      break;
    }
    break;
  }

  case OSD_SCREEN_BLACKBOX_PRESETS: {
    osd_menu_start();
    osd_menu_header("BLACKBOX PRESETS");

    // Display currently active preset wrapped in angle brackets
    // as prefacing it with "ACTIVE: " takes too much space
    const char *label = NULL;
    for (int i = 0; i < blackbox_presets_count; i++) {
      const blackbox_preset_t *preset = &blackbox_presets[i];
      if (blackbox_preset_equals(preset, &profile.blackbox)) {
        label = preset->name_osd;
        break;
      }
    }
    if (osd_menu_label_start(3, 3)) {
      osd_write_char('<');
      if (label) {
        osd_write_str(label);
      } else {
        const uint32_t rate = profile.blackbox.sample_rate_hz;
        osd_write_str("CUSTOM, HZ=");
        osd_write_int(rate, 4);
      }
      osd_write_char('>');
    }

    // Display available presets
    int y = 5;
    for (int i = 0; i < blackbox_presets_count; i++, y++) {
      const blackbox_preset_t *preset = &blackbox_presets[i];
      if (osd_menu_button(3, y, preset->name_osd)) {
        blackbox_preset_apply(preset, &profile.blackbox);
        if (osd_menu_finish()) {
          osd_push_screen_replace(OSD_SCREEN_BLACKBOX_PRESETS);
        }
      }
    }

    osd_menu_select_save_and_exit(3);
    osd_menu_finish();
    break;
  }

  case OSD_SCREEN_STICK_WIZARD:
    osd_menu_start();
    osd_menu_header("STICK CALIBRATION");

    osd_menu_label(9, 5, "LEFT TO EXIT");
    osd_menu_label(9, 7, "RIGHT TO BEGIN");

    if (osd_menu_finish() && osd_state.selection) {
      stick_wizard_start(false);
      osd_push_screen_replace(OSD_SCREEN_STICK_WIZARD_CALIBRATION);
      osd_state.selection = 0;
    }
    break;

  case OSD_SCREEN_STICK_WIZARD_CALIBRATION:
    osd_menu_start();
    osd_menu_header("RECORDING");

    osd_menu_label(9, 5, "MOVE STICKS");
    osd_menu_label(9, 7, "TO EXTENTS");

    if (osd_menu_finish() && state.stick_calibration_wizard == STICK_WIZARD_WAIT_FOR_CONFIRM) {
      osd_push_screen_replace(OSD_SCREEN_STICK_CONFIRM);
    }
    break;

  case OSD_SCREEN_STICK_CONFIRM: // 5 sec to test / confirm calibration
    osd_menu_start();
    osd_menu_header("TESTING CALIBRATION");

    osd_menu_label(6, 5, "MOVE STICKS AGAIN");
    osd_menu_label(9, 7, "TO EXTENTS");

    if (osd_menu_finish() && ((state.stick_calibration_wizard == STICK_WIZARD_SUCCESS) || (state.stick_calibration_wizard == STICK_WIZARD_FAILED))) {
      osd_push_screen_replace(OSD_SCREEN_STICK_RESULT);
    }
    break;

  case OSD_SCREEN_STICK_RESULT: // results of calibration
    osd_menu_start();
    osd_menu_header("STICK CALIBRATION");

    if (state.stick_calibration_wizard == STICK_WIZARD_SUCCESS) {
      osd_menu_label(10, 4, "CALIBRATION");
      osd_menu_label(12, 6, "SUCCESS");
      osd_menu_label(7, 8, "PUSH LEFT TO EXIT");
    }
    if (state.stick_calibration_wizard == STICK_WIZARD_FAILED) {
      osd_menu_label(10, 4, "CALIBRATION");
      osd_menu_label(12, 6, "FAILED");
      osd_menu_label(7, 8, "PUSH LEFT TO EXIT");
    }

    osd_menu_finish();

    if (osd_state.selection > 0) {
      osd_state.selection = 0;
    }
    break;
  }
}
