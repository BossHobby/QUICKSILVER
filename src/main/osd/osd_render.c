#include "osd_render.h"

#include "control.h"
#include "debug.h"
#include "drv_osd.h"
#include "drv_time.h"
#include "filter.h"
#include "flash.h"
#include "float.h"
#include "osd_adjust.h"
#include "osd_menu_maps.h"
#include "profile.h"
#include "project.h"
#include "rx.h"
#include "stdio.h"
#include "string.h"
#include "util.h"
#include "vtx.h"

#ifdef ENABLE_OSD

// double promotions the following are unavoidable
#pragma GCC diagnostic push
#pragma GCC diagnostic ignored "-Wdouble-promotion"

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

// Flash Variables - 32bit					# of osd elements and flash memory start position in defines.h
extern profile_t profile;
extern vtx_settings_t vtx_settings;
extern vtx_settings_t vtx_settings_copy;

static osd_system_t osd_system = OSD_SYS_NONE;

osd_state_t osd_state;

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

//************************************************************************************************************************************************************************************
//																					STATE VARIABLES
//************************************************************************************************************************************************************************************
// case & state variables for switch logic and profile adjustments
uint8_t last_display_phase;
uint8_t osd_cursor;
uint8_t last_osd_cursor[6];
uint8_t osd_select;
uint8_t increase_osd_value;
uint8_t decrease_osd_value;
uint8_t reboot_fc_requested = 0;
#define MAIN_MENU 1
#define SUB_MENU 0

//************************************************************************************************************************************************************************************
//																					UTILITY FUNCTIONS
//************************************************************************************************************************************************************************************

void osd_display_reset() {
  osd_state.menu_phase = 0;                     // reset menu to to main menu
  osd_state.display_phase = OSD_SCREEN_REGULAR; // jump to regular osd display next loop
  osd_state.element = 0;                        // start with first screen element
}

uint8_t user_select(uint8_t active_elements, uint8_t total_elements) {
  if (osd_cursor < 1)
    osd_cursor = 1;
  if (osd_cursor > active_elements)
    osd_cursor = active_elements;
  uint8_t inactive_elements = total_elements - active_elements;
  if (osd_state.menu_phase == 1)
    return OSD_ATTR_INVERT;
  if (osd_state.menu_phase > 1 && osd_state.menu_phase <= inactive_elements)
    return OSD_ATTR_TEXT;
  if (osd_cursor == (osd_state.menu_phase - inactive_elements) && osd_select == 0) {
    return OSD_ATTR_INVERT;
  } else {
    return OSD_ATTR_TEXT;
  }
}

uint8_t grid_selection(uint8_t element, uint8_t row) {
  if (osd_select == element && osd_cursor == row) {
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

#define HOLD 0
#define TEMP 1

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
  if (osd_state.menu_phase <= string_element_qty)
    return;
  if (osd_state.menu_phase > string_element_qty + data_element_qty)
    return;
  static uint8_t skip_loop = 0;
  if (osd_state.menu_phase == string_element_qty + 1 && skip_loop == 0) { // skip a loop to prevent dma collision with previous print function
    skip_loop++;
    return;
  }
  skip_loop = 0;
  uint8_t index = osd_state.menu_phase - string_element_qty - 1;

  osd_transaction_t *txn = osd_txn_init();
  osd_txn_start(grid_selection(grid[index][0], grid[index][1]), print_position[index][0], print_position[index][1]);
  osd_txn_write_char(profile.osd.callsign[index]);
  osd_txn_submit(txn);

  osd_state.menu_phase++;
}

void print_osd_menu_strings(uint8_t string_element_qty, uint8_t active_element_qty, const char element_names[string_element_qty][21], const uint8_t print_position[string_element_qty][2]) {
  if (osd_state.menu_phase > string_element_qty)
    return;
  if (osd_state.menu_phase == 0) {
    if (osd_clear_async())
      osd_state.menu_phase++;
    return;
  }

  osd_transaction_t *txn = osd_txn_init();
  osd_txn_start(user_select(active_element_qty, string_element_qty), print_position[osd_state.menu_phase - 1][0], print_position[osd_state.menu_phase - 1][1]);
  osd_txn_write_str(element_names[osd_state.menu_phase - 1]);
  osd_txn_submit(txn);

  osd_state.menu_phase++;
}

void print_osd_adjustable_enums(uint8_t string_element_qty, uint8_t data_element_qty, const char data_to_print[21], const uint8_t grid[data_element_qty][2], const uint8_t print_position[data_element_qty][2]) {
  if (osd_state.menu_phase <= string_element_qty)
    return;
  if (osd_state.menu_phase > string_element_qty + data_element_qty)
    return;
  static uint8_t skip_loop = 0;
  if (osd_state.menu_phase == string_element_qty + 1 && skip_loop == 0) { // skip a loop to prevent dma collision with previous print function
    skip_loop++;
    return;
  }
  skip_loop = 0;
  uint8_t index = osd_state.menu_phase - string_element_qty - 1;

  osd_transaction_t *txn = osd_txn_init();
  osd_txn_start(grid_selection(grid[index][0], grid[index][1]), print_position[index][0], print_position[index][1]);
  osd_txn_write_str(data_to_print);
  osd_txn_submit(txn);

  osd_state.menu_phase++;
}

void print_osd_adjustable_vectors(uint8_t menu_type, uint8_t string_element_qty, uint8_t data_element_qty, vec3_t *pointer, const uint8_t data_index[data_element_qty][2], const uint8_t grid[data_element_qty][2], const uint8_t print_position[data_element_qty][2]) {
  if (osd_state.menu_phase <= string_element_qty)
    return;
  if (osd_state.menu_phase > string_element_qty + data_element_qty)
    return;
  static uint8_t skip_loop = 0;
  if (osd_state.menu_phase == string_element_qty + 1 && skip_loop == 0) { // skip a loop to prevent dma collision with previous print function
    skip_loop++;
    return;
  }
  skip_loop = 0;

  uint8_t index = osd_state.menu_phase - string_element_qty - 1;

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

  osd_state.menu_phase++;
}

void print_osd_adjustable_float(uint8_t string_element_qty, uint8_t data_element_qty, float *pointer[], const uint8_t grid[data_element_qty][2], const uint8_t print_position[data_element_qty][2], uint8_t precision) {
  if (osd_state.menu_phase <= string_element_qty)
    return;
  if (osd_state.menu_phase > string_element_qty + data_element_qty)
    return;
  static uint8_t skip_loop = 0;
  if (osd_state.menu_phase == string_element_qty + 1 && skip_loop == 0) { // skip a loop to prevent dma collision with previous print function
    skip_loop++;
    return;
  }
  skip_loop = 0;

  uint8_t index = osd_state.menu_phase - string_element_qty - 1;

  osd_transaction_t *txn = osd_txn_init();
  osd_txn_start(grid_selection(grid[index][0], grid[index][1]), print_position[index][0], print_position[index][1]);
  osd_txn_write_float(*pointer[index] + FLT_EPSILON, 4, precision);
  osd_txn_submit(txn);

  osd_state.menu_phase++;
}

void print_osd_mixed_data(uint8_t string_element_qty, uint8_t data_element_qty, float *pointer[], uint8_t *pointer2[], const char filtertype_labels[5][21], const uint8_t grid[data_element_qty][2], const uint8_t print_position[data_element_qty][2], uint8_t precision) {
  if (osd_state.menu_phase <= string_element_qty)
    return;
  if (osd_state.menu_phase > string_element_qty + data_element_qty)
    return;
  static uint8_t skip_loop = 0;
  if (osd_state.menu_phase == string_element_qty + 1 && skip_loop == 0) { // skip a loop to prevent dma collision with previous print function
    skip_loop++;
    return;
  }
  skip_loop = 0;

  uint8_t index = osd_state.menu_phase - string_element_qty - 1;

  osd_transaction_t *txn = osd_txn_init();
  osd_txn_start(grid_selection(grid[index][0], grid[index][1]), print_position[index][0], print_position[index][1]);
  if (*pointer[index] != POINTER_REDIRECT) {
    osd_txn_write_float(*pointer[index] + FLT_EPSILON, 4, precision);
  } else {
    osd_txn_write_str(filtertype_labels[*pointer2[index]]);
  }
  osd_txn_submit(txn);

  osd_state.menu_phase++;
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

  //************OSD MENU DISPLAY ROUTINES HERE*************
  switch (osd_state.display_phase) // phase starts at 2, RRR gesture subtracts 1 to enter the menu, RRR again or DDD subtracts 1 to clear the screen and return to regular display
  {
  case OSD_SCREEN_CLEAR: // osd screen clears, resets to regular display, and resets wizard and menu starting points
    if (osd_clear_async())
      osd_display_reset();
    break; // screen has been cleared for this loop - break out of display function

  case OSD_SCREEN_MAIN_MENU: // osd menu is active
    last_display_phase = 2;
    print_osd_menu_strings(10, 9, main_menu_labels, main_menu_positions);
    if (osd_state.menu_phase == 11)
      osd_select_menu_item(8, main_menu_map, MAIN_MENU);
    break; // osd menu has been displayed for this loop	- break out of display function

  case OSD_SCREEN_REGULAR: // regular osd display
    osd_display_regular();
    break;

    //**********************************************************************************************************************************************************************************************
    //																				OSD MENUS BELOW THIS POINT
    //**********************************************************************************************************************************************************************************************
  case OSD_SCREEN_PID_PROFILE:
    last_display_phase = 1;
    print_osd_menu_strings(3, 2, pid_profiles_labels, pid_profiles_positions);
    if (osd_state.menu_phase == 4)
      osd_submenu_select(&profile.pid.pid_profile, 2, pid_submenu_map);
    break;

  case OSD_SCREEN_PID:
    last_display_phase = 3;
    if (profile.pid.pid_profile == PID_PROFILE_1)
      print_osd_menu_strings(8, 4, pid_profile1_labels, pid_profile_positions);
    else
      print_osd_menu_strings(8, 4, pid_profile2_labels, pid_profile_positions);
    print_osd_adjustable_vectors(BF_PIDS, 8, 9, get_pid_term(pid_profile_data_index[osd_state.menu_phase - 9][0]), pid_profile_data_index, pid_profile_grid, pid_profile_data_positions);
    if (osd_state.menu_phase == 18)
      osd_vector_adjust(get_pid_term(osd_cursor), 3, 3, BF_PIDS, pid_profile_adjust_limits);
    break;

  case OSD_SCREEN_FILTERS:
    last_display_phase = 1;
    print_osd_menu_strings(3, 2, filter_labels, filter_positions);
    if (osd_state.menu_phase == 4)
      osd_select_menu_item(2, filter_submenu_map, SUB_MENU);
    break;

  case OSD_SCREEN_RATES:
    last_display_phase = 1;
    print_osd_menu_strings(3, 2, rates_profile_labels, rates_profile_positions);
    if (osd_state.menu_phase == 4)
      osd_submenu_select(&profile_current_rates()->mode, 2, rates_submenu_map);
    break;

  case OSD_SCREEN_SW_RATES:
    last_display_phase = 6;
    print_osd_menu_strings(8, 4, sw_rates_labels, sw_rates_positions);
    print_osd_adjustable_vectors(SW_RATES, 8, 9, get_sw_rate_term(sw_rates_data_index[osd_state.menu_phase - 9][0]), sw_rates_data_index, sw_rates_grid, sw_rates_data_positions);
    if (osd_state.menu_phase == 18)
      osd_vector_adjust(get_sw_rate_term(osd_cursor), 3, 3, SW_RATES, sw_rates_adjust_limits);
    break;

  case OSD_SCREEN_BF_RATES:
    last_display_phase = 6;
    print_osd_menu_strings(8, 4, bf_rates_labels, bf_rates_positions);
    print_osd_adjustable_vectors(ROUNDED, 8, 9, get_bf_rate_term(bf_rates_data_index[osd_state.menu_phase - 9][0]), bf_rates_data_index, bf_rates_grid, bf_rates_data_positions);
    if (osd_state.menu_phase == 18)
      osd_vector_adjust(get_bf_rate_term(osd_cursor), 3, 3, ROUNDED, bf_rates_adjust_limits);
    break;

  case OSD_SCREEN_FLIGHT_MODES:
    last_display_phase = 1;
    print_osd_menu_strings(12, 11, flight_modes_labels, flight_modes_positions);
    print_osd_adjustable_enums(12, 10, get_aux_status(profile.receiver.aux[flight_modes_aux_items[osd_state.menu_phase - 13]]), flight_modes_grid, flight_modes_data_positions);
    if (osd_state.menu_phase == 23)
      osd_enum_adjust(flight_modes_ptr, 10, flight_modes_aux_limits);
    break;

  case OSD_SCREEN_ELEMENTS:
    last_display_phase = 1;
    print_osd_menu_strings(5, 4, osd_elements_menu_labels, osd_elements_menu_positions);
    if (osd_state.menu_phase == 6)
      osd_select_menu_item(4, osd_elements_map, SUB_MENU);
    break;

  case OSD_SCREEN_VTX:
    last_display_phase = 1;
    if (vtx_settings.detected) {
      populate_vtx_buffer_once();
      print_osd_menu_strings(6, 5, vtx_labels, vtx_positions);
      // print the buffer and not the actual status
      print_osd_adjustable_enums(6, 4, get_vtx_status(osd_state.menu_phase - 7), vtx_grid, vtx_data_positions);
      if (osd_state.menu_phase == 11)
        // adjust the buffer and not the actual settings
        osd_enum_adjust(vtx_ptr, 4, vtx_limits);
      // save & exit needs a function to write the buffer to the actual settings
    } else {
      print_osd_menu_strings(3, 0, vtx_na_labels, vtx_na_positions);
      if (osd_select)
        osd_select = 0;
    }
    break;

  case OSD_SCREEN_SPECIAL_FEATURES:
    last_display_phase = 1;
    print_osd_menu_strings(8, 7, special_features_labels, special_features_positions);
    if (osd_state.menu_phase == 9)
      osd_select_menu_item(7, special_features_map, SUB_MENU);
    break;

  case OSD_SCREEN_STICK_BOOST:
    last_display_phase = 12;
    print_osd_menu_strings(3, 2, stickboost_labels, stickboost_profile_positions);
    if (osd_state.menu_phase == 4)
      osd_submenu_select(&profile.pid.stick_profile, 2, stickboost_submenu_map);
    break;

  case OSD_SCREEN_STICK_BOOST_ADJUST:
    last_display_phase = 13;
    if (profile.pid.stick_profile == STICK_PROFILE_OFF)
      print_osd_menu_strings(7, 3, stickboost1_labels, stickboost_positions);
    else
      print_osd_menu_strings(7, 3, stickboost2_labels, stickboost_positions);
    print_osd_adjustable_vectors(ROUNDED, 7, 6, get_stick_profile_term(stickboost_data_index[osd_state.menu_phase - 8][0]), stickboost_data_index, stickboost_grid, stickboost_data_positions);
    if (osd_state.menu_phase == 14)
      osd_vector_adjust(get_stick_profile_term(osd_cursor), 2, 3, ROUNDED, stickboost_adjust_limits);
    break;

  case OSD_SCREEN_ELEMENTS_ADD_REMOVE:
    last_display_phase = 10;
    print_osd_menu_strings(12, 11, osd_display_labels, osd_display_positions);
    print_osd_adjustable_enums(12, 10, get_decode_element_string((osd_element_t *)(osd_elements() + osd_elements_active_items[osd_state.menu_phase - 13]), ACTIVE), osd_display_grid, osd_display_data_positions);
    if (osd_state.menu_phase == 23)
      osd_encoded_adjust(osd_elements() + osd_elements_active_items[osd_cursor - 1], 10, 1, ACTIVE);
    break;

  case OSD_SCREEN_ELEMENTS_POSITION:
    last_display_phase = 10;
    print_osd_menu_strings(14, 11, osd_position_labels, osd_position_adjust_positions);
    print_osd_adjustable_enums(14, 20, get_decode_element_string((osd_element_t *)(osd_elements() + osd_position_active_items[osd_state.menu_phase - 15]), osd_position_index[osd_state.menu_phase - 15]), osd_position_grid, osd_position_data_positions);
    if (osd_state.menu_phase == 35 && osd_select > 0)
      osd_encoded_adjust(osd_elements() + osd_elements_active_items[osd_cursor - 1], 10, 2, osd_select + 1);
    break;

  case OSD_SCREEN_ELEMENTS_STYLE:
    last_display_phase = 10;
    print_osd_menu_strings(12, 11, osd_text_style, osd_text_style_positions);
    print_osd_adjustable_enums(12, 10, get_decode_element_string((osd_element_t *)(osd_elements() + osd_elements_active_items[osd_state.menu_phase - 13]), ATTRIBUTE), osd_display_grid, osd_display_data_positions);
    if (osd_state.menu_phase == 23)
      osd_encoded_adjust(osd_elements() + osd_elements_active_items[osd_cursor - 1], 10, 1, ATTRIBUTE);
    break;

  case OSD_SCREEN_CALLSIGN:
    last_display_phase = 10;
    print_osd_menu_strings(23, 2, osd_callsign_edit_labels, osd_callsign_edit_positions);
    print_osd_callsign_adjustable(23, 20, osd_callsign_grid, osd_callsign_edit_data_positions);
    if (osd_state.menu_phase == 44)
      osd_encoded_adjust_callsign();
    break;

  case OSD_SCREEN_LOWBAT:
    last_display_phase = 12;
    print_osd_menu_strings(3, 2, lowbatt_labels, lowbatt_positions);
    print_osd_adjustable_float(3, 1, low_batt_ptr, lowbatt_grid, lowbatt_data_positions, 1);
    if (osd_state.menu_phase == 5)
      osd_float_adjust(low_batt_ptr, 1, 1, lowbatt_adjust_limits, 0.1);
    break;

  case OSD_SCREEN_LEVEL_MODE:
    last_display_phase = 12;
    print_osd_menu_strings(3, 2, level_submenu_labels, level_submenu_positions);
    if (osd_state.menu_phase == 4)
      osd_select_menu_item(2, level_submenu_map, SUB_MENU);
    break;

  case OSD_SCREEN_MOTOR_BOOST:
    last_display_phase = 12;
    print_osd_menu_strings(3, 2, motor_boost_labels, motor_boost_positions);
    if (osd_state.menu_phase == 4)
      osd_select_menu_item(2, motor_boost_map, SUB_MENU);
    break;

  case OSD_SCREEN_DIGITAL_IDLE:
    last_display_phase = 12;
    print_osd_menu_strings(3, 2, motoridle_labels, motoridle_positions);
    print_osd_adjustable_float(3, 1, motoridle_ptr, motoridle_grid, motoridle_data_positions, 1);
    if (osd_state.menu_phase == 5)
      osd_float_adjust(motoridle_ptr, 1, 1, motoridle_adjust_limits, 0.1);
    break;

  case OSD_SCREEN_LEVEL_MAX_ANGLE:
    last_display_phase = 20;
    print_osd_menu_strings(3, 2, maxangle_labels, maxangle_positions);
    print_osd_adjustable_float(3, 1, level_maxangle_ptr, maxangle_grid, maxangle_data_positions, 0);
    if (osd_state.menu_phase == 5)
      osd_float_adjust(level_maxangle_ptr, 1, 1, maxangle_adjust_limits, 1.0);
    break;

  case OSD_SCREEN_LEVEL_STRENGTH:
    last_display_phase = 20;
    print_osd_menu_strings(6, 3, levelmode_labels, levelmode_positions);
    print_osd_adjustable_float(6, 4, level_pid_ptr, levelmode_grid, levelmode_data_positions, 1);
    if (osd_state.menu_phase == 11)
      osd_float_adjust(level_pid_ptr, 2, 2, levelmode_adjust_limits, 0.5);
    break;

  case OSD_SCREEN_TORQUE_BOOST:
    last_display_phase = 21;
    print_osd_menu_strings(3, 2, torqueboost_labels, torqueboost_positions);
    print_osd_adjustable_float(3, 1, torqueboost_ptr, torqueboost_grid, torqueboost_data_positions, 1);
    if (osd_state.menu_phase == 5)
      osd_float_adjust(torqueboost_ptr, 1, 1, torqueboost_adjust_limits, 0.1);
    break;

  case OSD_SCREEN_THROTTLE_BOOST:
    last_display_phase = 21;
    print_osd_menu_strings(3, 2, throttleboost_labels, throttleboost_positions);
    print_osd_adjustable_float(3, 1, throttleboost_ptr, throttleboost_grid, throttleboost_data_positions, 1);
    if (osd_state.menu_phase == 5)
      osd_float_adjust(throttleboost_ptr, 1, 1, throttleboost_adjust_limits, 0.5);
    break;

  case OSD_SCREEN_TURTLE_THROTTLE:
    last_display_phase = 12;
    print_osd_menu_strings(3, 2, turtlethrottle_labels, turtlethrottle_positions);
    print_osd_adjustable_float(3, 1, turtlethrottle_ptr, turtlethrottle_grid, turtlethrottle_data_positions, 0);
    if (osd_state.menu_phase == 5)
      osd_float_adjust(turtlethrottle_ptr, 1, 1, turtlethrottle_adjust_limits, 10.0);
    break;

  case OSD_SCREEN_GYRO_FILTER:
    last_display_phase = 5;
    print_osd_menu_strings(6, 5, gyrofilter_labels, gyrofilter_positions);
    print_osd_mixed_data(6, 4, gyrofilter_ptr, gyrofilter_ptr2, gyrofilter_type_labels, gyrofilter_grid, gyrofilter_data_positions, 0);
    if (osd_state.menu_phase == 11)
      osd_mixed_data_adjust(gyrofilter_ptr, gyrofilter_ptr2, 4, 1, gyrofilter_adjust_limits, 10.0, gyrofilter_reboot_request);
    break;

  case OSD_SCREEN_DTERM_FILTER:
    last_display_phase = 5;
    print_osd_menu_strings(9, 8, dtermfilter_labels, dtermfilter_positions);
    print_osd_mixed_data(9, 7, dtermfilter_ptr, dtermfilter_ptr2, dtermfilter_type_labels, dtermfilter_grid, dtermfilter_data_positions, 0);
    if (osd_state.menu_phase == 17)
      osd_mixed_data_adjust(dtermfilter_ptr, dtermfilter_ptr2, 7, 1, dtermfilter_adjust_limits, 10.0, dtermfilter_reboot_request);
    break;

  case OSD_SCREEN_PID_MODIFIER:
    last_display_phase = 12;
    print_osd_menu_strings(6, 5, pidmodify_labels, pidmodify_positions);
    print_osd_mixed_data(6, 4, pidmodify_ptr, pidmodify_ptr2, pidmodify_type_labels, pidmodify_grid, pidmodify_data_positions, 2);
    if (osd_state.menu_phase == 11)
      osd_mixed_data_adjust(pidmodify_ptr, pidmodify_ptr2, 4, 1, pidmodify_adjust_limits, 0.05, pidmodify_reboot_request);
    break;

  case OSD_SCREEN_RC_LINK:
    last_display_phase = 1;
    print_osd_menu_strings(3, 2, rc_link_labels, rc_link_positions);
    if (osd_state.menu_phase == 4)
      osd_select_menu_item(7, rc_link_map, SUB_MENU);
    break;

  case OSD_SCREEN_RSSI:
    last_display_phase = 31;
    print_osd_menu_strings(4, 3, rssi_menu_labels, rssi_menu_positions);
    print_osd_adjustable_enums(4, 2, get_rssi_source_status(osd_state.menu_phase - 5), rssi_source_data_grid, rssi_source_data_positions);
    if (osd_state.menu_phase == 7)
      osd_enum_adjust(rssi_source_ptr, 2, rssi_source_limits);
    break;

  case OSD_SCREEN_STICK_WIZARD: // stick wizard select menu
    last_display_phase = 31;
    print_osd_menu_strings(3, 0, stick_wizard_labels_1, stick_wizard_positions_1);
    if (osd_state.menu_phase == 4) {
      if (osd_select) {
        request_stick_calibration_wizard();
        osd_state.display_phase = OSD_SCREEN_STICK_WIZARD_CALIBRATION;
        osd_select = 0;
        osd_state.menu_phase = 0;
      }
    }
    break;

  case OSD_SCREEN_STICK_WIZARD_CALIBRATION: // 5 sec to calibrate
    print_osd_menu_strings(3, 0, stick_wizard_labels_2, stick_wizard_positions_2);
    if (osd_state.menu_phase == 4) {
      if (state.stick_calibration_wizard == WAIT_FOR_CONFIRM) {
        osd_state.display_phase = OSD_SCREEN_STICK_CONFIRM;
        osd_state.menu_phase = 0;
      }
    }
    break;

  case OSD_SCREEN_STICK_CONFIRM: // 5 sec to test / confirm calibration
    print_osd_menu_strings(3, 0, stick_wizard_labels_3, stick_wizard_positions_3);
    if (osd_state.menu_phase == 4) {
      if ((state.stick_calibration_wizard == CALIBRATION_SUCCESS) || (state.stick_calibration_wizard == TIMEOUT)) {
        osd_state.display_phase = OSD_SCREEN_STICK_RESULT;
        osd_state.menu_phase = 0;
      }
    }
    break;

  case OSD_SCREEN_STICK_RESULT: // results of calibration
    last_display_phase = 31;
    if (state.stick_calibration_wizard == CALIBRATION_SUCCESS) {
      print_osd_menu_strings(4, 0, stick_wizard_labels_4, stick_wizard_positions_4); // osd_state.menu_phase will be 4 after this
    }
    if (state.stick_calibration_wizard == TIMEOUT) {
      print_osd_menu_strings(4, 0, stick_wizard_labels_5, stick_wizard_positions_5); // osd_state.menu_phase will be 4 after this
    }
    if (osd_select > 0)
      osd_select = 0;
    break;
  }

  if (osd_state.display_phase != OSD_SCREEN_REGULAR && rx_aux_on(AUX_ARMING)) {
    flags.arm_safety = 1;
  }
}
//******************************************************************************************************************************

#pragma GCC diagnostic pop
#endif
