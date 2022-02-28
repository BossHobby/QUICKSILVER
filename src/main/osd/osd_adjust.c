#include "osd_adjust.h"

#include <stdio.h>

#include "drv_spi_max7456.h"
#include "drv_time.h"
#include "flash.h"
#include "osd_menu_maps.h"
#include "osd_render.h"
#include "profile.h"
#include "project.h"
#include "util.h"
#include "vtx.h"

extern profile_t profile;
extern vtx_settings_t vtx_settings;
vtx_settings_t vtx_settings_copy;
static uint8_t vtx_buffer_populated = 0;

//**************************************************************** utility and tracking functions*********************************************************

void osd_save_exit() {
  osd_state.selection = 0;
  osd_state.cursor = 1;
  osd_state.cursor_history_size = 0;
  osd_state.screen = OSD_SCREEN_CLEAR;

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
  reset_looptime();

  if (osd_state.reboot_fc_requested)
    NVIC_SystemReset();
}

uint8_t osd_push_cursor() {
  osd_state.cursor_history[osd_state.cursor_history_size] = osd_state.cursor;
  osd_state.cursor_history_size++;
  return osd_state.cursor;
}

uint8_t osd_pop_cursor() {
  if (!osd_state.cursor_history_size) {
    return 0;
  }

  osd_state.cursor = osd_state.cursor_history[osd_state.cursor_history_size - 1];
  osd_state.cursor_history_size--;

  return osd_state.cursor;
}

void osd_submenu_select(uint8_t *pointer, uint8_t rows, const uint8_t next_menu[]) {
  if (osd_state.selection == 1) { // stick was pushed right to select a next menu
    osd_state.selection = 0;      // reset the trigger
    osd_push_cursor();
    if (osd_state.cursor <= rows) {
      *pointer = osd_state.cursor - 1; // update profile
      osd_push_screen(next_menu[osd_state.cursor - 1]);
    }
  }
}

void osd_select_menu_item(uint8_t rows, const uint8_t menu_map[], uint8_t main_menu) {
  if (osd_state.selection == 1) { // main menu
    osd_state.selection = 0;      // reset the trigger
    osd_push_cursor();
    if (osd_state.cursor <= rows) {
      osd_push_screen(menu_map[osd_state.cursor - 1]);
    }
    if (main_menu && osd_state.cursor == rows + 1) {
      osd_save_exit(); // include save&exit in main menu
    }
  }
}

// populate a vtx_status_temp_buffer with current settings only once
void populate_vtx_buffer_once() {
  if (!vtx_buffer_populated) {
    vtx_settings_copy = vtx_settings;
    vtx_buffer_populated = 1;
  }
  return;
}

//**********************************************************encoded flash memory adjust functions**********************************************************

void osd_encoded_adjust_callsign() {
  if (osd_state.selection > 20) {
    osd_state.selection = 20;   // limit osd select variable from accumulating past 1 columns of adjustable items
    osd_state.screen_phase = 1; // repaint the screen again
  }
  if (osd_state.selection_increase) {
    profile.osd.callsign[osd_state.selection - 1]++;
    osd_state.screen_phase = 1; // repaint the screen again
  }
  if (osd_state.selection_decrease) {
    profile.osd.callsign[osd_state.selection - 1]--;
    osd_state.screen_phase = 1; // repaint the screen again
  }

  osd_state.selection_increase = 0;
  osd_state.selection_decrease = 0;

  if (osd_state.cursor == 2 && osd_state.selection == 1) {
    osd_save_exit();
  }
}

void osd_encoded_adjust(uint32_t *pointer, uint8_t rows, uint8_t columns, uint8_t status) {
  if (osd_state.selection > columns) {
    osd_state.selection = columns; // limit osd select variable from accumulating past 1 columns of adjustable items
    osd_state.screen_phase = 1;    // repaint the screen again
  }
  if (osd_state.cursor <= rows) {
    switch (status) {
    case 0: // adjust active or inactive element
      if (osd_state.selection_increase && osd_decode(*pointer, status) == 0x00) {
        *pointer = *pointer + 1;
        osd_state.screen_phase = 1; // repaint the screen again
      }
      if (osd_state.selection_decrease && osd_decode(*pointer, status) == 0x01) {
        *pointer = *pointer - 1;
        osd_state.screen_phase = 1; // repaint the screen again
      }
      break;
    case 1:                                                                       // adjust TEXT or INVERT
      if (osd_state.selection_increase && osd_decode(*pointer, status) == TEXT) { // increase requested and currently on TEXT
        *pointer = *pointer | (0x02);                                             // flip the 2nd bit on
        osd_state.screen_phase = 1;                                               // repaint the screen again
      }
      if (osd_state.selection_decrease && osd_decode(*pointer, status) == INVERT) { // decrease requested and currently on INVERT
        *pointer = *pointer ^ (0x02);                                               // flip the 2nd bit off
        osd_state.screen_phase = 1;                                                 // repaint the screen again
      }
      break;
    case 2: // adjust positionX
      if (osd_state.selection_increase && osd_decode(*pointer, status) != 30) {
        *pointer = (((*pointer >> 2) + 1) << 2) + (*pointer & 0x03);
        osd_state.screen_phase = 1; // repaint the screen again
      }
      if (osd_state.selection_decrease && osd_decode(*pointer, status) != 0) {
        *pointer = (((*pointer >> 2) - 1) << 2) + (*pointer & 0x03);
        osd_state.screen_phase = 1; // repaint the screen again
      }
      break;
    case 3: // adjust positionY
      if (osd_state.selection_increase && osd_decode(*pointer, status) != 15) {
        *pointer = (((*pointer >> 7) + 1) << 7) + (*pointer & 0x7F);
        osd_state.screen_phase = 1; // repaint the screen again
      }
      if (osd_state.selection_decrease && osd_decode(*pointer, status) != 0) {
        *pointer = (((*pointer >> 7) - 1) << 7) + (*pointer & 0x7F);
        osd_state.screen_phase = 1; // repaint the screen again
      }
      break;
    }
    osd_state.selection_increase = 0;
    osd_state.selection_decrease = 0;
  }

  if (osd_state.cursor == rows + 1 && osd_state.selection == 1) {
    osd_save_exit();
  }
}

//************************************************************profile variable adjust functions***********************************************************

float adjust_rounded_float(float input, float adjust_amount) {
  const float value = (int)(input * 100.0f + (input <= 0 ? -0.5f : 0.5f));
  if (osd_state.selection_increase) {
    osd_state.selection_increase = 0;
    osd_state.screen_phase = 1; // repaint the screen again
    return (float)(value + (100.0f * adjust_amount)) / 100.0f;
  }
  if (osd_state.selection_decrease) {
    osd_state.selection_decrease = 0;
    osd_state.screen_phase = 1; // repaint the screen again
    return (float)(value - (100.0f * adjust_amount)) / 100.0f;
  }
  return input;
}

const char *get_rssi_source_status(uint8_t data_to_print) {
  static char *rssi_source_status[] = {"PACKET RATE", "CHANNEL    ", "PROTOCOL   "};
  static char *rssi_source_channel[] = {"CHANNEL 5  ", "CHANNEL 6  ", "CHANNEL 7  ", "CHANNEL 8  ", "CHANNEL 9  ", "CHANNEL 10 ", "CHANNEL 11 ", "CHANNEL 12 ", "CHANNEL 13 ", "CHANNEL 14 ", "CHANNEL 15 ", "CHANNEL 16 ", "ALWAYS OFF ", "ALWAYS ON  ", "GESTURE AUX", "ERROR      "};
  if (data_to_print == 0) {
    return rssi_source_status[profile.receiver.lqi_source];
  } else {
    return rssi_source_channel[profile.receiver.aux[12]];
  }
}

const char *get_aux_status(int input) {
  static char *respond[] = {"CHANNEL 5  ", "CHANNEL 6  ", "CHANNEL 7  ", "CHANNEL 8  ", "CHANNEL 9  ", "CHANNEL 10 ", "CHANNEL 11 ", "CHANNEL 12 ", "CHANNEL 13 ", "CHANNEL 14 ", "CHANNEL 15 ", "CHANNEL 16 ", "ALWAYS OFF ", "ALWAYS ON  ", "GESTURE AUX", "ERROR      "};
  return respond[input];
}

const char *get_vtx_status(int input) {
  if (input < 0)
    return 0;
  static char *vtx_data_status[4][8] = {{"A", "B", "E", "F", "R"}, {"1", "2", "3", "4", "5", "6", "7", "8"}, {"1", "2", "3", "4"}, {"OFF", "ON ", "N/A"}};
  return vtx_data_status[input][*vtx_ptr[input]];
}

vec3_t *get_pid_term(uint8_t term) {
  switch (term) {
  case 1:
    return &profile.pid.pid_rates[profile.pid.pid_profile].kp;
  case 2:
    return &profile.pid.pid_rates[profile.pid.pid_profile].ki;
  case 3:
    return &profile.pid.pid_rates[profile.pid.pid_profile].kd;
  }
  return NULL;
}

vec3_t *get_sw_rate_term(uint8_t term) {
  return &profile_current_rates()->rate[term - 1];
}

vec3_t *get_bf_rate_term(uint8_t term) {
  return &profile_current_rates()->rate[term - 1];
}

vec3_t *get_stick_profile_term(uint8_t term) {
  switch (term) {
  case 1:
    return &profile.pid.stick_rates[profile.pid.stick_profile].accelerator;
  case 2:
    return &profile.pid.stick_rates[profile.pid.stick_profile].transition;
  }
  return NULL;
}

void osd_vector_adjust(vec3_t *pointer, uint8_t rows, uint8_t columns, uint8_t special_case, const float adjust_limit[rows * columns][2]) {
  if (osd_state.selection > columns) {
    osd_state.selection = columns; // limit osd select variable from accumulating past 3 columns of adjustable items
    osd_state.screen_phase = 1;    // repaint the screen again
  }
  if (osd_state.cursor <= rows) {
    uint8_t adjust_tracker = ((osd_state.cursor - 1) * columns) + (osd_state.selection - 1);
    if ((osd_state.selection_increase && pointer->axis[osd_state.selection - 1] < adjust_limit[adjust_tracker][1]) || (osd_state.selection_decrease && pointer->axis[osd_state.selection - 1] > adjust_limit[adjust_tracker][0])) {
      if (special_case == BF_PIDS)
        pointer->axis[osd_state.selection - 1] = adjust_rounded_float(pointer->axis[osd_state.selection - 1], bf_pids_increments[adjust_tracker]);
      if (special_case == SW_RATES)
        pointer->axis[osd_state.selection - 1] = adjust_rounded_float(pointer->axis[osd_state.selection - 1], sw_rates_increments[adjust_tracker]);
      if (special_case == ROUNDED)
        pointer->axis[osd_state.selection - 1] = adjust_rounded_float(pointer->axis[osd_state.selection - 1], rounded_increments[adjust_tracker]);
    }
    osd_state.selection_increase = 0;
    osd_state.selection_decrease = 0;
  }
  if (osd_state.cursor == rows + 1 && osd_state.selection == 1) {
    osd_save_exit();
  }
}

void osd_float_adjust(float *pointer[], uint8_t rows, uint8_t columns, const float adjust_limit[rows * columns][2], float adjust_amount) {
  if (osd_state.selection > columns) {
    osd_state.selection = columns;
    osd_state.screen_phase = 1; // repaint the screen again
  }
  if (osd_state.cursor <= rows) {
    uint8_t adjust_tracker = ((osd_state.cursor - 1) * columns) + (osd_state.selection - 1);
    if ((osd_state.selection_increase && *pointer[adjust_tracker] < adjust_limit[adjust_tracker][1]) || (osd_state.selection_decrease && *pointer[adjust_tracker] > adjust_limit[adjust_tracker][0])) {
      *pointer[adjust_tracker] = adjust_rounded_float(*pointer[adjust_tracker], adjust_amount);
    }
    osd_state.selection_increase = 0;
    osd_state.selection_decrease = 0;
  }
  if (osd_state.cursor == rows + 1 && osd_state.selection == 1) {
    osd_save_exit();
  }
}

void osd_enum_adjust(uint8_t *pointer[], uint8_t rows, const uint8_t increase_limit[]) {
  if (osd_state.selection > 1) {
    osd_state.selection = 1;    // limit osd select variable from accumulating past 1 columns of adjustable items
    osd_state.screen_phase = 1; // repaint the screen again
  }
  if (osd_state.cursor <= rows && osd_state.selection > 0) {
    uint8_t adjust_tracker = osd_state.cursor - 1;
    uint8_t i = *pointer[adjust_tracker];
    if (osd_state.selection_increase && i != increase_limit[adjust_tracker]) { // limits need to be 11 for arming, 14 for everything else on flight modes
      i++;
      *pointer[adjust_tracker] = i;
      osd_state.screen_phase = 1; // repaint the screen again
    }
    if (osd_state.selection_decrease && i != 0) { // limit is always 0 for an enum
      i--;
      *pointer[adjust_tracker] = i;
      osd_state.screen_phase = 1; // repaint the screen again
    }
    osd_state.selection_increase = 0;
    osd_state.selection_decrease = 0;
  }
  if (osd_state.cursor == rows + 1 && osd_state.selection == 1) {
    osd_save_exit();
  }
}

void osd_mixed_data_adjust(float *pointer[], uint8_t *pointer2[], uint8_t rows, uint8_t columns, const float adjust_limit[rows * columns][2], float adjust_amount, const uint8_t reboot_request[rows * columns]) {
  if (osd_state.selection > columns) {
    osd_state.selection = columns;
    osd_state.screen_phase = 1; // repaint the screen again
  }

  if (osd_state.cursor <= rows && osd_state.selection > 0) {
    uint8_t adjust_tracker = ((osd_state.cursor - 1) * columns) + (osd_state.selection - 1);

    if (*pointer[adjust_tracker] != POINTER_REDIRECT) { // POINTER_REDIRECT = -999.0 is a dummy value to indicate skipping to another data type
      if ((osd_state.selection_increase && *pointer[adjust_tracker] < adjust_limit[adjust_tracker][1]) || (osd_state.selection_decrease && *pointer[adjust_tracker] > adjust_limit[adjust_tracker][0])) {
        *pointer[adjust_tracker] = adjust_rounded_float(*pointer[adjust_tracker], adjust_amount);
      }
    } else {
      uint8_t i = *pointer2[adjust_tracker];
      if (osd_state.selection_increase && i != adjust_limit[adjust_tracker][1]) {
        i++;
        *pointer2[adjust_tracker] = i;
        osd_state.screen_phase = 1; // repaint the screen again
        if (reboot_request[adjust_tracker] == 1)
          osd_state.reboot_fc_requested = 1;
      }
      if (osd_state.selection_decrease && i != 0) { // limit is always 0 for an enum or uint8_t
        i--;
        *pointer2[adjust_tracker] = i;
        osd_state.screen_phase = 1; // repaint the screen again
        if (reboot_request[adjust_tracker] == 1)
          osd_state.reboot_fc_requested = 1;
      }
    }
    osd_state.selection_increase = 0;
    osd_state.selection_decrease = 0;
  }

  if (osd_state.cursor == rows + 1 && osd_state.selection == 1) {
    osd_save_exit();
  }
}
