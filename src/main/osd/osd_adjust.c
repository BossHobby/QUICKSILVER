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
  osd_state.selection = 0;
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

// populate a vtx_status_temp_buffer with current settings only once
void populate_vtx_buffer_once() {
  if (!vtx_buffer_populated) {
    vtx_settings_copy = vtx_settings;
    vtx_buffer_populated = 1;
  }
  return;
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
