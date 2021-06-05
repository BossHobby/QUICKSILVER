#include "osd_adjust.h"

#include <stdio.h>

#include "defines.h"
#include "drv_spi_max7456.h"
#include "drv_time.h"
#include "flash.h"
#include "osd_menu_maps.h"
#include "osd_render.h"
#include "profile.h"
#include "util.h"
#include "vtx.h"

extern uint8_t osd_menu_phase;
extern uint8_t osd_display_phase;
extern uint8_t osd_cursor;
extern uint8_t last_osd_cursor[6];
extern uint8_t osd_select;
extern uint8_t increase_osd_value;
extern uint8_t decrease_osd_value;
extern uint8_t reboot_fc_requested;
extern profile_t profile;
extern vtx_settings_t vtx_settings;
vtx_settings_t vtx_settings_copy;
static uint8_t vtx_buffer_populated = 0;

//**************************************************************** utility and tracking functions*********************************************************

void osd_save_exit(void) {
  osd_select = 0;
  osd_cursor = 0;
  for (uint8_t i = 0; i < 6; i++) {
    last_osd_cursor[i] = 0;
  }
  osd_display_phase = 0;

  //check if vtx settings need to be updated
  if (vtx_buffer_populated)
    vtx_settings = vtx_settings_copy;

  //check for fc reboot request
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

  if (reboot_fc_requested)
    NVIC_SystemReset();
}

uint8_t last_cursor_array_stuffer(uint8_t cursor, uint8_t add_new) { //where add_new can be either STORE_VALUE or RETURN_VALUE
  if (add_new) {
    for (int i = 5; i >= 1; i--) { //shift all the values to the right one array position
      last_osd_cursor[i] = last_osd_cursor[i - 1];
    }
    last_osd_cursor[0] = cursor; //add the most recent value to slot 0
    return 0;
  } else {
    uint8_t next_cursor_value = last_osd_cursor[0]; //remove slot 0 for return and left shift the rest over
    for (int i = 1; i <= 5; i++) {
      last_osd_cursor[i - 1] = last_osd_cursor[i];
    }
    return next_cursor_value;
  }
}

void osd_submenu_select(uint8_t *pointer, uint8_t rows, const uint8_t next_menu[]) {
  if (osd_select == 1) { //stick was pushed right to select a next menu
    osd_select = 0;      //reset the trigger
    last_cursor_array_stuffer(osd_cursor, STORE_VALUE);
    if (osd_cursor <= rows) {
      *pointer = osd_cursor - 1;                     //update profile
      osd_display_phase = next_menu[osd_cursor - 1]; //update display phase to the next menu screen
      osd_cursor = 0;                                //reset the cursor
      osd_menu_phase = 0;                            //clear the screen
    }
  }
}

void osd_select_menu_item(uint8_t rows, const uint8_t menu_map[], uint8_t main_menu) {
  if (osd_select == 1) { //main menu
    osd_select = 0;      //reset the trigger
    last_cursor_array_stuffer(osd_cursor, STORE_VALUE);
    if (osd_cursor <= rows) {
      osd_display_phase = menu_map[osd_cursor - 1];
      osd_cursor = 0;
      osd_menu_phase = 0;
    }
    if (main_menu) {
      if (osd_cursor == rows + 1)
        osd_save_exit(); //include save&exit in main menu
    }
  }
}

//populate a vtx_status_temp_buffer with current settings only once
void populate_vtx_buffer_once(void) {
  if (!vtx_buffer_populated) {
    vtx_settings_copy = vtx_settings;
    vtx_buffer_populated = 1;
  }
  return;
}

//**********************************************************encoded flash memory adjust functions**********************************************************

uint32_t get_callsign_bitmask(uint8_t input) {
  if (input == 8)
    return 0xFF;
  if (input == 16)
    return 0xFFFF;
  if (input == 24)
    return 0xFFFFFF;
  return 0;
}

void osd_encoded_adjust_callsign(void) {
  if (osd_select > 20) {
    osd_select = 20;    //limit osd select variable from accumulating past 1 columns of adjustable items
    osd_menu_phase = 1; //repaint the screen again
  }
  if (increase_osd_value) {
    profile.osd.elements[callsign_shift_index[osd_select - 1][0]] = (((profile.osd.elements[callsign_shift_index[osd_select - 1][0]] >> callsign_shift_index[osd_select - 1][1]) + 1) << callsign_shift_index[osd_select - 1][1]) + (profile.osd.elements[callsign_shift_index[osd_select - 1][0]] & get_callsign_bitmask(callsign_shift_index[osd_select - 1][1]));
    osd_menu_phase = 1; //repaint the screen again
  }
  if (decrease_osd_value) {
    profile.osd.elements[callsign_shift_index[osd_select - 1][0]] = (((profile.osd.elements[callsign_shift_index[osd_select - 1][0]] >> callsign_shift_index[osd_select - 1][1]) - 1) << callsign_shift_index[osd_select - 1][1]) + (profile.osd.elements[callsign_shift_index[osd_select - 1][0]] & get_callsign_bitmask(callsign_shift_index[osd_select - 1][1]));
    osd_menu_phase = 1; //repaint the screen again
  }
  increase_osd_value = 0;
  decrease_osd_value = 0;
  if (osd_cursor == 2) {
    if (osd_select == 1) {
      osd_save_exit();
    }
  }
}

void osd_encoded_adjust(uint32_t *pointer, uint8_t rows, uint8_t columns, uint8_t status) {
  if (osd_select > columns) {
    osd_select = columns; //limit osd select variable from accumulating past 1 columns of adjustable items
    osd_menu_phase = 1;   //repaint the screen again
  }
  if (osd_cursor <= rows) {
    switch (status) {
    case 0: //adjust active or inactive element
      if (increase_osd_value && osd_decode(*pointer, status) == 0x00) {
        *pointer = *pointer + 1;
        osd_menu_phase = 1; //repaint the screen again
      }
      if (decrease_osd_value && osd_decode(*pointer, status) == 0x01) {
        *pointer = *pointer - 1;
        osd_menu_phase = 1; //repaint the screen again
      }
      break;
    case 1:                                                             //adjust TEXT or INVERT
      if (increase_osd_value && osd_decode(*pointer, status) == TEXT) { //increase requested and currently on TEXT
        *pointer = *pointer | (0x02);                                   //flip the 2nd bit on
        osd_menu_phase = 1;                                             //repaint the screen again
      }
      if (decrease_osd_value && osd_decode(*pointer, status) == INVERT) { //decrease requested and currently on INVERT
        *pointer = *pointer ^ (0x02);                                     //flip the 2nd bit off
        osd_menu_phase = 1;                                               //repaint the screen again
      }
      break;
    case 2: //adjust positionX
      if (increase_osd_value && osd_decode(*pointer, status) != 30) {
        *pointer = (((*pointer >> 2) + 1) << 2) + (*pointer & 0x03);
        osd_menu_phase = 1; //repaint the screen again
      }
      if (decrease_osd_value && osd_decode(*pointer, status) != 0) {
        *pointer = (((*pointer >> 2) - 1) << 2) + (*pointer & 0x03);
        osd_menu_phase = 1; //repaint the screen again
      }
      break;
    case 3: //adjust positionY
      if (increase_osd_value && osd_decode(*pointer, status) != 15) {
        *pointer = (((*pointer >> 7) + 1) << 7) + (*pointer & 0x7F);
        osd_menu_phase = 1; //repaint the screen again
      }
      if (decrease_osd_value && osd_decode(*pointer, status) != 0) {
        *pointer = (((*pointer >> 7) - 1) << 7) + (*pointer & 0x7F);
        osd_menu_phase = 1; //repaint the screen again
      }
      break;
    }
    increase_osd_value = 0;
    decrease_osd_value = 0;
  }
  if (osd_cursor == rows + 1) {
    if (osd_select == 1) {
      osd_save_exit();
    }
  }
}

//************************************************************profile variable adjust functions***********************************************************

float adjust_rounded_float(float input, float adjust_amount) {
  const float value = (int)(input * 100.0f + (input <= 0 ? -0.5f : 0.5f));
  if (increase_osd_value) {
    increase_osd_value = 0;
    osd_menu_phase = 1; //repaint the screen again
    return (float)(value + (100.0f * adjust_amount)) / 100.0f;
  }
  if (decrease_osd_value) {
    decrease_osd_value = 0;
    osd_menu_phase = 1; //repaint the screen again
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
  switch (term) {
  case 1:
    return &profile.rate.silverware.max_rate;
  case 2:
    return &profile.rate.silverware.acro_expo;
  case 3:
    return &profile.rate.silverware.angle_expo;
  }
  return NULL;
}

vec3_t *get_bf_rate_term(uint8_t term) {
  switch (term) {
  case 1:
    return &profile.rate.betaflight.rc_rate;
  case 2:
    return &profile.rate.betaflight.super_rate;
  case 3:
    return &profile.rate.betaflight.expo;
  }
  return NULL;
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
  if (osd_select > columns) {
    osd_select = columns; //limit osd select variable from accumulating past 3 columns of adjustable items
    osd_menu_phase = 1;   //repaint the screen again
  }
  if (osd_cursor <= rows) {
    uint8_t adjust_tracker = ((osd_cursor - 1) * columns) + (osd_select - 1);
    if ((increase_osd_value && pointer->axis[osd_select - 1] < adjust_limit[adjust_tracker][1]) || (decrease_osd_value && pointer->axis[osd_select - 1] > adjust_limit[adjust_tracker][0])) {
      if (special_case == BF_PIDS)
        pointer->axis[osd_select - 1] = adjust_rounded_float(pointer->axis[osd_select - 1], bf_pids_increments[adjust_tracker]);
      if (special_case == SW_RATES)
        pointer->axis[osd_select - 1] = adjust_rounded_float(pointer->axis[osd_select - 1], sw_rates_increments[adjust_tracker]);
      if (special_case == ROUNDED)
        pointer->axis[osd_select - 1] = adjust_rounded_float(pointer->axis[osd_select - 1], rounded_increments[adjust_tracker]);
    }
    increase_osd_value = 0;
    decrease_osd_value = 0;
  }
  if (osd_cursor == rows + 1) {
    if (osd_select == 1) {
      osd_save_exit();
    }
  }
}

void osd_float_adjust(float *pointer[], uint8_t rows, uint8_t columns, const float adjust_limit[rows * columns][2], float adjust_amount) {
  if (osd_select > columns) {
    osd_select = columns;
    osd_menu_phase = 1; //repaint the screen again
  }
  if (osd_cursor <= rows) {
    uint8_t adjust_tracker = ((osd_cursor - 1) * columns) + (osd_select - 1);
    if ((increase_osd_value && *pointer[adjust_tracker] < adjust_limit[adjust_tracker][1]) || (decrease_osd_value && *pointer[adjust_tracker] > adjust_limit[adjust_tracker][0])) {
      *pointer[adjust_tracker] = adjust_rounded_float(*pointer[adjust_tracker], adjust_amount);
    }
    increase_osd_value = 0;
    decrease_osd_value = 0;
  }
  if (osd_cursor == rows + 1) {
    if (osd_select == 1) {
      osd_save_exit();
    }
  }
}

void osd_enum_adjust(uint8_t *pointer[], uint8_t rows, const uint8_t increase_limit[]) {
  if (osd_select > 1) {
    osd_select = 1;     //limit osd select variable from accumulating past 1 columns of adjustable items
    osd_menu_phase = 1; //repaint the screen again
  }
  if (osd_cursor <= rows && osd_select > 0) {
    uint8_t adjust_tracker = osd_cursor - 1;
    uint8_t i = *pointer[adjust_tracker];
    if (increase_osd_value && i != increase_limit[adjust_tracker]) { //limits need to be 11 for arming, 14 for everything else on flight modes
      i++;
      *pointer[adjust_tracker] = i;
      osd_menu_phase = 1; //repaint the screen again
    }
    if (decrease_osd_value && i != 0) { //limit is always 0 for an enum
      i--;
      *pointer[adjust_tracker] = i;
      osd_menu_phase = 1; //repaint the screen again
    }
    increase_osd_value = 0;
    decrease_osd_value = 0;
  }
  if (osd_cursor == rows + 1) {
    if (osd_select == 1) {
      osd_save_exit();
    }
  }
}

void osd_mixed_data_adjust(float *pointer[], uint8_t *pointer2[], uint8_t rows, uint8_t columns, const float adjust_limit[rows * columns][2], float adjust_amount, const uint8_t reboot_request[rows * columns]) {
  if (osd_select > columns) {
    osd_select = columns;
    osd_menu_phase = 1; //repaint the screen again
  }

  if (osd_cursor <= rows && osd_select > 0) {
    uint8_t adjust_tracker = ((osd_cursor - 1) * columns) + (osd_select - 1);

    if (*pointer[adjust_tracker] != POINTER_REDIRECT) { //POINTER_REDIRECT = -999.0 is a dummy value to indicate skipping to another data type
      if ((increase_osd_value && *pointer[adjust_tracker] < adjust_limit[adjust_tracker][1]) || (decrease_osd_value && *pointer[adjust_tracker] > adjust_limit[adjust_tracker][0])) {
        *pointer[adjust_tracker] = adjust_rounded_float(*pointer[adjust_tracker], adjust_amount);
      }
    } else {
      uint8_t i = *pointer2[adjust_tracker];
      if (increase_osd_value && i != adjust_limit[adjust_tracker][1]) {
        i++;
        *pointer2[adjust_tracker] = i;
        osd_menu_phase = 1; //repaint the screen again
        if (reboot_request[adjust_tracker] == 1)
          reboot_fc_requested = 1;
      }
      if (decrease_osd_value && i != 0) { //limit is always 0 for an enum or uint8_t
        i--;
        *pointer2[adjust_tracker] = i;
        osd_menu_phase = 1; //repaint the screen again
        if (reboot_request[adjust_tracker] == 1)
          reboot_fc_requested = 1;
      }
    }
    increase_osd_value = 0;
    decrease_osd_value = 0;
  }

  if (osd_cursor == rows + 1) {
    if (osd_select == 1) {
      osd_save_exit();
    }
  }
}
