#include "osd_render.h"

#include "control.h"
#include "debug.h"
#include "drv_spi_max7456.h"
#include "drv_time.h"
#include "filter.h"
#include "float.h"
#include "osd_adjust.h"
#include "osd_menu_maps.h"
#include "profile.h"
#include "project.h"
#include "rx.h"
#include "stdio.h"
#include "string.h"
#include "util.h"
#include "flash.h"
#include "vtx.h"

#ifdef ENABLE_OSD

void osd_init(void) {
  spi_max7456_init(); //init spi
  max7456_init();     //init the max chip
  osd_intro();        //print the splash screen
}

//************************************************************************************************************************************************************************************
//																					FLASH MEMORY
//************************************************************************************************************************************************************************************
/*screen elements characteristics written like registers in a 32bit binany number
except callsign which will take 6 addresses.  callsign bit 1 will be enable/disable,
bit 2 will be text/invert, and the remaining 5 addresses will be 4 characters each.  6 total addresses
will allow callsign text to fill 20 characters
BIT
0			-		0 is display element inactive , 1 is display element active
1			-		0 is TEXT, 1 is INVERT
2:6		-		the X screen position (column)
7:10	-		the Y screen position	(row)
11:15	-		not currently used
16:31	-		available for two binary ascii characters but not currently used
*/

//Flash Variables - 32bit					# of osd elements and flash memory start position in defines.h
extern profile_t profile;
extern vtx_settings_t vtx_settings;
extern vtx_settings_t vtx_settings_copy;

//pointers to flash variable array
unsigned long *callsign1 = profile.osd.elements;
unsigned long *callsign2 = (profile.osd.elements + 1);
unsigned long *callsign3 = (profile.osd.elements + 2);
unsigned long *callsign4 = (profile.osd.elements + 3);
unsigned long *callsign5 = (profile.osd.elements + 4);
unsigned long *callsign6 = (profile.osd.elements + 5);
unsigned long *fuelgauge_volts = (profile.osd.elements + 6);
unsigned long *filtered_volts = (profile.osd.elements + 7);
unsigned long *gyro_degrees = (profile.osd.elements + 8);
unsigned long *flight_mode = (profile.osd.elements + 9);
unsigned long *rssi = (profile.osd.elements + 10);
unsigned long *stopwatch = (profile.osd.elements + 11);
unsigned long *arm_disarm = (profile.osd.elements + 12);
unsigned long *osd_throttle = (profile.osd.elements + 13);
unsigned long *osd_vtx = (profile.osd.elements + 14);

#define ACTIVE 0
#define ATTRIBUTE 1
#define POSITIONX 2
#define POSITIONY 3

uint8_t osd_decode(uint32_t element, uint8_t status) {
  switch (status) {
  case 0:
    return (element & 0x01);
    break;
  case 1:
    if (((element >> 1) & 0x01) == 0x01)
      return INVERT;
    else
      return TEXT;
    break;
  case 2:
    return ((element >> 2) & 0x1F);
    break;
  case 3:
    return ((element >> 7) & 0x0F);
    break;
  }
  return 0;
}

const char *get_position_string(int input) {
  static char *respond[] = {" 0", " 1", " 2", " 3", " 4", " 5", " 6", " 7", " 8", " 9", "10", "11", "12", "13", "14", "15", "16", "17", "18", "19", "20", "21", "22", "23", "24", "25", "26", "27", "28", "29", "30", "ERROR"};
  return respond[input];
}

const char *get_decode_element_string(uint32_t input, uint8_t status) {
  switch (status) {
  case 0: //ACTIVE
    if (osd_decode(input, status))
      return "ACTIVE  ";
    else
      return "INACTIVE";
    break;
  case 1: //ATTRIBUTE
    if (osd_decode(input, status) == INVERT)
      return "INVERT";
    else
      return "NORMAL";
    break;
  case 2:
    return get_position_string(osd_decode(input, status));
    break;
  case 3:
    return get_position_string(osd_decode(input, status));
    break;
  }
  return 0;
}

//************************************************************************************************************************************************************************************
//																					STATE VARIABLES
//************************************************************************************************************************************************************************************
// case & state variables for switch logic and profile adjustments
uint8_t osd_display_phase = 2;
uint8_t last_display_phase;
uint8_t osd_wizard_phase = 0;
uint8_t osd_menu_phase = 0;
uint8_t osd_display_element = 0;
uint8_t display_trigger = 0;
uint8_t last_lowbatt_state = 2;
uint8_t last_lowbatt_state2 = 2;
uint8_t last_lowbatt_state3 = 2;
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

void osd_display_reset(void) {
  osd_wizard_phase = 0;    //reset the wizard
  osd_menu_phase = 0;      //reset menu to to main menu
  osd_display_phase = 2;   //jump to regular osd display next loop
  osd_display_element = 0; //start with first screen element
  last_lowbatt_state = 2;  //reset last lowbatt comparator
  last_lowbatt_state2 = 2;
  last_lowbatt_state3 = 2;
}

uint8_t user_select(uint8_t active_elements, uint8_t total_elements) {
  if (osd_cursor < 1)
    osd_cursor = 1;
  if (osd_cursor > active_elements)
    osd_cursor = active_elements;
  uint8_t inactive_elements = total_elements - active_elements;
  if (osd_menu_phase == 1)
    return INVERT;
  if (osd_menu_phase > 1 && osd_menu_phase <= inactive_elements)
    return TEXT;
  if (osd_cursor == (osd_menu_phase - inactive_elements) && osd_select == 0) {
    return INVERT;
  } else {
    return TEXT;
  }
}

uint8_t grid_selection(uint8_t element, uint8_t row) {
  if (osd_select == element && osd_cursor == row) {
    return INVERT;
  } else {
    return TEXT;
  }
}

//************************************************************************************************************************************************************************************
//																					PRINT FUNCTIONS
//************************************************************************************************************************************************************************************

void print_osd_callsign_adjustable(uint8_t string_element_qty, uint8_t data_element_qty, const uint8_t grid[data_element_qty][2], const uint8_t print_position[data_element_qty][2]) {
  if (osd_menu_phase <= string_element_qty)
    return;
  if (osd_menu_phase > string_element_qty + data_element_qty)
    return;
  static uint8_t skip_loop = 0;
  if (osd_menu_phase == string_element_qty + 1 && skip_loop == 0) { //skip a loop to prevent dma collision with previous print function
    skip_loop++;
    return;
  }
  skip_loop = 0;
  uint8_t index = osd_menu_phase - string_element_qty - 1;
  uint8_t character[] = {(profile.osd.elements[callsign_shift_index[index][0]] >> callsign_shift_index[index][1]) & 0xFF};
  osd_print_data(character, 1, grid_selection(grid[index][0], grid[index][1]), print_position[index][0], print_position[index][1]);
  osd_menu_phase++;
}

uint8_t print_osd_callsign(void) {
  static uint8_t index = 0;
  static uint8_t callsign_length = 0;
  if (index == 0) {
    for (uint8_t i = 19; i >= 0; i--) {
      uint8_t last_user_input = (profile.osd.elements[callsign_shift_index[i][0]] >> callsign_shift_index[i][1]) & 0xFF;
      if (last_user_input != 0x3F) {
        callsign_length = i + 1;
        index++;
        return 0;
      }
    }
    callsign_length = 0; //for loop found callsign is all spaces so set length to 0 and return
    index = 0;
    return 1;
  }

  if (index <= callsign_length && callsign_length > 0) {
    uint8_t character[] = {(profile.osd.elements[callsign_shift_index[index - 1][0]] >> callsign_shift_index[index - 1][1]) & 0xFF};
    osd_print_data(character, 1, osd_decode(*callsign1, ATTRIBUTE), osd_decode(*callsign1, POSITIONX) + index - 1, osd_decode(*callsign1, POSITIONY));
    index++;
    return 0;
  }
  index = 0;
  return 1;
}

uint8_t print_osd_flightmode(void) {
  const char flightmode_labels[5][21] = {{"   ACRO   "}, {"  LEVEL   "}, {" RACEMODE "}, {" HORIZON  "}, {"RM HORIZON"}};
  static uint8_t index = 0;
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
  if (index < 10) {
    uint8_t character[] = {flightmode_labels[flightmode][index]};
    osd_print_data(character, 1, osd_decode(*flight_mode, ATTRIBUTE), osd_decode(*flight_mode, POSITIONX) + index, osd_decode(*flight_mode, POSITIONY));
    index++;
    return 0;
  }
  index = 0;
  return 1;
}

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
} status_label;

uint8_t print_status(uint8_t delay_clear, uint8_t label) { //3 stage return - 0 = stick and hold, 1 = move on but come back to clear, 2 = status print done
  const char system_status_labels[12][21] = {{"               "}, {" **DISARMED**  "}, {"  **ARMED**    "}, {" STICK BOOST 1 "}, {" STICK BOOST 2 "}, {" **FAILSAFE**  "}, {"THROTTLE SAFETY"}, {" ARMING SAFETY "}, {"**LOW BATTERY**"}, {"**MOTOR TEST** "}, {"  **TURTLE**   "}, {" **LOOPTIME**  "}};
  static uint8_t last_label;
  static uint8_t index;
  static uint8_t delay_counter = 25;
  if (last_label != label) { //critical to reset indexers if a print sequence is interrupted by a new request
    index = 0;
    delay_counter = 25;
  }
  uint8_t clear[1] = {32}; //prints a space
  uint8_t character[] = {system_status_labels[label][index]};
  last_label = label;
  if (index < 15) {
    osd_print_data(character, 1, osd_decode(*arm_disarm, ATTRIBUTE) | BLINK, osd_decode(*arm_disarm, POSITIONX) + index, osd_decode(*arm_disarm, POSITIONY));
    index++;
    return 0;
  } else {
    if (delay_clear) {
      if (!delay_counter) {
        if (index < 30) {
          osd_print_data(clear, 1, osd_decode(*arm_disarm, ATTRIBUTE), osd_decode(*arm_disarm, POSITIONX) + (index - 15), osd_decode(*arm_disarm, POSITIONY));
          index++;
          return 0;
        } else {
          delay_counter = 25;
          index = 0;
          return 2;
        }
      } else {
        delay_counter--;
        return 1;
      }
    } else {
      index = 0;
      return 2;
    }
  }
  return 2;
}

#define TEMP 1
#define HOLD 0
typedef struct {
  uint8_t loop_warning;
  uint8_t aux_boost;
  uint8_t aux_motor_test;
  control_flags_t flags;
} sys_status_t;
uint8_t print_osd_system_status(void) {
  static uint8_t ready = 0;
  uint8_t print_stage = 2; // 0 makes the main osd function stick and non zero lets it pass on
  static sys_status_t last_sys_status;
  static sys_status_t printing = {
      .loop_warning = 0,
      .aux_boost = 0,
      .aux_motor_test = 0,
      .flags = {0}};
  extern uint8_t looptime_warning;
  //things happen here
  if (ready) {
    if ((flags.arm_state != last_sys_status.flags.arm_state) || printing.flags.arm_state) {
      last_sys_status.flags.arm_state = flags.arm_state;
      printing.flags.arm_state = 1;
      if (flags.arm_state)
        print_stage = print_status(TEMP, ARM);
      else
        print_stage = print_status(TEMP, DISARM);
      if (print_stage == 2)
        printing.flags.arm_state = 0;
      return print_stage;
    }
    if (((looptime_warning != last_sys_status.loop_warning) || printing.loop_warning) && (state.looptime_autodetect != LOOPTIME_8K)) { //mute warnings till we are on the edge of 4k->2k
      last_sys_status.loop_warning = looptime_warning;
      printing.loop_warning = 1;
      print_stage = print_status(TEMP, LOOP);
      if (print_stage == 2)
        printing.loop_warning = 0;
      return print_stage;
    }
    if (rx_aux_on(AUX_STICK_BOOST_PROFILE) != last_sys_status.aux_boost || printing.aux_boost) {
      last_sys_status.aux_boost = rx_aux_on(AUX_STICK_BOOST_PROFILE);
      printing.aux_boost = 1;
      if (rx_aux_on(AUX_STICK_BOOST_PROFILE))
        print_stage = print_status(TEMP, BOOST_2);
      else
        print_stage = print_status(TEMP, BOOST_1);
      if (print_stage == 2)
        printing.aux_boost = 0;
      return print_stage;
    }
    if (flags.failsafe != last_sys_status.flags.failsafe || printing.flags.failsafe) {
      last_sys_status.flags.failsafe = flags.failsafe;
      printing.flags.failsafe = 1;
      if (flags.failsafe)
        print_stage = print_status(HOLD, FAILSAFE);
      else
        print_stage = print_status(HOLD, CLEAR);
      if (print_stage == 2)
        printing.flags.failsafe = 0;
      return print_stage;
    }
    if ((flags.arm_safety && !flags.failsafe) || printing.flags.arm_safety) {
      printing.flags.arm_safety = 1;
      if (flags.arm_safety) {
        print_stage = print_status(HOLD, ARM_SFTY);
      } else {
        print_stage = print_status(HOLD, CLEAR);
        if (print_stage == 2)
          printing.flags.arm_safety = 0;
      }
      return print_stage;
    }
    if ((flags.throttle_safety && !flags.arm_safety) || printing.flags.throttle_safety) {
      printing.flags.throttle_safety = 1;
      if (flags.throttle_safety) {
        print_stage = print_status(HOLD, THRTL_SFTY);
      } else {
        print_stage = print_status(HOLD, CLEAR);
        if (print_stage == 2)
          printing.flags.throttle_safety = 0;
      }
      return print_stage;
    }
    if ((flags.lowbatt != last_sys_status.flags.lowbatt && !flags.arm_safety && !flags.throttle_safety && !flags.failsafe) || printing.flags.lowbatt) {
      last_sys_status.flags.lowbatt = flags.lowbatt;
      printing.flags.lowbatt = 1;
      if (flags.lowbatt)
        print_stage = print_status(HOLD, LOW_BAT);
      else
        print_stage = print_status(HOLD, CLEAR);
      if (print_stage == 2)
        printing.flags.lowbatt = 0;
      return print_stage;
    }
    if ((rx_aux_on(AUX_MOTORS_TO_THROTTLE_MODE) && !flags.arm_safety && !flags.throttle_safety && !flags.failsafe) || (printing.aux_motor_test && !flags.arm_safety && !flags.throttle_safety && !flags.failsafe)) {
      printing.aux_motor_test = 1;
      if (rx_aux_on(AUX_MOTORS_TO_THROTTLE_MODE)) {
        print_stage = print_status(HOLD, MOTOR_TEST);
      } else {
        print_stage = print_status(HOLD, CLEAR);
        if (print_stage == 2)
          printing.aux_motor_test = 0;
      }
      return print_stage;
    }
    if ((flags.turtle && !flags.arm_safety && !flags.throttle_safety && !flags.failsafe) || (printing.flags.turtle && !flags.arm_safety && !flags.throttle_safety && !flags.failsafe)) {
      printing.flags.turtle = 1;
      if (flags.turtle) {
        print_stage = print_status(HOLD, TURTL);
      } else {
        print_stage = print_status(HOLD, CLEAR);
        if (print_stage == 2)
          printing.flags.turtle = 0;
      }
      return print_stage;
    }
  }
  if (ready == 0) {
    last_sys_status.loop_warning = looptime_warning;
    last_sys_status.aux_boost = rx_aux_on(AUX_STICK_BOOST_PROFILE);
    last_sys_status.aux_motor_test = rx_aux_on(AUX_MOTORS_TO_THROTTLE_MODE);
    last_sys_status.flags = flags;
    ready = 1;
  }
  return ready;
}

void print_osd_rssi(void) {
  static float rx_rssi_filt;
  uint8_t osd_rssi[5];
  if (flags.failsafe)
    state.rx_rssi = 0.0f;
  lpf(&rx_rssi_filt, state.rx_rssi, FILTERCALC(state.looptime * 1e6f * 133.0f, 2e6f)); //2 second filtertime and 15hz refresh rate @4k, 30hz@ 8k loop
  fast_fprint(osd_rssi, 5, (rx_rssi_filt - 0.5f), 0);
  osd_rssi[4] = 1;
  osd_print_data(osd_rssi, 5, osd_decode(*rssi, ATTRIBUTE), osd_decode(*rssi, POSITIONX), osd_decode(*rssi, POSITIONY));
}

void print_osd_menu_strings(uint8_t string_element_qty, uint8_t active_element_qty, const char element_names[string_element_qty][21], const uint8_t print_position[string_element_qty][2]) {
  if (osd_menu_phase > string_element_qty)
    return;
  if (osd_menu_phase == 0) {
    if (osd_runtime_screen_clear())
      osd_menu_phase++;
    return;
  }
  osd_print(element_names[osd_menu_phase - 1], user_select(active_element_qty, string_element_qty), print_position[osd_menu_phase - 1][0], print_position[osd_menu_phase - 1][1]);
  osd_menu_phase++;
}

void print_osd_adjustable_enums(uint8_t string_element_qty, uint8_t data_element_qty, const char data_to_print[21], const uint8_t grid[data_element_qty][2], const uint8_t print_position[data_element_qty][2]) {
  if (osd_menu_phase <= string_element_qty)
    return;
  if (osd_menu_phase > string_element_qty + data_element_qty)
    return;
  static uint8_t skip_loop = 0;
  if (osd_menu_phase == string_element_qty + 1 && skip_loop == 0) { //skip a loop to prevent dma collision with previous print function
    skip_loop++;
    return;
  }
  skip_loop = 0;
  uint8_t index = osd_menu_phase - string_element_qty - 1;
  osd_print(data_to_print, grid_selection(grid[index][0], grid[index][1]), print_position[index][0], print_position[index][1]);
  osd_menu_phase++;
}

void print_osd_adjustable_vectors(uint8_t menu_type, uint8_t string_element_qty, uint8_t data_element_qty, vec3_t *pointer, const uint8_t data_index[data_element_qty][2], const uint8_t grid[data_element_qty][2], const uint8_t print_position[data_element_qty][2]) {
  if (osd_menu_phase <= string_element_qty)
    return;
  if (osd_menu_phase > string_element_qty + data_element_qty)
    return;
  static uint8_t skip_loop = 0;
  if (osd_menu_phase == string_element_qty + 1 && skip_loop == 0) { //skip a loop to prevent dma collision with previous print function
    skip_loop++;
    return;
  }
  skip_loop = 0;
  uint8_t index = osd_menu_phase - string_element_qty - 1;
  uint8_t data_buffer[5];
  if (menu_type == BF_PIDS)
    fast_fprint(data_buffer, 5, pointer->axis[data_index[index][1]], 0);
  if (menu_type == SW_RATES) {
    if (index < 3)
      fast_fprint(data_buffer, 5, pointer->axis[data_index[index][1]], 0);
    else
      fast_fprint(data_buffer, 5, pointer->axis[data_index[index][1]] + FLT_EPSILON, 2);
  }
  if (menu_type == ROUNDED)
    fast_fprint(data_buffer, 5, pointer->axis[data_index[index][1]] + FLT_EPSILON, 2);
  osd_print_data(data_buffer, 5, grid_selection(grid[index][0], grid[index][1]), print_position[index][0], print_position[index][1]);
  osd_menu_phase++;
}

void print_osd_adjustable_float(uint8_t string_element_qty, uint8_t data_element_qty, float *pointer[], const uint8_t grid[data_element_qty][2], const uint8_t print_position[data_element_qty][2], uint8_t precision) {
  if (osd_menu_phase <= string_element_qty)
    return;
  if (osd_menu_phase > string_element_qty + data_element_qty)
    return;
  static uint8_t skip_loop = 0;
  if (osd_menu_phase == string_element_qty + 1 && skip_loop == 0) { //skip a loop to prevent dma collision with previous print function
    skip_loop++;
    return;
  }
  skip_loop = 0;
  uint8_t data_buffer[5];
  uint8_t index = osd_menu_phase - string_element_qty - 1;
  fast_fprint(data_buffer, 5, *pointer[index] + FLT_EPSILON, precision);
  osd_print_data(data_buffer, 5, grid_selection(grid[index][0], grid[index][1]), print_position[index][0], print_position[index][1]);
  osd_menu_phase++;
}

void print_osd_mixed_data(uint8_t string_element_qty, uint8_t data_element_qty, float *pointer[], uint8_t *pointer2[], const char filtertype_labels[5][21], const uint8_t grid[data_element_qty][2], const uint8_t print_position[data_element_qty][2], uint8_t precision) {
  if (osd_menu_phase <= string_element_qty)
    return;
  if (osd_menu_phase > string_element_qty + data_element_qty)
    return;
  static uint8_t skip_loop = 0;
  if (osd_menu_phase == string_element_qty + 1 && skip_loop == 0) { //skip a loop to prevent dma collision with previous print function
    skip_loop++;
    return;
  }
  skip_loop = 0;
  uint8_t data_buffer[5];
  uint8_t index = osd_menu_phase - string_element_qty - 1;
  if (*pointer[index] != POINTER_REDIRECT) {
    fast_fprint(data_buffer, 5, *pointer[index] + FLT_EPSILON, precision);
    osd_print_data(data_buffer, 5, grid_selection(grid[index][0], grid[index][1]), print_position[index][0], print_position[index][1]);
  } else {
    osd_print(filtertype_labels[*pointer2[index]], grid_selection(grid[index][0], grid[index][1]), print_position[index][0], print_position[index][1]);
  }
  osd_menu_phase++;
}

//************************************************************************************************************************************************************************************
//************************************************************************************************************************************************************************************
//																				MAIN OSD DISPLAY FUNCTION
//************************************************************************************************************************************************************************************
//************************************************************************************************************************************************************************************

void osd_display(void) {

  //first check if video signal autodetect needs to run - run if necessary
  extern uint8_t lastsystem; //initialized at 99 for none then becomes 0 or 1 for ntsc/pal
  if (lastsystem > 1)        //if no camera was detected at boot up
  {
    osd_checksystem(); //try to detect camera
    if (lastsystem < 2) {
      osd_display_reset(); //camera has been detected while in the main loop and screen has been cleared again - reset screen cases
    }
  }

  //************OSD MENU DISPLAY ROUTINES HERE*************
  switch (osd_display_phase) //phase starts at 2, RRR gesture subtracts 1 to enter the menu, RRR again or DDD subtracts 1 to clear the screen and return to regular display
  {
  case 0: //osd screen clears, resets to regular display, and resets wizard and menu starting points
    if (osd_runtime_screen_clear())
      osd_display_reset();
    break; //screen has been cleared for this loop - break out of display function

  case 1:                //osd menu is active
    last_display_phase = 2;
    print_osd_menu_strings(10, 9, main_menu_labels, main_menu_positions);
    if (osd_menu_phase == 11)
      osd_select_menu_item(8, main_menu_map, MAIN_MENU);
    break; //osd menu has been displayed for this loop	- break out of display function

  case 2: //regular osd display

    switch (osd_display_element) {
    case 0:
      if (osd_decode(*callsign1, ACTIVE)) {
        uint8_t callsign_done = print_osd_callsign();
        if (callsign_done)
          osd_display_element++;
      } else {
        osd_display_element++;
      }
      break; //screen has been displayed for this loop - break out of display function

    case 1:
      if (osd_decode(*fuelgauge_volts, ACTIVE)) {
        uint8_t osd_fuelgauge_volts[5];
        fast_fprint(osd_fuelgauge_volts, 4, state.vbatt_comp, 1);
        osd_fuelgauge_volts[4] = 'V';
        osd_print_data(osd_fuelgauge_volts, 5, osd_decode(*fuelgauge_volts, ATTRIBUTE), osd_decode(*fuelgauge_volts, POSITIONX) + 3, osd_decode(*fuelgauge_volts, POSITIONY));
      }
      osd_display_element++;
      break;

    case 2:
      if (osd_decode(*fuelgauge_volts, ACTIVE)) {
        if (flags.lowbatt != last_lowbatt_state) {
          uint8_t osd_cellcount[2] = {state.lipo_cell_count + 48, 'S'};
          if (!flags.lowbatt) {
            osd_print_data(osd_cellcount, 2, osd_decode(*fuelgauge_volts, ATTRIBUTE), osd_decode(*fuelgauge_volts, POSITIONX), osd_decode(*fuelgauge_volts, POSITIONY));
          } else {
            osd_print_data(osd_cellcount, 2, BLINK | INVERT, osd_decode(*fuelgauge_volts, POSITIONX), osd_decode(*fuelgauge_volts, POSITIONY));
          }
          last_lowbatt_state = flags.lowbatt;
        }
      }
      osd_display_element++;
      break;

    case 3:
      if (osd_decode(*filtered_volts, ACTIVE)) {
        uint8_t osd_filtered_volts[5];
        fast_fprint(osd_filtered_volts, 4, state.vbattfilt_corr, 1);
        osd_filtered_volts[4] = 'V';
        osd_print_data(osd_filtered_volts, 5, osd_decode(*filtered_volts, ATTRIBUTE), osd_decode(*filtered_volts, POSITIONX) + 3, osd_decode(*filtered_volts, POSITIONY));
      }
      osd_display_element++;
      break;

    case 4:
      if (osd_decode(*filtered_volts, ACTIVE)) {
        if (flags.lowbatt != last_lowbatt_state2) {
          uint8_t osd_cellcount2[2] = {state.lipo_cell_count + 48, 'S'};
          if (!flags.lowbatt) {
            osd_print_data(osd_cellcount2, 2, osd_decode(*filtered_volts, ATTRIBUTE), osd_decode(*filtered_volts, POSITIONX), osd_decode(*filtered_volts, POSITIONY));
          } else {
            osd_print_data(osd_cellcount2, 2, BLINK | INVERT, osd_decode(*filtered_volts, POSITIONX), osd_decode(*filtered_volts, POSITIONY));
          }
          last_lowbatt_state2 = flags.lowbatt;
        }
      }
      osd_display_element++;
      break;

    case 5:
      if (osd_decode(*gyro_degrees, ACTIVE)) {
        uint8_t osd_gyro_temp[5];
        fast_fprint(osd_gyro_temp, 5, state.gyro_temp, 0);
        osd_gyro_temp[4] = 14; //degrees C
        osd_print_data(osd_gyro_temp, 5, osd_decode(*gyro_degrees, ATTRIBUTE), osd_decode(*gyro_degrees, POSITIONX) + 3, osd_decode(*gyro_degrees, POSITIONY));
      }
      osd_display_element++;
      break;

    case 6:
      if (osd_decode(*stopwatch, ACTIVE)) {
        uint8_t osd_stopwatch[5];
        fast_fprint(osd_stopwatch, 5, state.uptime, 0);
        osd_stopwatch[4] = 112; //Z+23 is fly hr
        osd_print_data(osd_stopwatch, 5, osd_decode(*stopwatch, ATTRIBUTE), osd_decode(*stopwatch, POSITIONX), osd_decode(*stopwatch, POSITIONY));
      }
      osd_display_element++;
      break;

    case 7:
      if (osd_decode(*flight_mode, ACTIVE)) {
        uint8_t flightmode_done = print_osd_flightmode();
        if (flightmode_done)
          osd_display_element++;
      } else {
        osd_display_element++;
      }
      break;

    case 8:
      if (osd_decode(*osd_throttle, ACTIVE)) {
        uint8_t osd_throttle_value[5];
        fast_fprint(osd_throttle_value, 5, (state.throttle * 100.0f), 0);
        osd_throttle_value[4] = 4;
        osd_print_data(osd_throttle_value, 5, osd_decode(*osd_throttle, ATTRIBUTE), osd_decode(*osd_throttle, POSITIONX), osd_decode(*osd_throttle, POSITIONY));
      }
      osd_display_element++;
      break;

    case 9:
      if (osd_decode(*arm_disarm, ACTIVE)) {
        uint8_t system_status_done = print_osd_system_status();
        if (system_status_done)
          osd_display_element++;
      } else {
        osd_display_element++;
      }
      break;

    case 10:
      if (osd_decode(*rssi, ACTIVE)) {
        print_osd_rssi();
      }
      osd_display_element++;
      break;

    case 11:
      if (osd_decode(*osd_vtx, ACTIVE) && vtx_settings.detected) {
        uint8_t osd_vtx_freq[5];
        fast_fprint(osd_vtx_freq, 5, vtx_frequency_from_channel(vtx_settings.band, vtx_settings.channel), 0);
        osd_print_data(osd_vtx_freq, 5, osd_decode(*osd_vtx, ATTRIBUTE), osd_decode(*osd_vtx, POSITIONX), osd_decode(*osd_vtx, POSITIONY));
      }
      osd_display_element++;
      break;

    case 12: //end of regular display - display_trigger counter sticks here till it wraps
      display_trigger++;
      if (display_trigger == 0)
        osd_display_element = 1;
      break;
    }
    break;
    //**********************************************************************************************************************************************************************************************
    //																				OSD MENUS BELOW THIS POINT
    //**********************************************************************************************************************************************************************************************
  case 3: //pids profile menu
    last_display_phase = 1;
    print_osd_menu_strings(3, 2, pid_profiles_labels, pid_profiles_positions);
    if (osd_menu_phase == 4)
      osd_submenu_select(&profile.pid.pid_profile, 2, pid_submenu_map);
    break;

  case 4: //pids profiles
    last_display_phase = 3;
    if (profile.pid.pid_profile == PID_PROFILE_1)
      print_osd_menu_strings(8, 4, pid_profile1_labels, pid_profile_positions);
    else
      print_osd_menu_strings(8, 4, pid_profile2_labels, pid_profile_positions);
    print_osd_adjustable_vectors(BF_PIDS, 8, 9, get_pid_term(pid_profile_data_index[osd_menu_phase - 9][0]), pid_profile_data_index, pid_profile_grid, pid_profile_data_positions);
    if (osd_menu_phase == 18)
      osd_vector_adjust(get_pid_term(osd_cursor), 3, 3, BF_PIDS, pid_profile_adjust_limits);
    break;

  case 5: //filters menu
    last_display_phase = 1;
    print_osd_menu_strings(3, 2, filter_labels, filter_positions);
    if (osd_menu_phase == 4)
      osd_select_menu_item(2, filter_submenu_map, SUB_MENU);
    break;

  case 6: //main rates menu
    last_display_phase = 1;
    print_osd_menu_strings(3, 2, rates_profile_labels, rates_profile_positions);
    if (osd_menu_phase == 4)
      osd_submenu_select(&profile.rate.mode, 2, rates_submenu_map);
    break;

  case 7: //silverware rates submenu
    last_display_phase = 6;
    print_osd_menu_strings(8, 4, sw_rates_labels, sw_rates_positions);
    print_osd_adjustable_vectors(SW_RATES, 8, 9, get_sw_rate_term(sw_rates_data_index[osd_menu_phase - 9][0]), sw_rates_data_index, sw_rates_grid, sw_rates_data_positions);
    if (osd_menu_phase == 18)
      osd_vector_adjust(get_sw_rate_term(osd_cursor), 3, 3, SW_RATES, sw_rates_adjust_limits);
    break;

  case 8: //betaflight rates submenu
    last_display_phase = 6;
    print_osd_menu_strings(8, 4, bf_rates_labels, bf_rates_positions);
    print_osd_adjustable_vectors(ROUNDED, 8, 9, get_bf_rate_term(bf_rates_data_index[osd_menu_phase - 9][0]), bf_rates_data_index, bf_rates_grid, bf_rates_data_positions);
    if (osd_menu_phase == 18)
      osd_vector_adjust(get_bf_rate_term(osd_cursor), 3, 3, ROUNDED, bf_rates_adjust_limits);
    break;

  case 9: //flight modes menu
    last_display_phase = 1;
    print_osd_menu_strings(12, 11, flight_modes_labels, flight_modes_positions);
    print_osd_adjustable_enums(12, 10, get_aux_status(profile.channel.aux[flight_modes_aux_items[osd_menu_phase - 13]]), flight_modes_grid, flight_modes_data_positions);
    if (osd_menu_phase == 23)
      osd_enum_adjust(flight_modes_ptr, 10, flight_modes_aux_limits);
    break;

  case 10: //osd elements menu
    last_display_phase = 1;
    print_osd_menu_strings(5, 4, osd_elements_menu_labels, osd_elements_menu_positions);
    if (osd_menu_phase == 6)
      osd_select_menu_item(4, osd_elements_map, SUB_MENU);
    break;

  case 11: //vtx
    last_display_phase = 1;
    if (vtx_settings.detected) {
      populate_vtx_buffer_once();
      print_osd_menu_strings(6, 5, vtx_labels, vtx_positions);
      // print the buffer and not the actual status
      print_osd_adjustable_enums(6, 4, get_vtx_status(osd_menu_phase - 7), vtx_grid, vtx_data_positions);
      if (osd_menu_phase == 11)
        //adjust the buffer and not the actual settings
        osd_enum_adjust(vtx_ptr, 4, vtx_limits);
      //save & exit needs a function to write the buffer to the actual settings
    } else {
      print_osd_menu_strings(3, 0, vtx_na_labels, vtx_na_positions);
      if (osd_select)
        osd_select = 0;
    }
    break;

  case 12: //special features
    last_display_phase = 1;
    print_osd_menu_strings(8, 7, special_features_labels, special_features_positions);
    if (osd_menu_phase == 9)
      osd_select_menu_item(7, special_features_map, SUB_MENU);
    break;

  case 13: //stick boost profiles
    last_display_phase = 12;
    print_osd_menu_strings(3, 2, stickboost_labels, stickboost_profile_positions);
    if (osd_menu_phase == 4)
      osd_submenu_select(&profile.pid.stick_profile, 2, stickboost_submenu_map);
    break;

  case 14: //adjustable stick boost profiles
    last_display_phase = 13;
    if (profile.pid.stick_profile == STICK_PROFILE_OFF)
      print_osd_menu_strings(7, 3, stickboost1_labels, stickboost_positions);
    else
      print_osd_menu_strings(7, 3, stickboost2_labels, stickboost_positions);
    print_osd_adjustable_vectors(ROUNDED, 7, 6, get_stick_profile_term(stickboost_data_index[osd_menu_phase - 8][0]), stickboost_data_index, stickboost_grid, stickboost_data_positions);
    if (osd_menu_phase == 14)
      osd_vector_adjust(get_stick_profile_term(osd_cursor), 2, 3, ROUNDED, stickboost_adjust_limits);
    break;

  case 15: //add or remove osd elements to display
    last_display_phase = 10;
    print_osd_menu_strings(12, 11, osd_display_labels, osd_display_positions);
    print_osd_adjustable_enums(12, 10, get_decode_element_string(profile.osd.elements[osd_elements_active_items[osd_menu_phase - 13]], ACTIVE), osd_display_grid, osd_display_data_positions);
    if (osd_menu_phase == 23)
      osd_encoded_adjust(&profile.osd.elements[osd_elements_active_items[osd_cursor - 1]], 10, 1, ACTIVE);
    break;

  case 16: //edit element positions
    last_display_phase = 10;
    print_osd_menu_strings(14, 11, osd_position_labels, osd_position_adjust_positions);
    print_osd_adjustable_enums(14, 20, get_decode_element_string(profile.osd.elements[osd_position_active_items[osd_menu_phase - 15]], osd_position_index[osd_menu_phase - 15]), osd_position_grid, osd_position_data_positions);
    if (osd_menu_phase == 35 && osd_select > 0)
      osd_encoded_adjust(&profile.osd.elements[osd_elements_active_items[osd_cursor - 1]], 10, 2, osd_select + 1);
    break;

  case 17: //edit display text style
    last_display_phase = 10;
    print_osd_menu_strings(12, 11, osd_text_style, osd_text_style_positions);
    print_osd_adjustable_enums(12, 10, get_decode_element_string(profile.osd.elements[osd_elements_active_items[osd_menu_phase - 13]], ATTRIBUTE), osd_display_grid, osd_display_data_positions);
    if (osd_menu_phase == 23)
      osd_encoded_adjust(&profile.osd.elements[osd_elements_active_items[osd_cursor - 1]], 10, 1, ATTRIBUTE);
    break;

  case 18: //edit callsign text
    last_display_phase = 10;
    print_osd_menu_strings(23, 2, osd_callsign_edit_labels, osd_callsign_edit_positions);
    print_osd_callsign_adjustable(23, 20, osd_callsign_grid, osd_callsign_edit_data_positions);
    if (osd_menu_phase == 44)
      osd_encoded_adjust_callsign();
    break;

  case 19: //edit lowbatt threshold
    last_display_phase = 12;
    print_osd_menu_strings(3, 2, lowbatt_labels, lowbatt_positions);
    print_osd_adjustable_float(3, 1, low_batt_ptr, lowbatt_grid, lowbatt_data_positions, 1);
    if (osd_menu_phase == 5)
      osd_float_adjust(low_batt_ptr, 1, 1, lowbatt_adjust_limits, 0.1);
    break;

  case 20: //edit levelmode
    last_display_phase = 12;
    print_osd_menu_strings(3, 2, level_submenu_labels, level_submenu_positions);
    if (osd_menu_phase == 4)
      osd_select_menu_item(2, level_submenu_map, SUB_MENU);
    break;

  case 21: //motor boost submenu
    last_display_phase = 12;
    print_osd_menu_strings(3, 2, motor_boost_labels, motor_boost_positions);
    if (osd_menu_phase == 4)
      osd_select_menu_item(2, motor_boost_map, SUB_MENU);
    break;

  case 22: //edit digital idle
    last_display_phase = 12;
    print_osd_menu_strings(3, 2, motoridle_labels, motoridle_positions);
    print_osd_adjustable_float(3, 1, motoridle_ptr, motoridle_grid, motoridle_data_positions, 1);
    if (osd_menu_phase == 5)
      osd_float_adjust(motoridle_ptr, 1, 1, motoridle_adjust_limits, 0.1);
    break;

  case 23: //edit level max angle
    last_display_phase = 20;
    print_osd_menu_strings(3, 2, maxangle_labels, maxangle_positions);
    print_osd_adjustable_float(3, 1, level_maxangle_ptr, maxangle_grid, maxangle_data_positions, 0);
    if (osd_menu_phase == 5)
      osd_float_adjust(level_maxangle_ptr, 1, 1, maxangle_adjust_limits, 1.0);
    break;

  case 24: //edit level strength
    last_display_phase = 20;
    print_osd_menu_strings(6, 3, levelmode_labels, levelmode_positions);
    print_osd_adjustable_float(6, 4, level_pid_ptr, levelmode_grid, levelmode_data_positions, 1);
    if (osd_menu_phase == 11)
      osd_float_adjust(level_pid_ptr, 2, 2, levelmode_adjust_limits, 0.5);
    break;

  case 25: //edit torque boost
    last_display_phase = 21;
    print_osd_menu_strings(3, 2, torqueboost_labels, torqueboost_positions);
    print_osd_adjustable_float(3, 1, torqueboost_ptr, torqueboost_grid, torqueboost_data_positions, 1);
    if (osd_menu_phase == 5)
      osd_float_adjust(torqueboost_ptr, 1, 1, torqueboost_adjust_limits, 0.1);
    break;

  case 26: //edit throttle boost
    last_display_phase = 21;
    print_osd_menu_strings(3, 2, throttleboost_labels, throttleboost_positions);
    print_osd_adjustable_float(3, 1, throttleboost_ptr, throttleboost_grid, throttleboost_data_positions, 1);
    if (osd_menu_phase == 5)
      osd_float_adjust(throttleboost_ptr, 1, 1, throttleboost_adjust_limits, 0.5);
    break;

  case 27: //edit turtle throttle percent
    last_display_phase = 12;
    print_osd_menu_strings(3, 2, turtlethrottle_labels, turtlethrottle_positions);
    print_osd_adjustable_float(3, 1, turtlethrottle_ptr, turtlethrottle_grid, turtlethrottle_data_positions, 0);
    if (osd_menu_phase == 5)
      osd_float_adjust(turtlethrottle_ptr, 1, 1, turtlethrottle_adjust_limits, 10.0);
    break;

  case 28: //edit gyro filters
    last_display_phase = 5;
    print_osd_menu_strings(6, 5, gyrofilter_labels, gyrofilter_positions);
    print_osd_mixed_data(6, 4, gyrofilter_ptr, gyrofilter_ptr2, gyrofilter_type_labels, gyrofilter_grid, gyrofilter_data_positions, 0);
    if (osd_menu_phase == 11)
      osd_mixed_data_adjust(gyrofilter_ptr, gyrofilter_ptr2, 4, 1, gyrofilter_adjust_limits, 10.0, gyrofilter_reboot_request);
    break;

  case 29: //edit dterm filters
    last_display_phase = 5;
    print_osd_menu_strings(9, 8, dtermfilter_labels, dtermfilter_positions);
    print_osd_mixed_data(9, 7, dtermfilter_ptr, dtermfilter_ptr2, dtermfilter_type_labels, dtermfilter_grid, dtermfilter_data_positions, 0);
    if (osd_menu_phase == 17)
      osd_mixed_data_adjust(dtermfilter_ptr, dtermfilter_ptr2, 7, 1, dtermfilter_adjust_limits, 10.0, dtermfilter_reboot_request);
    break;

  case 30: //edit pid modifiers
    last_display_phase = 12;
    print_osd_menu_strings(6, 5, pidmodify_labels, pidmodify_positions);
    print_osd_mixed_data(6, 4, pidmodify_ptr, pidmodify_ptr2, pidmodify_type_labels, pidmodify_grid, pidmodify_data_positions, 2);
    if (osd_menu_phase == 11)
      osd_mixed_data_adjust(pidmodify_ptr, pidmodify_ptr2, 4, 1, pidmodify_adjust_limits, 0.05, pidmodify_reboot_request);
    break;

  case 31: //rc link
    last_display_phase = 1;
    print_osd_menu_strings(3, 2, rc_link_labels, rc_link_positions);
    if (osd_menu_phase == 4)
      osd_select_menu_item(7, rc_link_map, SUB_MENU);
    break;

  case 32:
    last_display_phase = 31;
    print_osd_menu_strings(4, 3, rssi_menu_labels, rssi_menu_positions);
    print_osd_adjustable_enums(4, 2, get_rssi_source_status(osd_menu_phase - 5), rssi_source_data_grid, rssi_source_data_positions);
    if (osd_menu_phase == 7)
      osd_enum_adjust(rssi_source_ptr, 2, rssi_source_limits);
    break;

  case 33:
    last_display_phase = 31;
    print_osd_menu_strings(1, 0, stick_wizard_labels, stick_wizard_positions);
    if (osd_select) //temp placeholder till feature is developed
      osd_select = 0;
    break;
  }
  if (osd_display_phase != 2 && rx_aux_on(AUX_ARMING))
    flags.arm_safety = 1; //final safety check to disallow arming during OSD operation
} //end osd_display()
//******************************************************************************************************************************
#endif
