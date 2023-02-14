#pragma once

#include <stdint.h>

#define OSD_HISTORY_SIZE 8

typedef enum {
  OSD_SCREEN_CLEAR,
  OSD_SCREEN_MAIN_MENU,
  OSD_SCREEN_REGULAR,

  OSD_SCREEN_PID_PROFILE,
  OSD_SCREEN_PID,
  OSD_SCREEN_FILTERS,
  OSD_SCREEN_RATE_PROFILES,
  OSD_SCREEN_RATES,
  OSD_SCREEN_FLIGHT_MODES,
  OSD_SCREEN_ELEMENTS,
  OSD_SCREEN_VTX,
  OSD_SCREEN_SPECIAL_FEATURES,
  OSD_SCREEN_STICK_BOOST,
  OSD_SCREEN_STICK_BOOST_ADJUST,
  OSD_SCREEN_ELEMENTS_ADD_REMOVE,
  OSD_SCREEN_ELEMENTS_POSITION,
  OSD_SCREEN_ELEMENTS_STYLE,
  OSD_SCREEN_CALLSIGN,
  OSD_SCREEN_LOWBAT,
  OSD_SCREEN_LEVEL_MODE,
  OSD_SCREEN_MOTOR_BOOST,
  OSD_SCREEN_MOTOR_SETTINGS,
  OSD_SCREEN_THROTTLE_SETTINGS,
  OSD_SCREEN_LEVEL_MAX_ANGLE,
  OSD_SCREEN_LEVEL_STRENGTH,
  OSD_SCREEN_TORQUE_BOOST,
  OSD_SCREEN_THROTTLE_BOOST,
  OSD_SCREEN_GYRO_FILTER,
  OSD_SCREEN_DTERM_FILTER,
  OSD_SCREEN_PID_MODIFIER,
  OSD_SCREEN_RC_LINK,
  OSD_SCREEN_RSSI,
  OSD_SCREEN_BLACKBOX,
  OSD_SCREEN_STICK_WIZARD,
  OSD_SCREEN_STICK_WIZARD_CALIBRATION,
  OSD_SCREEN_STICK_CONFIRM,
  OSD_SCREEN_STICK_RESULT
} osd_screens_t;

typedef enum {
  OSD_INPUT_UP,
  OSD_INPUT_DOWN,
  OSD_INPUT_LEFT,
  OSD_INPUT_RIGHT,
} osd_input_t;

typedef enum {
  OSD_CALLSIGN,
  OSD_CELL_COUNT,
  OSD_FUELGAUGE_VOLTS,
  OSD_FILTERED_VOLTS,
  OSD_GYRO_TEMP,
  OSD_FLIGHT_MODE,
  OSD_RSSI,
  OSD_STOPWATCH,
  OSD_SYSTEM_STATUS,
  OSD_THROTTLE,
  OSD_VTX_CHANNEL,
  OSD_CURRENT_DRAW,

  OSD_ELEMENT_MAX
} osd_elements_t;

/*
screen elements characteristics written like registers in a 32bit binany number
BIT
0			-		0 is display element inactive , 1 is display element active
1			-		0 is TEXT, 1 is INVERT
2:10	-		the X screen position (column)
11:17	-		the Y screen position	(row)
17:32	-		not currently used
*/

typedef enum {
  ACTIVE = 0,
  ATTRIBUTE = 1,
  POSITIONX = 2,
  POSITIONY = 3,
} osd_element_attrs_t;

typedef struct {
  uint32_t active : 1;
  uint32_t attribute : 1;
  uint32_t pos_x : 8;
  uint32_t pos_y : 8;
  uint32_t _unused : 14;
} __attribute__((packed)) osd_element_t;

#define ENCODE_OSD_ELEMENT(active, attr, x, y) ((y << 10) | (x << 2) | (attr << 1) | active)

typedef enum {
  OSD_PHASE_CLEAR,
  OSD_PHASE_REFRESH,
  OSD_PHASE_RENDER,
  OSD_PHASE_IDLE,
} osd_screen_phase_t;

typedef struct {
  osd_elements_t element;

  osd_screens_t screen;
  osd_screens_t screen_history[OSD_HISTORY_SIZE];
  uint8_t screen_history_size;
  osd_screen_phase_t screen_phase;

  uint8_t cursor;
  uint8_t cursor_history[OSD_HISTORY_SIZE];
  uint8_t cursor_history_size;
  uint8_t cursor_min;
  uint8_t cursor_max;

  uint8_t selection;
  uint8_t selection_max;
  uint8_t selection_increase;
  uint8_t selection_decrease;

  uint8_t reboot_fc_requested;
} osd_state_t;

extern osd_state_t osd_state;

void osd_init();
void osd_display();
void osd_display_reset();
void osd_clear();

osd_screens_t osd_push_screen(osd_screens_t screen);
osd_screens_t osd_push_screen_replace(osd_screens_t screen);
osd_screens_t osd_pop_screen();

void osd_handle_input(osd_input_t input);

void osd_exit();
void osd_save_exit();
