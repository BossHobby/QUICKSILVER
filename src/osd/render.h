#pragma once

#include <stdint.h>

#include "driver/osd/osd.h"

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
  OSD_SCREEN_BLACKBOX_PRESETS,
  OSD_SCREEN_STICK_WIZARD,
  OSD_SCREEN_STICK_WIZARD_CALIBRATION,
  OSD_SCREEN_STICK_CONFIRM,
  OSD_SCREEN_STICK_RESULT
} osd_screens_t;

typedef enum {
  OSD_INPUT_NONE,
  OSD_INPUT_UP,
  OSD_INPUT_DOWN,
  OSD_INPUT_LEFT,
  OSD_INPUT_RIGHT,
  OSD_INPUT_MAX,
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
  OSD_CROSSHAIR,
  OSD_CURRENT_DRAWN,

  OSD_ELEMENT_MAX
} osd_elements_t;

typedef struct {
  uint32_t active : 1;
  uint32_t attribute : 1;
  uint32_t pos_sd_x : 5;
  uint32_t pos_sd_y : 5;
  uint32_t pos_hd_x : 7;
  uint32_t pos_hd_y : 7;
  uint32_t _unused : 6;
} __attribute__((packed)) osd_element_t;

#define ENCODE_OSD_ELEMENT(active, attr, sd_x, sd_y, hd_x, hd_y) ((hd_y << 20) | (hd_x << 12) | (sd_y << 7) | (sd_x << 2) | (attr << 1) | active)
#define pos_x(el) (osd_system == OSD_SYS_HD ? el->pos_hd_x : el->pos_sd_x)
#define pos_y(el) (osd_system == OSD_SYS_HD ? el->pos_hd_y : el->pos_sd_y)

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
extern osd_system_t osd_system;

void osd_init();
void osd_display();
void osd_display_reset();

uint8_t osd_attr(osd_element_t *el);

osd_screens_t osd_push_screen(osd_screens_t screen);
osd_screens_t osd_push_screen_replace(osd_screens_t screen);
osd_screens_t osd_pop_screen();

void osd_handle_input(osd_input_t input);

void osd_exit();
void osd_save_exit();
