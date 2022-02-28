#pragma once

#include <stdint.h>

#include "osd_menu_maps.h"

#define OSD_HISTORY_SIZE 8

typedef enum {
  OSD_INPUT_UP,
  OSD_INPUT_DOWN,
  OSD_INPUT_LEFT,
  OSD_INPUT_RIGHT,
} osd_input_t;

typedef enum {
  OSD_CALLSIGN,
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

typedef struct {
  osd_elements_t element;

  osd_screens_t screen;
  osd_screens_t screen_history[OSD_HISTORY_SIZE];
  uint8_t screen_history_size;
  uint8_t screen_phase;

  uint8_t cursor;
  uint8_t cursor_history[OSD_HISTORY_SIZE];
  uint8_t cursor_history_size;
  uint8_t cursor_min;
  uint8_t cursor_max;

  uint8_t selection;
  uint8_t selection_increase;
  uint8_t selection_decrease;

  uint8_t reboot_fc_requested;
} osd_state_t;

extern osd_state_t osd_state;

void osd_init();
void osd_display();
void osd_clear();

osd_screens_t osd_push_screen(osd_screens_t screen);
osd_screens_t osd_pop_screen();

void osd_handle_input(osd_input_t input);

uint8_t osd_decode(uint32_t element, uint8_t status);
