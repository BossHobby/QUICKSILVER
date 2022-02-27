#pragma once

#include <stdint.h>

#include "osd_menu_maps.h"

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

  uint8_t menu_phase;

  osd_screens_t screen;
  uint8_t last_display_phase;
} osd_state_t;

extern osd_state_t osd_state;

void osd_init();
void osd_display();
void osd_clear();
uint8_t osd_decode(uint32_t element, uint8_t status);
