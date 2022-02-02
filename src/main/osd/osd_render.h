#pragma once

#include <stdint.h>

typedef enum {
  OSD_CALLSIGN,
  OSD_FUELGAUGE_VOLTS,
  OSD_FUELGAUGE_VOLTS_CELLS,
  OSD_FILTERED_VOLTS,
  OSD_FILTERED_VOLTS_CELLS,
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

void osd_init();
void osd_display();
void osd_clear();
uint8_t osd_decode(uint32_t element, uint8_t status);
