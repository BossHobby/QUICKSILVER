#pragma once

#include <stdbool.h>

#define GPS_MIN_SATS_FOR_LOCK 4

void gps_init();
bool gps_update();