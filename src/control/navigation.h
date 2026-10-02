#pragma once

#include <stdint.h>

#include "control/control.h"
#include "util/util.h"

constexpr float EARTH_RADIUS = 6371000.0f; // meters
constexpr float METERS_PER_DEGREE_LAT = EARTH_RADIUS * M_PI_F / 180.0f;
constexpr uint32_t NAV_GPS_STALE_MS = 500;

// Local displacement in meters; GPS coordinates are signed degrees * 1e7.
void nav_position_delta(gps_coord_t start, gps_coord_t end, float *north, float *east);

void nav_init();
void nav_update();

// Smoothed earth-vertical acceleration, m/s^2 up.
float nav_vertical_accel();
