#include "navigation.h"

#include <math.h>

#include "driver/baro/baro.h"
#include "flight/control.h"
#include "io/gps.h"
#include "util/util.h"

static void calculate_bearing_and_distance(const gps_coord_t start, const gps_coord_t end, float *bearing, float *distance) {
  // Constants
  const float EARTH_RADIUS = 6371000.0f;
  const float SCALE = M_PI_F / (180.0f * 10000000.0f);

  const float lat_start_rad = (float)start.lat * SCALE;
  const float lon_start_rad = (float)start.lon * SCALE;
  const float lat_end_rad = (float)end.lat * SCALE;
  const float lon_end_rad = (float)end.lon * SCALE;

  const float cos_lat_start = cosf(lat_start_rad);
  const float sin_lat_start = sinf(lat_start_rad);
  const float cos_lat_end = cosf(lat_end_rad);
  const float sin_lat_end = sinf(lat_end_rad);
  const float delta_lon = lon_end_rad - lon_start_rad;
  const float cos_delta_lon = cosf(delta_lon);
  const float sin_delta_lon = sinf(delta_lon);

  const float sin_half_delta_lat = sinf((lat_end_rad - lat_start_rad) * 0.5f);
  const float sin_half_delta_lon = sinf(delta_lon * 0.5f);
  const float a = sin_half_delta_lat * sin_half_delta_lat +
                  cos_lat_start * cos_lat_end * sin_half_delta_lon * sin_half_delta_lon;

  *distance = EARTH_RADIUS * 2.0f * atan2f(sqrtf(a), sqrtf(1.0f - a));

  const float y = sin_delta_lon * cos_lat_end;
  const float x = cos_lat_start * sin_lat_end - sin_lat_start * cos_lat_end * cos_delta_lon;

  *bearing = normalize_rad(atan2f(y, x)) * RADTODEG;
}

void nav_update() {
  const bool had_baro_update = baro_update();
  const bool had_gps_update = gps_update();

  static uint8_t last_arm_state = 0;
  static bool had_gps_lock_arm = false;
  if (flags.arm_state != last_arm_state) {
    if (flags.arm_state) {
      // just armed
      state.baro_launch_altitude = state.baro_altitude;
      had_gps_lock_arm = state.gps_lock;
      if (had_gps_lock_arm)
        state.gps_home = state.gps_coord;
    }
    last_arm_state = flags.arm_state;
  }

  if (!flags.arm_state) {
    state.altitude = 0;
    state.baro_launch_altitude = 0;
    state.gps_home = state.gps_coord;
    had_gps_lock_arm = false;
  } else if (had_baro_update) {
    state.altitude = state.baro_altitude - state.baro_launch_altitude;
  } else if (had_gps_update && had_gps_lock_arm) {
    if (state.gps_heading_accuracy < 2.0f) {
      state.heading = normalize_deg(state.gps_heading);
      state.attitude.yaw = state.heading * DEGTORAD;
    } else {
      state.heading = normalize_deg(state.attitude.yaw * RADTODEG);
    }
    calculate_bearing_and_distance(state.gps_home, state.gps_coord, &state.home_bearing, &state.home_distance);
  }
}