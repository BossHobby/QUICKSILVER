#include "navigation.h"

#include "driver/baro/baro.h"
#include "flight/control.h"
#include "io/gps.h"

void nav_update() {
  gps_update();

  const bool had_baro_update = baro_update();

  static uint8_t last_arm_state = 0;
  if (flags.arm_state != last_arm_state) {
    if (flags.arm_state) {
      // just armed
      state.baro_launch_altitude = state.baro_altitude;
    } else {
      state.baro_launch_altitude = 0;
    }
    last_arm_state = flags.arm_state;
  }

  if (!flags.arm_state) {
    state.altitude = 0;
  } else if (had_baro_update) {
    state.altitude = state.baro_altitude - state.baro_launch_altitude;
  }
}