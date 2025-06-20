#include "navigation.h"

#include <math.h>

#include "core/profile.h"
#include "driver/baro/baro.h"
#include "flight/control.h"
#include "io/gps.h"
#include "rx/rx.h"
#include "util/util.h"

#define EARTH_RADIUS 6371000.0f
#define METERS_PER_DEGREE_LAT (EARTH_RADIUS * M_PI_F / 180.0f)

// Navigation constants
#define NAV_KP 0.5f
#define NAV_KI 0.05f
#define NAV_KD 0.1f

#define ALT_KP 0.8f
#define ALT_KI 0.02f
#define ALT_KD 0.2f

#define RTH_CLIMB_RATE 3.0f      // m/s
#define RTH_DESCENT_RATE 2.0f    // m/s
#define RTH_CRUISE_SPEED 8.0f    // m/s
#define RTH_LAND_ALTITUDE 5.0f   // meters
#define RTH_LAND_RATE 0.5f       // m/s
#define RTH_HOME_RADIUS 3.0f     // meters

#define MAX_NAV_ANGLE 25.0f      // degrees
#define MAX_NAV_SPEED 10.0f      // m/s
#define MAX_YAW_RATE 45.0f       // deg/s


typedef struct {
  rth_state_t state;
  bool active;
  float target_altitude;
  float start_altitude;  // Altitude when RTH was activated
  float nav_integral_roll;
  float nav_integral_pitch;
  float alt_integral;
  float last_nav_error_roll;
  float last_nav_error_pitch;
  float last_alt_error;
  uint32_t last_update_time;
} rth_data_t;

static rth_data_t rth = {
    .state = RTH_STATE_INACTIVE,
    .active = false,
    .target_altitude = 0.0f,
    .start_altitude = 0.0f,
    .nav_integral_roll = 0.0f,
    .nav_integral_pitch = 0.0f,
    .alt_integral = 0.0f,
    .last_nav_error_roll = 0.0f,
    .last_nav_error_pitch = 0.0f,
    .last_alt_error = 0.0f,
    .last_update_time = 0
};

static void nav_update_gps(const gps_coord_t start, const gps_coord_t end) {
  const float scale = M_PI_F / (180.0f * 10000000.0f);

  const float lat_start_rad = (float)start.lat * scale;
  const float lon_start_rad = (float)start.lon * scale;
  const float lat_end_rad = (float)end.lat * scale;
  const float lon_end_rad = (float)end.lon * scale;

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

  state.home_distance = EARTH_RADIUS * 2.0f * atan2f(sqrtf(a), sqrtf(1.0f - a));

  const float y = sin_delta_lon * cos_lat_end;
  const float x = cos_lat_start * sin_lat_end - sin_lat_start * cos_lat_end * cos_delta_lon;

  state.home_bearing = normalize_rad(atan2f(y, x)) * RADTODEG;
}

static void nav_calculate_position_error(float *error_north, float *error_east) {
  // Calculate position error in meters
  const float scale = 1.0f / 10000000.0f;
  
  // Latitude difference in degrees
  const float lat_diff = (state.gps_home.lat - state.gps_coord.lat) * scale;
  *error_north = lat_diff * METERS_PER_DEGREE_LAT;
  
  // Longitude difference needs to account for latitude
  const float lon_diff = (state.gps_home.lon - state.gps_coord.lon) * scale;
  const float lat_rad = (float)state.gps_coord.lat * scale * DEGTORAD;
  const float meters_per_degree_lon = METERS_PER_DEGREE_LAT * cosf(lat_rad);
  *error_east = lon_diff * meters_per_degree_lon;
}

static void nav_update_altitude_control(float target_alt, float max_rate) {
  const uint32_t now = time_micros();
  const float dt = (now - rth.last_update_time) * 1e-6f;
  
  if (dt <= 0.0f || dt > 0.1f) {
    return; // Invalid dt
  }
  
  const float error = target_alt - state.altitude;
  
  // Limit climb/descent rate
  float rate_limited_error = error;
  if (fabsf(error) > max_rate * dt) {
    rate_limited_error = (error > 0) ? max_rate * dt : -max_rate * dt;
  }
  
  // PID controller for altitude
  const float p_term = ALT_KP * rate_limited_error;
  
  rth.alt_integral = constrain(rth.alt_integral + error * dt, -20.0f, 20.0f);
  const float i_term = ALT_KI * rth.alt_integral;
  
  const float derivative = (error - rth.last_alt_error) / dt;
  const float d_term = ALT_KD * derivative;
  rth.last_alt_error = error;
  
  // Convert to throttle command (0.0 to 1.0)
  float throttle = 0.5f + constrain(p_term + i_term + d_term, -0.4f, 0.4f);
  state.rx_override.throttle = constrain(throttle, 0.1f, 0.9f);
}

static void nav_update_horizontal_control() {
  const uint32_t now = time_micros();
  const float dt = (now - rth.last_update_time) * 1e-6f;
  
  if (dt <= 0.0f || dt > 0.1f) {
    return; // Invalid dt
  }
  
  // Calculate position error
  float error_north, error_east;
  nav_calculate_position_error(&error_north, &error_east);
  
  // Convert to body frame using current heading
  const float heading_rad = state.heading * DEGTORAD;
  const float cos_heading = cosf(heading_rad);
  const float sin_heading = sinf(heading_rad);
  
  const float error_forward = error_north * cos_heading + error_east * sin_heading;
  const float error_right = error_east * cos_heading - error_north * sin_heading;
  
  // Limit speed based on distance
  const float distance = sqrtf(error_north * error_north + error_east * error_east);
  float speed_limit = RTH_CRUISE_SPEED;
  
  // Slow down when approaching home
  if (distance < RTH_HOME_RADIUS * 3.0f) {
    speed_limit = RTH_CRUISE_SPEED * (distance / (RTH_HOME_RADIUS * 3.0f));
    speed_limit = constrain(speed_limit, 1.0f, RTH_CRUISE_SPEED);
  }
  
  // Apply speed limit to errors
  const float max_error = speed_limit * dt;
  float limited_error_forward = error_forward;
  float limited_error_right = error_right;
  if (fabsf(limited_error_forward) > max_error) {
    limited_error_forward = (limited_error_forward > 0) ? max_error : -max_error;
  }
  if (fabsf(limited_error_right) > max_error) {
    limited_error_right = (limited_error_right > 0) ? max_error : -max_error;
  }
  
  // PID controllers for roll and pitch
  // Pitch (forward/backward)
  const float pitch_p = NAV_KP * limited_error_forward;
  rth.nav_integral_pitch = constrain(rth.nav_integral_pitch + limited_error_forward * dt, -10.0f, 10.0f);
  const float pitch_i = NAV_KI * rth.nav_integral_pitch;
  const float pitch_d = NAV_KD * (limited_error_forward - rth.last_nav_error_pitch) / dt;
  rth.last_nav_error_pitch = limited_error_forward;
  
  // Roll (left/right)
  const float roll_p = NAV_KP * limited_error_right;
  rth.nav_integral_roll = constrain(rth.nav_integral_roll + limited_error_right * dt, -10.0f, 10.0f);
  const float roll_i = NAV_KI * rth.nav_integral_roll;
  const float roll_d = NAV_KD * (limited_error_right - rth.last_nav_error_roll) / dt;
  rth.last_nav_error_roll = limited_error_right;
  
  // Convert to angle commands and limit
  const float max_angle_rad = MAX_NAV_ANGLE * DEGTORAD;
  float pitch_angle = constrain(pitch_p + pitch_i + pitch_d, -max_angle_rad, max_angle_rad);
  float roll_angle = constrain(roll_p + roll_i + roll_d, -max_angle_rad, max_angle_rad);
  
  // Convert angles to stick inputs (-1 to 1)
  state.rx_override.pitch = -pitch_angle / max_angle_rad;
  state.rx_override.roll = roll_angle / max_angle_rad;
  
  // Yaw control to point toward home using IMU heading (with GPS fusion)
  if (distance > RTH_HOME_RADIUS) {
    // Calculate heading error
    float heading_error = state.home_bearing - state.heading;
    
    // Normalize to -180 to 180
    while (heading_error > 180.0f) heading_error -= 360.0f;
    while (heading_error < -180.0f) heading_error += 360.0f;
    
    // Use moderate yaw gain since heading is now fused at high rate
    float yaw_gain = 0.12f;
    
    // Convert to radians and limit yaw rate
    float yaw_rate = heading_error * DEGTORAD * yaw_gain;
    const float max_yaw_rate_rad = MAX_YAW_RATE * DEGTORAD;
    yaw_rate = constrain(yaw_rate, -max_yaw_rate_rad, max_yaw_rate_rad);
    
    // Convert to stick input
    state.rx_override.yaw = yaw_rate / max_yaw_rate_rad;
  } else {
    state.rx_override.yaw = 0.0f;
  }
}

static void nav_update_rth() {
  const uint32_t now = time_micros();
  
  // Check GPS requirements
  if (!state.gps_lock || state.gps_sats < GPS_MIN_SATS_FOR_LOCK) {
    // Lost GPS, hold position
    state.rx_override.roll = 0.0f;
    state.rx_override.pitch = 0.0f;
    state.rx_override.yaw = 0.0f;
    nav_update_altitude_control(state.altitude, 0.0f); // Hold current altitude
    return;
  }
  
  switch (rth.state) {
    case RTH_STATE_INACTIVE:
      // Should not happen
      break;
      
    case RTH_STATE_CLIMB:
      // Climb to RTH altitude (offset from start altitude)
      nav_update_altitude_control(rth.target_altitude, RTH_CLIMB_RATE);
      
      // Hold position while climbing
      state.rx_override.roll = 0.0f;
      state.rx_override.pitch = 0.0f;
      state.rx_override.yaw = 0.0f;
      
      // Check if we reached target altitude
      if (fabsf(state.altitude - rth.target_altitude) < 1.0f) {
        rth.state = RTH_STATE_NAVIGATE;
        // Reset navigation integrals
        rth.nav_integral_roll = 0.0f;
        rth.nav_integral_pitch = 0.0f;
      }
      break;
      
    case RTH_STATE_NAVIGATE:
      // Navigate to home position
      nav_update_altitude_control(rth.target_altitude, RTH_DESCENT_RATE);
      nav_update_horizontal_control();
      
      // Check if we're close to home
      if (state.home_distance < RTH_HOME_RADIUS) {
        if (profile.navigation.rth_auto_land) {
          rth.state = RTH_STATE_DESCEND;
          rth.target_altitude = RTH_LAND_ALTITUDE;
        } else {
          // Just hover at home
          rth.state = RTH_STATE_HOVER_HOME;
        }
      }
      break;
      
    case RTH_STATE_DESCEND:
      // Descend to landing altitude
      nav_update_altitude_control(rth.target_altitude, RTH_DESCENT_RATE);
      
      // Try to stay at home position
      nav_update_horizontal_control();
      
      // Check if we reached landing altitude
      if (state.altitude <= RTH_LAND_ALTITUDE + 0.5f) {
        rth.state = RTH_STATE_LAND;
        rth.target_altitude = 0.0f;
      }
      break;
      
    case RTH_STATE_LAND:
      // Land slowly
      nav_update_altitude_control(rth.target_altitude, RTH_LAND_RATE);
      
      // Try to stay at home position
      nav_update_horizontal_control();
      
      // Check if landed (altitude near zero or on ground)
      if (state.altitude < 0.5f || flags.on_ground) {
        rth.state = RTH_STATE_LANDED;
        // Disarm
        flags.arm_switch = 0;
        flags.arm_state = 0;
      }
      break;
      
    case RTH_STATE_LANDED:
      // Mission complete, disarm
      state.rx_override.roll = 0.0f;
      state.rx_override.pitch = 0.0f;
      state.rx_override.yaw = 0.0f;
      state.rx_override.throttle = 0.0f;
      flags.controls_override = 0;
      break;
      
    case RTH_STATE_HOVER_HOME:
      // Hover at home position
      nav_update_altitude_control(rth.target_altitude, RTH_DESCENT_RATE);
      nav_update_horizontal_control();
      // Keep controls active but don't disarm
      break;
  }
  
  rth.last_update_time = now;
}

void nav_rth_start() {
  if (!rth.active && state.gps_lock && state.gps_sats >= GPS_MIN_SATS_FOR_LOCK) {
    rth.active = true;
    rth.state = RTH_STATE_CLIMB;
    
    // Set target altitude as offset from current altitude
    rth.start_altitude = state.altitude;
    rth.target_altitude = rth.start_altitude + profile.navigation.rth_altitude;
    
    // Reset PID integrals
    rth.nav_integral_roll = 0.0f;
    rth.nav_integral_pitch = 0.0f;
    rth.alt_integral = 0.0f;
    rth.last_nav_error_roll = 0.0f;
    rth.last_nav_error_pitch = 0.0f;
    rth.last_alt_error = 0.0f;
    rth.last_update_time = time_micros();
    
    // Take control
    flags.controls_override = 1;
  }
}

void nav_rth_stop() {
  if (rth.active) {
    rth.active = false;
    rth.state = RTH_STATE_INACTIVE;
    flags.controls_override = 0;
    
    // Clear overrides
    state.rx_override.roll = 0.0f;
    state.rx_override.pitch = 0.0f;
    state.rx_override.yaw = 0.0f;
    state.rx_override.throttle = 0.0f;
  }
}

bool nav_rth_active() {
  return rth.active;
}

rth_state_t nav_get_rth_state() {
  return rth.state;
}

void nav_update() {
  const bool had_baro_update = baro_update();
  const bool had_gps_update = gps_update();

  static bool had_gps_lock_arm = false;

  static uint8_t last_arm_state = 0;
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
    
    // Stop RTH if disarmed
    if (rth.active) {
      nav_rth_stop();
    }
  } else {
    if (had_baro_update) {
      state.altitude = state.baro_altitude - state.baro_launch_altitude;
    }
    if (had_gps_update && had_gps_lock_arm) {
      nav_update_gps(state.gps_home, state.gps_coord);
    }

    // RTH control via AUX channel
    static uint8_t last_rth_aux = 0;
    const uint8_t rth_aux = rx_aux_on(AUX_RETURN_TO_HOME);
    if (rth_aux != last_rth_aux) {
      if (rth_aux) {
        nav_rth_start();
      } else {
        nav_rth_stop();
      }
      last_rth_aux = rth_aux;
    }

    // Update RTH if active
    if (rth.active) {
      nav_update_rth();
    }
    
    // Check for failsafe RTH
    static uint8_t last_failsafe = 0;
    if (profile.navigation.rth_on_failsafe && flags.failsafe != last_failsafe) {
      if (flags.failsafe && !rth.active && had_gps_lock_arm) {
        // Activate RTH on failsafe
        nav_rth_start();
      } else if (!flags.failsafe && rth.active && last_failsafe) {
        // Cancel RTH when failsafe cleared (unless manually activated)
        if (!rx_aux_on(AUX_RETURN_TO_HOME)) {
          nav_rth_stop();
        }
      }
      last_failsafe = flags.failsafe;
    }
  }
}