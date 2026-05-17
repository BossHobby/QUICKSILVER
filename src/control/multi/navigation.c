#include "navigation.h"

#include <math.h>

#include "core/profile.h"
#include "driver/baro/baro.h"
#include "control/control.h"
#include "io/blackbox.h"
#include "io/gps.h"
#include "rx/rx.h"
#include "util/util.h"

#define EARTH_RADIUS 6371000.0f
#define METERS_PER_DEGREE_LAT (EARTH_RADIUS * M_PI_F / 180.0f)

#define ALT_KP 0.8f
#define ALT_KI 0.02f
#define ALT_KD 0.2f
#define RTH_CRUISE_SPEED_LIMIT 4.0f
#define RTH_MAX_ANGLE_LIMIT 15.0f
#define RTH_MAX_YAW_RATE_LIMIT 30.0f
#define RTH_NAV_ACCEL_LIMIT 2.0f
#define RTH_NAV_KD_LIMIT 0.01f
#define RTH_THROTTLE_HEADROOM 0.15f
#define RTH_THROTTLE_SLEW_RATE 1.5f


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
  float last_altitude;
  float last_distance;
  float best_distance;
  float progress_boost;
  float last_roll_command;
  float last_pitch_command;
  float last_yaw_command;
  float last_throttle_command;
  float last_desired_vel_north;
  float last_desired_vel_east;
  uint32_t last_update_time;
  uint32_t last_progress_time_ms;
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
    .last_altitude = 0.0f,
    .last_distance = 0.0f,
    .best_distance = 0.0f,
    .progress_boost = 1.0f,
    .last_roll_command = 0.0f,
    .last_pitch_command = 0.0f,
    .last_yaw_command = 0.0f,
    .last_throttle_command = 0.0f,
    .last_desired_vel_north = 0.0f,
    .last_desired_vel_east = 0.0f,
    .last_update_time = 0,
    .last_progress_time_ms = 0
};

static bool nav_home_valid = false;
static bool nav_gps_sane = false;
static gps_coord_t nav_last_gps_coord;
static bool nav_last_gps_coord_valid = false;
static uint32_t nav_last_checked_gps_ms = 0;

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

static float nav_distance_between(const gps_coord_t start, const gps_coord_t end) {
  const float scale = 1.0f / 10000000.0f;
  const float lat_diff = (float)(end.lat - start.lat) * scale;
  const float lon_diff = (float)(end.lon - start.lon) * scale;
  const float lat_rad = (float)start.lat * scale * DEGTORAD;
  const float north = lat_diff * METERS_PER_DEGREE_LAT;
  const float east = lon_diff * METERS_PER_DEGREE_LAT * cosf(lat_rad);
  return sqrtf(north * north + east * east);
}

static bool nav_gps_quality_ok(void) {
  const uint8_t min_sats = max(profile.navigation.rth_min_sats, GPS_MIN_SATS_FOR_LOCK);
  const uint32_t stale_ms = max(profile.navigation.rth_gps_stale_ms, 100U);
  const float max_hacc = max(profile.navigation.rth_max_horizontal_accuracy, 1.0f);

  return state.gps_lock &&
         state.gps_sats >= min_sats &&
         state.gps_horizontal_accuracy <= max_hacc &&
         (time_millis() - state.gps_last_update_ms) <= stale_ms;
}

static void nav_update_gps_sanity(void) {
  if (!nav_gps_quality_ok()) {
    nav_gps_sane = false;
    return;
  }

  if (state.gps_last_update_ms == nav_last_checked_gps_ms) {
    return;
  }
  nav_last_checked_gps_ms = state.gps_last_update_ms;

  if (nav_last_gps_coord_valid) {
    const float jump = nav_distance_between(nav_last_gps_coord, state.gps_coord);
    if (jump > max(profile.navigation.rth_max_position_jump, 1.0f)) {
      nav_gps_sane = false;
      return;
    }
  }

  nav_last_gps_coord = state.gps_coord;
  nav_last_gps_coord_valid = true;
  nav_gps_sane = true;
}

static bool nav_rth_can_start(void) {
  return nav_home_valid &&
         nav_gps_sane &&
         state.home_distance >= profile.navigation.rth_min_distance &&
         profile.navigation.rth_throttle_min < profile.navigation.rth_throttle_max;
}

static void nav_update_altitude_control(float target_alt, float max_rate, float dt) {

  const float error = target_alt - state.altitude;
  const float configured_throttle_min = constrain(profile.navigation.rth_throttle_min, 0.0f, 1.0f);
  const float configured_throttle_hover = constrain(profile.navigation.rth_throttle_hover, configured_throttle_min, 1.0f);
  const float throttle_min = max(configured_throttle_min, configured_throttle_hover * 0.5f);
  const float throttle_ceiling = max(throttle_min, 1.0f - RTH_THROTTLE_HEADROOM);
  const float throttle_max = constrain(profile.navigation.rth_throttle_max, throttle_min, throttle_ceiling);
  const float throttle_hover = constrain(configured_throttle_hover, throttle_min, throttle_max);
  const float safe_dt = max(dt, 0.001f);

  const float desired_rate = constrain(error, -max_rate, max_rate);
  const float vertical_rate = (state.altitude - rth.last_altitude) / safe_dt;
  rth.last_altitude = state.altitude;
  const float rate_error = desired_rate - vertical_rate;

  // PID controller for altitude
  const float p_term = ALT_KP * rate_error;

  rth.alt_integral = constrain(rth.alt_integral + error * safe_dt, -20.0f, 20.0f);
  const float i_term = ALT_KI * rth.alt_integral;

  const float derivative = (error - rth.last_alt_error) / safe_dt;
  const float d_term = ALT_KD * derivative;
  rth.last_alt_error = error;

  // Convert to throttle command (0.0 to 1.0)
  const float throttle = throttle_hover + constrain(p_term + i_term + d_term, throttle_min - throttle_hover, throttle_max - throttle_hover);
  const float throttle_target = constrain(throttle, throttle_min, throttle_max);
  const float throttle_step = RTH_THROTTLE_SLEW_RATE * safe_dt;
  rth.last_throttle_command = constrain(throttle_target, rth.last_throttle_command - throttle_step, rth.last_throttle_command + throttle_step);
  state.rx_override.throttle = constrain(rth.last_throttle_command, throttle_min, throttle_max);
}

static void nav_update_horizontal_control(float dt) {

  const float safe_dt = max(dt, 0.001f);
  float error_north, error_east;
  nav_calculate_position_error(&error_north, &error_east);
  const float distance = sqrtf(error_north * error_north + error_east * error_east);

  const float cruise_speed = constrain(profile.navigation.rth_cruise_speed * rth.progress_boost, 0.1f, RTH_CRUISE_SPEED_LIMIT);
  const float home_radius = max(profile.navigation.rth_home_radius, 0.1f);
  float speed_limit = cruise_speed;
  if (distance < home_radius * 3.0f) {
    speed_limit = cruise_speed * (distance / (home_radius * 3.0f));
    speed_limit = constrain(speed_limit, min(1.0f, cruise_speed), cruise_speed);
  }

  float desired_vel_north = 0.0f;
  float desired_vel_east = 0.0f;
  if (distance > 0.01f) {
    const float desired_speed = min(speed_limit, distance);
    desired_vel_north = error_north * (desired_speed / distance);
    desired_vel_east = error_east * (desired_speed / distance);
  }

  const float velocity_step = RTH_NAV_ACCEL_LIMIT * safe_dt;
  rth.last_desired_vel_north = constrain(desired_vel_north, rth.last_desired_vel_north - velocity_step, rth.last_desired_vel_north + velocity_step);
  rth.last_desired_vel_east = constrain(desired_vel_east, rth.last_desired_vel_east - velocity_step, rth.last_desired_vel_east + velocity_step);
  desired_vel_north = rth.last_desired_vel_north;
  desired_vel_east = rth.last_desired_vel_east;

  const float vel_error_north = desired_vel_north - state.gps_vel_north;
  const float vel_error_east = desired_vel_east - state.gps_vel_east;

  const float heading_rad = state.heading * DEGTORAD;
  const float cos_heading = cosf(heading_rad);
  const float sin_heading = sinf(heading_rad);

  const float error_forward = vel_error_north * cos_heading + vel_error_east * sin_heading;
  const float error_right = vel_error_east * cos_heading - vel_error_north * sin_heading;

  blackbox_set_debug(BBOX_DEBUG_NAVIGATION, 7, (int16_t)(error_forward * 10)); // 0.1 m/s
  blackbox_set_debug(BBOX_DEBUG_NAVIGATION, 8, (int16_t)(error_right * 10)); // 0.1 m/s
  blackbox_set_debug(BBOX_DEBUG_NAVIGATION, 10, (int16_t)(desired_vel_north * 10));
  blackbox_set_debug(BBOX_DEBUG_NAVIGATION, 11, (int16_t)(desired_vel_east * 10));
  blackbox_set_debug(BBOX_DEBUG_NAVIGATION, 12, (int16_t)(state.gps_vel_north * 10));
  blackbox_set_debug(BBOX_DEBUG_NAVIGATION, 13, (int16_t)(state.gps_vel_east * 10));

  const float pitch_p = profile.navigation.nav_vel_kp * error_forward;
  rth.nav_integral_pitch = constrain(rth.nav_integral_pitch + error_forward * safe_dt, -10.0f, 10.0f);
  const float pitch_i = profile.navigation.nav_vel_ki * rth.nav_integral_pitch;
  const float nav_vel_kd = constrain(profile.navigation.nav_vel_kd, 0.0f, RTH_NAV_KD_LIMIT);
  const float pitch_d = nav_vel_kd * (error_forward - rth.last_nav_error_pitch) / safe_dt;
  rth.last_nav_error_pitch = error_forward;

  const float roll_p = profile.navigation.nav_vel_kp * error_right;
  rth.nav_integral_roll = constrain(rth.nav_integral_roll + error_right * safe_dt, -10.0f, 10.0f);
  const float roll_i = profile.navigation.nav_vel_ki * rth.nav_integral_roll;
  const float roll_d = nav_vel_kd * (error_right - rth.last_nav_error_roll) / safe_dt;
  rth.last_nav_error_roll = error_right;

  // Convert to angle commands and limit
  const float max_angle_rad = constrain(profile.navigation.rth_max_angle * rth.progress_boost, 1.0f, RTH_MAX_ANGLE_LIMIT) * DEGTORAD;
  const float pitch_angle = constrain(pitch_p + pitch_i + pitch_d, -max_angle_rad, max_angle_rad);
  const float roll_angle = constrain(roll_p + roll_i + roll_d, -max_angle_rad, max_angle_rad);

  // Angle mode interprets stick input using level_max_angle, so scale by that
  // rather than by the navigation angle limit.
  const float level_max_angle_rad = max(profile.rate.level_max_angle, 1.0f) * DEGTORAD;
  const float slew_step = max(profile.navigation.rth_angle_slew_rate, 0.1f) * safe_dt;
  const float pitch_command = pitch_angle / level_max_angle_rad;
  const float roll_command = roll_angle / level_max_angle_rad;
  rth.last_pitch_command = constrain(pitch_command, rth.last_pitch_command - slew_step, rth.last_pitch_command + slew_step);
  rth.last_roll_command = constrain(roll_command, rth.last_roll_command - slew_step, rth.last_roll_command + slew_step);
  state.rx_override.pitch = rth.last_pitch_command;
  state.rx_override.roll = rth.last_roll_command;
  blackbox_set_debug(BBOX_DEBUG_NAVIGATION, 14, (int16_t)(state.rx_override.pitch * 1000));
  blackbox_set_debug(BBOX_DEBUG_NAVIGATION, 15, (int16_t)(state.rx_override.roll * 1000));

  // Yaw control to point toward home using IMU heading (with GPS fusion)
  if (distance > home_radius) {
    // Calculate heading error
    float heading_error = state.home_bearing - state.heading;

    // Normalize to -180 to 180
    while (heading_error > 180.0f) heading_error -= 360.0f;
    while (heading_error < -180.0f) heading_error += 360.0f;

    // Log navigation heading error (different from GPS/IMU heading error)
    blackbox_set_debug(BBOX_DEBUG_NAVIGATION, 9, (int16_t)(heading_error * 10)); // 0.1 degrees

    // Base yaw gain with speed-based confidence adjustment
    // At low speeds, reduce yaw aggressiveness to allow GPS heading to stabilize
    // Reduced from 0.12 to 0.08 to minimize oscillation
    float yaw_gain = 0.08f * constrain(state.gps_speed / cruise_speed, 0.2f, 1.0f);

    // Progressive gain reduction for large errors to prevent overshooting
    if (fabsf(heading_error) > 90.0f) {
      yaw_gain *= 0.4f;  // Stronger reduction for very large errors
    } else if (fabsf(heading_error) > 45.0f) {
      yaw_gain *= 0.6f;  // Moderate reduction for medium errors
    }

    // Add deadband to prevent hunting around target heading
    if (fabsf(heading_error) < 5.0f) {
      heading_error = 0.0f;
    }

    // Convert to radians and limit yaw rate
    float yaw_rate = heading_error * DEGTORAD * yaw_gain;
    const float max_yaw_rate_rad = constrain(profile.navigation.rth_max_yaw_rate, 1.0f, RTH_MAX_YAW_RATE_LIMIT) * DEGTORAD;
    yaw_rate = constrain(yaw_rate, -max_yaw_rate_rad, max_yaw_rate_rad);

    // Convert to stick input (positive yaw rate = turn right/clockwise)
    float yaw_command = yaw_rate / max_yaw_rate_rad;

    // Apply exponential smoothing to reduce oscillation
    const float smoothing_factor = 0.7f;  // 0 = no smoothing, 1 = full smoothing
    yaw_command = rth.last_yaw_command * smoothing_factor + yaw_command * (1.0f - smoothing_factor);
    rth.last_yaw_command = yaw_command;

    state.rx_override.yaw = yaw_command;
  } else {
    state.rx_override.yaw = 0.0f;
    rth.last_yaw_command = 0.0f;
  }
}

#ifdef PIO_UNIT_TESTING
void nav_test_reset(void) {
  rth.nav_integral_roll = 0.0f;
  rth.nav_integral_pitch = 0.0f;
  rth.last_nav_error_roll = 0.0f;
  rth.last_nav_error_pitch = 0.0f;
  rth.last_altitude = 0.0f;
  rth.last_distance = 0.0f;
  rth.best_distance = 0.0f;
  rth.progress_boost = 1.0f;
  rth.last_roll_command = 0.0f;
  rth.last_pitch_command = 0.0f;
  rth.last_yaw_command = 0.0f;
  rth.last_throttle_command = 0.0f;
  rth.last_desired_vel_north = 0.0f;
  rth.last_desired_vel_east = 0.0f;
}

void nav_test_update_horizontal_control(float dt) {
  nav_update_horizontal_control(dt);
}
#endif

static void nav_update_progress(uint32_t now_ms) {
  const float progress_distance = max(profile.navigation.rth_progress_distance, 0.5f);
  const uint32_t timeout_ms = max(profile.navigation.rth_progress_timeout_ms, 1000U);

  if (state.home_distance + progress_distance < rth.best_distance) {
    rth.best_distance = state.home_distance;
    rth.last_progress_time_ms = now_ms;
    rth.progress_boost = 1.0f;
    return;
  }

  if ((now_ms - rth.last_progress_time_ms) > timeout_ms) {
    rth.progress_boost = constrain(rth.progress_boost + 0.25f, 1.0f, 1.75f);
    rth.last_progress_time_ms = now_ms;
  }
}

static void nav_update_rth() {
  const uint32_t now = time_micros();
  const uint32_t now_ms = time_millis();
  const float dt = (now - rth.last_update_time) * 1e-6f;
  rth.last_update_time = now;

  // Check GPS requirements
  if (!nav_gps_sane) {
    // Lost GPS, hold position
    state.rx_override.roll = 0.0f;
    state.rx_override.pitch = 0.0f;
    state.rx_override.yaw = 0.0f;
    nav_update_altitude_control(state.altitude, 0.0f, dt); // Hold current altitude
    return;
  }

  switch (rth.state) {
    case RTH_STATE_INACTIVE:
      // Should not happen
      break;

    case RTH_STATE_CLIMB:
      // Climb to RTH altitude (offset from start altitude)
      nav_update_altitude_control(rth.target_altitude, profile.navigation.rth_climb_rate, dt);

      // Hold level while climbing; horizontal navigation starts after target altitude.
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
      nav_update_progress(now_ms);
      nav_update_altitude_control(rth.target_altitude, profile.navigation.rth_descent_rate, dt);
      nav_update_horizontal_control(dt);

      // Check if we're close to home
      if (state.home_distance < profile.navigation.rth_home_radius) {
        rth.state = RTH_STATE_HOVER_HOME;
      }
      break;

    case RTH_STATE_HOVER_HOME:
      // Hover at home position
      nav_update_progress(now_ms);
      nav_update_altitude_control(rth.target_altitude, profile.navigation.rth_descent_rate, dt);
      nav_update_horizontal_control(dt);
      // Keep controls active but don't disarm
      break;
    }
}

void nav_rth_start() {
  if (!rth.active && nav_rth_can_start()) {
    rth.active = true;
    rth.state = RTH_STATE_CLIMB;

    // Set target altitude as offset from current altitude
    rth.start_altitude = state.altitude;
    rth.target_altitude = rth.start_altitude + profile.navigation.rth_altitude;

    // Reset navigation state
    rth.last_altitude = state.altitude;
    rth.last_distance = state.home_distance;
    rth.best_distance = state.home_distance;
    rth.progress_boost = 1.0f;
    rth.last_roll_command = 0.0f;
    rth.last_pitch_command = 0.0f;
    rth.last_yaw_command = 0.0f;
    rth.last_throttle_command = constrain(state.throttle, profile.navigation.rth_throttle_min, profile.navigation.rth_throttle_max);
    rth.last_desired_vel_north = 0.0f;
    rth.last_desired_vel_east = 0.0f;

    // Reset PID integrals
    rth.nav_integral_roll = 0.0f;
    rth.nav_integral_pitch = 0.0f;
    rth.alt_integral = 0.0f;
    rth.last_nav_error_roll = 0.0f;
    rth.last_nav_error_pitch = 0.0f;
    rth.last_alt_error = 0.0f;
    rth.last_update_time = time_micros();
    rth.last_progress_time_ms = time_millis();

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
    rth.last_roll_command = 0.0f;
    rth.last_pitch_command = 0.0f;
    rth.last_yaw_command = 0.0f;
    rth.last_throttle_command = 0.0f;
    rth.last_desired_vel_north = 0.0f;
    rth.last_desired_vel_east = 0.0f;
    rth.progress_boost = 1.0f;
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
  nav_update_gps_sanity();

  static bool had_gps_lock_arm = false;

  static uint8_t last_arm_state = 0;
  if (flags.arm_state != last_arm_state) {
    if (flags.arm_state) {
      // just armed
      state.baro_launch_altitude = state.baro_altitude;
      had_gps_lock_arm = nav_gps_sane;
      nav_home_valid = had_gps_lock_arm;
      if (nav_home_valid)
        state.gps_home = state.gps_coord;
    }
    last_arm_state = flags.arm_state;
  }

  if (!flags.arm_state) {
    state.altitude = 0;
    state.baro_launch_altitude = 0;
    state.gps_home = state.gps_coord;
    had_gps_lock_arm = false;
    nav_home_valid = false;

    // Stop RTH if disarmed
    if (rth.active) {
      nav_rth_stop();
    }
  } else {
    if (had_baro_update) {
      state.altitude = state.baro_altitude - state.baro_launch_altitude;
    }
    if (nav_gps_sane && had_gps_lock_arm) {
      nav_update_gps(state.gps_coord, state.gps_home);
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
      if (flags.failsafe && !rth.active && nav_home_valid) {
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
  // log navigation/rth data to blackbox for debugging
  // slots 0-3 are used by attitude for gps heading fusion
  blackbox_set_debug(BBOX_DEBUG_NAVIGATION, 4, (int16_t)(rth.state));
  blackbox_set_debug(BBOX_DEBUG_NAVIGATION, 5, (int16_t)(state.home_distance)); // meters
  blackbox_set_debug(BBOX_DEBUG_NAVIGATION, 6, (int16_t)(state.home_bearing * 10)); // 0.1 deg
  // slots 7-9 are set in nav_update_horizontal_control when RTH is active
}
