#include "navigation.h"

#include <math.h>

#include "control/control.h"
#include "core/profile.h"
#include "driver/baro/baro.h"
#include "io/blackbox.h"
#include "io/gps.h"
#include "rx/rx.h"
#include "util/util.h"

#define EARTH_RADIUS 6371000.0f
#define METERS_PER_DEGREE_LAT (EARTH_RADIUS * M_PI_F / 180.0f)

#define ALT_KP 0.15f // relative to the estimated level hover throttle
#define ALT_KI 0.05f
#define ALT_POSITION_KP 0.5f // m/s per meter of altitude error
#define ALT_ACCEL_LIMIT 1.0f // m/s^2 target vertical acceleration
#define RTH_MAX_YAW_RATE_LIMIT 30.0f
#define RTH_CLIMB_RATE 2.0f
#define RTH_DESCENT_RATE 2.0f
#define RTH_HOME_RADIUS 3.0f
#define RTH_ANGLE_SLEW_RATE 2.0f // normalized stick/s
#define RTH_MIN_DISTANCE 10.0f
#define RTH_MAX_HORIZONTAL_ACCURACY 30.0f
#define RTH_MAX_POSITION_JUMP 50.0f
#define RTH_GPS_STALE_MS 500U
#define RTH_PROGRESS_TIMEOUT_MS 15000U
#define RTH_PROGRESS_DISTANCE 5.0f
#define NAV_VEL_KP 0.08f
#define NAV_VEL_KI 0.02f
#define NAV_VEL_KD 0.0f
#define RTH_NAV_ACCEL_LIMIT 2.0f
#define RTH_NAV_BRAKE_ACCEL 1.5f // m/s^2, conservative approach deceleration
#define RTH_NAV_BRAKE_DELAY 4.0f // seconds, includes the velocity PI controller's response
#define RTH_NAV_POSITION_KP 0.5f // m/s per meter near home
#define RTH_NAV_COMMAND_TIME_CONSTANT 0.15f // seconds, smooth GPS-driven tilt changes
#define RTH_THROTTLE_HEADROOM 0.15f
#define RTH_THROTTLE_SLEW_RATE 1.5f
#define RTH_GPS_LOSS_TIMEOUT_MS 3000U
#define RTH_CLIMB_TIMEOUT_MS 10000U
#define RTH_YAW_KP 1.0f
#define RTH_TURN_ERROR_DEG 10.0f
#define RTH_TURN_RATE_DEG 5.0f
#define RTH_REALIGN_ERROR_DEG 45.0f
#define RTH_ACQUIRE_TIMEOUT_MS 15000U
#define RTH_ACQUIRE_DISTANCE_M 75.0f
#define RTH_ACQUIRE_PITCH_DEG 35.0f
#define RTH_ACQUIRE_SLEW_RATE 0.2f // normalized stick/s
#define RTH_ACQUIRE_CONFIDENCE 0.6f

// Sensor and input history survives RTH stop/start. Published outputs live in state.
static struct {
  bool home_valid;
  struct {
    bool valid;
    gps_coord_t position;
    bool position_valid;
    uint32_t checked_ms;
  } gps;
  struct {
    bool armed;
    bool rth_aux;
    bool failsafe;
  } previous;
} nav;

// Controller history is reset on every engagement.
static struct {
  float target_altitude; // meters above launch
  uint32_t updated_us;
  struct {
    uint32_t started_ms;
    gps_coord_t origin;
  } acquisition;
  struct {
    float integral_north;
    float integral_east;
    float velocity_north;
    float velocity_east;
    float acceleration_north;
    float acceleration_east;
    float desired_velocity_north;
    float desired_velocity_east;
    float roll_command;
    float pitch_command;
    bool sample_valid;
    uint32_t updated_ms;
  } horizontal;
  struct {
    float integral;
    float throttle_command;
    float desired_rate;
  } vertical;
  struct {
    float best_error; // meters: altitude error in CLIMB, distance home otherwise
    uint32_t updated_ms;
  } progress;
  struct {
    bool active;
    uint32_t started_ms;
    float altitude; // meters above launch, latched on loss
  } gps_loss;
} rth;

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

// Local displacement in meters; GPS coordinates are signed degrees * 1e7.
static void nav_position_delta(gps_coord_t start, gps_coord_t end, float *north, float *east) {
  const float scale = 1.0f / 10000000.0f;
  const float lat_diff = (end.lat - start.lat) * scale;
  const float lon_diff = (end.lon - start.lon) * scale;
  const float lat_rad = start.lat * scale * DEGTORAD;
  *north = lat_diff * METERS_PER_DEGREE_LAT;
  *east = lon_diff * (METERS_PER_DEGREE_LAT * cosf(lat_rad));
}

static float nav_distance_between(gps_coord_t start, gps_coord_t end) {
  float north, east;
  nav_position_delta(start, end, &north, &east);
  return sqrtf(north * north + east * east);
}

static bool nav_gps_quality_ok(void) {
  return state.gps_lock &&
         state.gps_sats >= GPS_MIN_SATS_FOR_LOCK &&
         state.gps_horizontal_accuracy <= RTH_MAX_HORIZONTAL_ACCURACY &&
         (time_millis() - state.gps_last_update_ms) <= RTH_GPS_STALE_MS;
}

static void nav_update_gps_sanity(void) {
  if (!nav_gps_quality_ok()) {
    nav.gps.valid = false;
    nav.gps.position_valid = false;
    return;
  }

  if (state.gps_last_update_ms == nav.gps.checked_ms) {
    return;
  }
  nav.gps.checked_ms = state.gps_last_update_ms;

  if (nav.gps.position_valid) {
    const float jump = nav_distance_between(nav.gps.position, state.gps_coord);
    if (jump > RTH_MAX_POSITION_JUMP) {
      nav.gps.valid = false;
      return;
    }
  }

  nav.gps.position = state.gps_coord;
  nav.gps.position_valid = true;
  nav.gps.valid = true;
}

static bool nav_altitude_source_ok(void) {
  return state.baro_valid &&
         (time_millis() - state.baro_last_update_ms) <= BARO_STALE_MS;
}

static bool nav_rth_can_start(void) {
  return state.rth_state != RTH_STATE_ABORTED &&
         nav.home_valid &&
         nav.gps.valid &&
         nav_altitude_source_ok() &&
         state.home_distance >= RTH_MIN_DISTANCE &&
         profile.navigation.rth_throttle_min < profile.navigation.rth_throttle_max;
}

static float nav_slew(float current, float target, float step) {
  return constrain(target, current - step, current + step);
}

static void nav_update_altitude_control(float target_alt, float max_rate, float dt) {
  const float error = target_alt - state.altitude;
  const float configured_throttle_min = constrain(profile.navigation.rth_throttle_min, 0.0f, 1.0f);
  const float configured_throttle_hover = constrain(profile.navigation.rth_throttle_hover, configured_throttle_min, 1.0f);
  const float throttle_min = configured_throttle_min;
  const float throttle_ceiling = MAX(throttle_min, 1.0f - RTH_THROTTLE_HEADROOM);
  const float throttle_max = constrain(profile.navigation.rth_throttle_max, throttle_min, throttle_ceiling);
  const float throttle_hover = constrain(configured_throttle_hover, throttle_min, throttle_max);
  const float safe_dt = MAX(dt, 0.001f);

  const float desired_rate = constrain(ALT_POSITION_KP * error, -max_rate, max_rate);
  rth.vertical.desired_rate = nav_slew(rth.vertical.desired_rate, desired_rate, ALT_ACCEL_LIMIT * safe_dt);
  const float rate_error = rth.vertical.desired_rate - state.baro_vertical_speed;
  const float tilt = MAX(cosf(state.attitude.roll) * cosf(state.attitude.pitch), 0.5f);
  // A low-hover-throttle craft gets more acceleration per throttle unit.
  // Scale the velocity gains with the trimmed hover estimate to compensate.
  const float effective_hover = constrain(throttle_hover + rth.vertical.integral, MAX(throttle_min, 0.05f), throttle_max);
  // Allow the integral to compensate a wrong hover setting across the usable
  // throttle range. A fixed +/-0.2 limit cannot trim 0.5 down to a 0.2 hover.
  const float integral = constrain(rth.vertical.integral + ALT_KI * effective_hover * rate_error * safe_dt,
                                   throttle_min * tilt - throttle_hover,
                                   throttle_max * tilt - throttle_hover);
  const float throttle = (throttle_hover + ALT_KP * effective_hover * rate_error + integral) / tilt;
  // The slew limiter is an actuator limit too. Do not wind up while the
  // applied throttle is still catching up with the proportional correction.
  const float throttle_step = RTH_THROTTLE_SLEW_RATE * safe_dt;
  const float applied_min = constrain(rth.vertical.throttle_command - throttle_step, throttle_min, throttle_max);
  const float applied_max = constrain(rth.vertical.throttle_command + throttle_step, throttle_min, throttle_max);
  if ((throttle >= applied_min && throttle <= applied_max) ||
      (throttle > applied_max && rate_error < 0.0f) ||
      (throttle < applied_min && rate_error > 0.0f)) {
    rth.vertical.integral = integral;
  }
  const float throttle_target = (throttle_hover + ALT_KP * effective_hover * rate_error + rth.vertical.integral) / tilt;
  rth.vertical.throttle_command = constrain(throttle_target, applied_min, applied_max);
  state.rx_override.throttle = constrain(rth.vertical.throttle_command, throttle_min, throttle_max);
}

static void nav_update_acceleration() {
  // Differentiate measurements at GPS cadence, not at the 100 Hz control rate.
  if (rth.horizontal.sample_valid && state.gps_last_update_ms == rth.horizontal.updated_ms) {
    return;
  }

  const uint32_t elapsed_ms = state.gps_last_update_ms - rth.horizontal.updated_ms;
  if (rth.horizontal.sample_valid && elapsed_ms > 0 && elapsed_ms <= RTH_GPS_STALE_MS) {
    const float sample_dt = elapsed_ms * 0.001f;
    const float gain = sample_dt / (0.2f + sample_dt);
    rth.horizontal.acceleration_north += gain * ((state.gps_vel_north - rth.horizontal.velocity_north) / sample_dt - rth.horizontal.acceleration_north);
    rth.horizontal.acceleration_east += gain * ((state.gps_vel_east - rth.horizontal.velocity_east) / sample_dt - rth.horizontal.acceleration_east);
  } else {
    rth.horizontal.acceleration_north = rth.horizontal.acceleration_east = 0.0f;
  }
  rth.horizontal.velocity_north = state.gps_vel_north;
  rth.horizontal.velocity_east = state.gps_vel_east;
  rth.horizontal.updated_ms = state.gps_last_update_ms;
  rth.horizontal.sample_valid = true;
}

static float nav_heading_error() {
  float error = state.home_bearing - state.heading;
  while (error > 180.0f) error -= 360.0f;
  while (error < -180.0f) error += 360.0f;
  return error;
}

static void nav_update_yaw(float distance, float home_radius, float dt) {
  // RTH supplies radians/s directly; rx_override.yaw must bypass pilot rates.
  state.rx_override.yaw = 0.0f;
  if (distance <= home_radius) {
    state.rth_yaw_rate = 0.0f;
    return;
  }

  float heading_error = nav_heading_error();
  blackbox_set_debug(BBOX_DEBUG_NAVIGATION, 9, (int16_t)(heading_error * 10)); // 0.1 degrees

  if (fabsf(heading_error) < 2.0f) {
    heading_error = 0.0f;
  }

  // Yaw must work while stationary; GPS ground speed does not limit yaw authority.
  const float max_yaw_rate = RTH_MAX_YAW_RATE_LIMIT * DEGTORAD;
  const float yaw_rate = constrain(heading_error * DEGTORAD * RTH_YAW_KP, -max_yaw_rate, max_yaw_rate);
  state.rth_yaw_rate += dt / (0.03f + dt) * (yaw_rate - state.rth_yaw_rate);
}

static void nav_update_horizontal_control(float dt) {
  const float safe_dt = MAX(dt, 0.001f);
  float error_north, error_east;
  nav_position_delta(state.gps_coord, state.gps_home, &error_north, &error_east);
  const float distance = sqrtf(error_north * error_north + error_east * error_east);

  const float cruise_speed = MAX(profile.navigation.rth_cruise_speed, 0.1f);
  const float home_radius = RTH_HOME_RADIUS;
  // Reserve response time as well as braking distance: d = v*t + v^2/(2*a).
  // Slow toward the home center continuously, without a minimum approach speed.
  const float brake_delay_speed = RTH_NAV_BRAKE_ACCEL * RTH_NAV_BRAKE_DELAY;
  const float braking_speed = sqrtf(brake_delay_speed * brake_delay_speed +
                                   2.0f * RTH_NAV_BRAKE_ACCEL * distance) - brake_delay_speed;
  const float speed_limit = MIN(cruise_speed, braking_speed);

  float desired_vel_north = 0.0f;
  float desired_vel_east = 0.0f;
  if (distance > 0.01f) {
    const float desired_speed = MIN(speed_limit, distance * RTH_NAV_POSITION_KP);
    desired_vel_north = error_north * (desired_speed / distance);
    desired_vel_east = error_east * (desired_speed / distance);
  }

  const float velocity_step = RTH_NAV_ACCEL_LIMIT * safe_dt;
  rth.horizontal.desired_velocity_north = nav_slew(rth.horizontal.desired_velocity_north, desired_vel_north, velocity_step);
  rth.horizontal.desired_velocity_east = nav_slew(rth.horizontal.desired_velocity_east, desired_vel_east, velocity_step);
  desired_vel_north = rth.horizontal.desired_velocity_north;
  desired_vel_east = rth.horizontal.desired_velocity_east;

  const float vel_error_north = desired_vel_north - state.gps_vel_north;
  const float vel_error_east = desired_vel_east - state.gps_vel_east;

  const float heading_rad = state.heading * DEGTORAD;
  const float cos_heading = cosf(heading_rad);
  const float sin_heading = sinf(heading_rad);

  blackbox_set_debug(BBOX_DEBUG_NAVIGATION, 10, (int16_t)(desired_vel_north * 10));
  blackbox_set_debug(BBOX_DEBUG_NAVIGATION, 11, (int16_t)(desired_vel_east * 10));
  blackbox_set_debug(BBOX_DEBUG_NAVIGATION, 12, (int16_t)(state.gps_vel_north * 10));
  blackbox_set_debug(BBOX_DEBUG_NAVIGATION, 13, (int16_t)(state.gps_vel_east * 10));

  nav_update_acceleration();

  // Do not build a persistent tilt demand while heading recovery is needed.
  // Use actual progress along the home vector, independent of estimated yaw.
  const bool moving_away = state.rth_state == RTH_STATE_NAVIGATE &&
      state.gps_vel_north * error_north + state.gps_vel_east * error_east < 0.0f;
  if (moving_away) {
    const float decay = expf(-2.0f * safe_dt);
    rth.horizontal.integral_north *= decay;
    rth.horizontal.integral_east *= decay;
  }
  const float authority = 0.25f + 0.75f * state.heading_confidence;
  const float max_angle_rad = MAX(profile.rate.level_max_angle, 0.0f) * DEGTORAD * authority;
  const float north_pd = NAV_VEL_KP * vel_error_north - NAV_VEL_KD * rth.horizontal.acceleration_north;
  const float east_pd = NAV_VEL_KP * vel_error_east - NAV_VEL_KD * rth.horizontal.acceleration_east;
  const float north_i = constrain(rth.horizontal.integral_north + NAV_VEL_KI * vel_error_north * safe_dt,
                                 -max_angle_rad * 0.5f, max_angle_rad * 0.5f);
  const float east_i = constrain(rth.horizontal.integral_east + NAV_VEL_KI * vel_error_east * safe_dt,
                                -max_angle_rad * 0.5f, max_angle_rad * 0.5f);
  const float candidate_north = north_pd + north_i;
  const float candidate_east = east_pd + east_i;
  const float candidate_length = hypotf(candidate_north, candidate_east);
  if (!moving_away && (candidate_length <= max_angle_rad || candidate_north * vel_error_north + candidate_east * vel_error_east < 0.0f)) {
    rth.horizontal.integral_north = north_i;
    rth.horizontal.integral_east = east_i;
  }
  float north_angle = north_pd + rth.horizontal.integral_north;
  float east_angle = east_pd + rth.horizontal.integral_east;
  const float angle_scale = MIN(1.0f, max_angle_rad / MAX(hypotf(north_angle, east_angle), 0.001f));
  north_angle *= angle_scale;
  east_angle *= angle_scale;
  const float pitch_angle = north_angle * cos_heading + east_angle * sin_heading;
  const float roll_angle = east_angle * cos_heading - north_angle * sin_heading;

  // Angle mode interprets stick input using level_max_angle, so scale by that
  // rather than by the navigation angle limit.
  const float level_max_angle_rad = MAX(profile.rate.level_max_angle, 1.0f) * DEGTORAD;
  const float slew_step = RTH_ANGLE_SLEW_RATE * safe_dt;
  const float pitch_command = pitch_angle / level_max_angle_rad;
  const float roll_command = roll_angle / level_max_angle_rad;
  const float command_gain = safe_dt / (RTH_NAV_COMMAND_TIME_CONSTANT + safe_dt);
  rth.horizontal.pitch_command += constrain(command_gain * (pitch_command - rth.horizontal.pitch_command), -slew_step, slew_step);
  rth.horizontal.roll_command += constrain(command_gain * (roll_command - rth.horizontal.roll_command), -slew_step, slew_step);
  state.rx_override.pitch = rth.horizontal.pitch_command;
  state.rx_override.roll = rth.horizontal.roll_command;
  nav_update_yaw(distance, home_radius, safe_dt);
}

static void nav_rth_abort(void) {
  nav_rth_stop();
  // Do not automatically retry a failed rescue during the same signal loss.
  state.rth_state = RTH_STATE_ABORTED;
}

static void nav_reset_horizontal(void) {
  rth.horizontal = {};
  state.rth_yaw_rate = 0.0f;
  state.rx_override.roll = state.rx_override.pitch = state.rx_override.yaw = 0.0f;
}

static void nav_begin_turn(uint32_t now_ms) {
  state.rth_state = RTH_STATE_TURN;
  nav_reset_horizontal();
  rth.progress.updated_ms = now_ms;
}

static void nav_begin_heading_acquisition(uint32_t now_ms) {
  state.rth_state = RTH_STATE_ACQUIRE_HEADING;
  nav_reset_horizontal();
  rth.acquisition.started_ms = now_ms;
  rth.acquisition.origin = state.gps_coord;
}

static bool nav_update_progress(uint32_t now_ms) {
  // Allow acceleration at reduced heading confidence before declaring no progress.
  if (state.home_distance + RTH_PROGRESS_DISTANCE < rth.progress.best_error) {
    rth.progress.best_error = state.home_distance;
    rth.progress.updated_ms = now_ms;
  }
  return now_ms - rth.progress.updated_ms <= RTH_PROGRESS_TIMEOUT_MS;
}

static void nav_update_rth() {
  const uint32_t now = time_micros();
  const uint32_t now_ms = time_millis();
  const float dt = (now - rth.updated_us) * 1e-6f;
  rth.updated_us = now;

  if (!nav_altitude_source_ok()) {
    nav_rth_abort();
    return;
  }
  if (!nav.gps.valid) {
    if (!rth.gps_loss.active) {
      rth.gps_loss.active = true;
      rth.gps_loss.started_ms = now_ms;
      rth.gps_loss.altitude = state.altitude;
      nav_reset_horizontal();
    }
    if (now_ms - rth.gps_loss.started_ms >= RTH_GPS_LOSS_TIMEOUT_MS) {
      nav_rth_abort();
      return;
    }
    // Level and hold a latched altitude; horizontal position is unobservable.
    nav_update_altitude_control(rth.gps_loss.altitude, RTH_CLIMB_RATE, dt);
    return;
  }
  if (rth.gps_loss.active) {
    rth.gps_loss.active = false;
    rth.progress.updated_ms = now_ms;
    rth.progress.best_error = state.rth_state == RTH_STATE_CLIMB
        ? fabsf(rth.target_altitude - state.altitude)
        : state.home_distance;
  }

  switch (state.rth_state) {
  case RTH_STATE_ABORTED:
  case RTH_STATE_INACTIVE:
    break;

  case RTH_STATE_CLIMB: {
    nav_update_altitude_control(rth.target_altitude, RTH_CLIMB_RATE, dt);

    // Hold level while climbing; horizontal navigation starts after target altitude.
    state.rx_override.roll = 0.0f;
    state.rx_override.pitch = 0.0f;
    state.rx_override.yaw = 0.0f;

    const float altitude_error = fabsf(state.altitude - rth.target_altitude);
    if (altitude_error + 0.5f < rth.progress.best_error) {
      rth.progress.best_error = altitude_error;
      rth.progress.updated_ms = now_ms;
    } else if (now_ms - rth.progress.updated_ms > RTH_CLIMB_TIMEOUT_MS) {
      nav_rth_abort();
      return;
    }
    if (altitude_error < 1.0f) {
      if (state.heading_confidence < RTH_MIN_HEADING_CONFIDENCE)
        nav_begin_heading_acquisition(now_ms);
      else
        nav_begin_turn(now_ms);
    }
    break;
  }
  case RTH_STATE_ACQUIRE_HEADING: {
    nav_update_altitude_control(rth.target_altitude, RTH_DESCENT_RATE, dt);
    // No earth-frame steering until heading is observable. Zero yaw rate holds
    // the physical orientation without chasing GPS corrections to the estimate.
    state.rx_override.roll = state.rx_override.yaw = state.rth_yaw_rate = 0.0f;
    const float north = (int64_t(state.gps_coord.lat) - rth.acquisition.origin.lat) * (METERS_PER_DEGREE_LAT / 1e7f);
    const float east = (int64_t(state.gps_coord.lon) - rth.acquisition.origin.lon) * (METERS_PER_DEGREE_LAT / 1e7f) *
                       cosf(rth.acquisition.origin.lat * (DEGTORAD / 1e7f));
    if (now_ms - rth.acquisition.started_ms >= RTH_ACQUIRE_TIMEOUT_MS ||
        north * north + east * east >= RTH_ACQUIRE_DISTANCE_M * RTH_ACQUIRE_DISTANCE_M) {
      nav_reset_horizontal();
      state.rth_state = RTH_STATE_HEADING_FAILED;
      rth.target_altitude = state.altitude;
    } else if (state.heading_confidence >= RTH_ACQUIRE_CONFIDENCE && !state.heading_correction_flags) {
      nav_begin_turn(now_ms);
    } else {
      const float pitch = constrain(profile.rate.level_max_angle, 0.0f, RTH_ACQUIRE_PITCH_DEG) /
                          MAX(profile.rate.level_max_angle, 1.0f);
      rth.horizontal.pitch_command = nav_slew(rth.horizontal.pitch_command, pitch,
          RTH_ACQUIRE_SLEW_RATE * MAX(dt, 0.001f));
      state.rx_override.pitch = rth.horizontal.pitch_command;
    }
    break;
  }
  case RTH_STATE_HEADING_FAILED:
    // Acquisition failed: keep altitude control and level attitude. In
    // failsafe, never turn a heading-only failure into an immediate motor drop.
    nav_reset_horizontal();
    nav_update_altitude_control(rth.target_altitude, RTH_DESCENT_RATE, dt);
    break;
  case RTH_STATE_TURN: {
    if (state.heading_confidence < RTH_MIN_HEADING_CONFIDENCE) {
      nav_begin_heading_acquisition(now_ms);
      return;
    }
    nav_update_altitude_control(rth.target_altitude, RTH_DESCENT_RATE, dt);
    // Hold level until the nose points home and the turn has slowed down.
    state.rx_override.roll = state.rx_override.pitch = 0.0f;
    nav_update_yaw(state.home_distance, RTH_HOME_RADIUS, MAX(dt, 0.001f));
    const bool at_home = state.home_distance < RTH_HOME_RADIUS;
    const bool aligned = fabsf(nav_heading_error()) < RTH_TURN_ERROR_DEG &&
                         fabsf(state.gyro.yaw) < RTH_TURN_RATE_DEG * DEGTORAD;
    if (at_home || aligned) {
      state.rth_state = at_home ? RTH_STATE_HOVER_HOME : RTH_STATE_NAVIGATE;
      rth.progress.best_error = state.home_distance;
      rth.progress.updated_ms = now_ms;
    } else {
      // Allow a half-turn at the yaw-rate limit plus ten seconds to settle.
      const uint32_t timeout_ms = 10000U + (uint32_t)(180000.0f / RTH_MAX_YAW_RATE_LIMIT);
      if (now_ms - rth.progress.updated_ms > timeout_ms) {
        nav_rth_abort();
      }
    }
    break;
  }
  case RTH_STATE_NAVIGATE:
    if (state.home_distance >= RTH_HOME_RADIUS && !nav_update_progress(now_ms)) {
      nav_rth_abort();
      return;
    }
    nav_update_altitude_control(rth.target_altitude, RTH_DESCENT_RATE, dt);
    if (state.home_distance > 2.0f * RTH_HOME_RADIUS &&
        fabsf(nav_heading_error()) > RTH_REALIGN_ERROR_DEG) {
      nav_begin_turn(now_ms);
      return;
    }
    nav_update_horizontal_control(dt);

    if (state.home_distance < RTH_HOME_RADIUS) {
      state.rth_state = RTH_STATE_HOVER_HOME;
    }
    break;

  case RTH_STATE_HOVER_HOME:
    // Stationary hover is success. Resume progress supervision if blown out.
    if (state.home_distance > 2.0f * RTH_HOME_RADIUS) {
      nav_begin_turn(now_ms);
      nav_update_altitude_control(rth.target_altitude, RTH_DESCENT_RATE, dt);
      return;
    }
    nav_update_altitude_control(rth.target_altitude, RTH_DESCENT_RATE, dt);
    nav_update_horizontal_control(dt);
    break;
  }
}

void nav_rth_start() {
  if (state.rth_active || !nav_rth_can_start()) {
    return;
  }

  rth = {};
  state.rth_active = true;
  state.rth_failsafe_active = profile.navigation.rth_on_failsafe && flags.failsafe_signal_lost;
  state.rth_state = RTH_STATE_CLIMB;
  rth.target_altitude = state.altitude + profile.navigation.rth_altitude;
  rth.progress.best_error = fabsf(profile.navigation.rth_altitude);
  rth.vertical.throttle_command = constrain(state.throttle, profile.navigation.rth_throttle_min, profile.navigation.rth_throttle_max);
  // Seed controller trim from the currently applied throttle for a smooth
  // takeover. This is an initial condition, not a learned hover measurement.
  const float tilt = MAX(cosf(state.attitude.roll) * cosf(state.attitude.pitch), 0.5f);
  rth.vertical.integral = rth.vertical.throttle_command * tilt - profile.navigation.rth_throttle_hover;
  rth.vertical.desired_rate = constrain(state.baro_vertical_speed, -RTH_DESCENT_RATE, RTH_CLIMB_RATE);
  rth.updated_us = time_micros();
  rth.progress.updated_ms = time_millis();
  nav_reset_horizontal();
  state.rx_override.throttle = rth.vertical.throttle_command;

  flags.controls_override = 1;
}

void nav_rth_stop() {
  if (state.rth_active) {
    flags.controls_override = 0;
    state.rx_override = (vec4_t){0};
  }
  rth = {};
  state.rth_state = RTH_STATE_INACTIVE;
  state.rth_active = false;
  state.rth_failsafe_active = false;
  state.rth_yaw_rate = 0.0f;
}

static void nav_update_request() {
  // RTH control via AUX channel
  const uint8_t rth_aux = rx_aux_on(AUX_RETURN_TO_HOME);
  if (rth_aux != nav.previous.rth_aux) {
    if (rth_aux) {
      nav_rth_start();
    } else if (!flags.failsafe_signal_lost) {
      nav_rth_stop();
    }
    nav.previous.rth_aux = rth_aux;
  }

  // Check for failsafe RTH
  if (profile.navigation.rth_on_failsafe) {
    if (flags.failsafe && !state.rth_active && nav.home_valid) {
      nav_rth_start();
    } else if (!flags.failsafe && state.rth_active && nav.previous.failsafe && !rth_aux) {
      // A manual RTH request keeps control after receiver recovery.
      nav_rth_stop();
    }
    nav.previous.failsafe = flags.failsafe;
  }
}

void nav_update() {
  nav_update_gps_sanity();

  if (flags.arm_state != nav.previous.armed) {
    if (flags.arm_state) {
      nav.home_valid = nav.gps.valid;
      if (nav.home_valid)
        state.gps_home = state.gps_coord;
    }
    nav.previous.armed = flags.arm_state;
  }

  if (!flags.arm_state) {
    state.gps_home = state.gps_coord;
    nav.home_valid = false;

    nav_rth_stop();
    nav.previous.rth_aux = nav.previous.failsafe = 0;
  } else {
    if (nav.gps.valid && nav.home_valid) {
      nav_update_gps(state.gps_coord, state.gps_home);
    }

    nav_update_request();
    if (state.rth_active) {
      nav_update_rth();
    }
  }
  // Navigation debug: 0 heading, 1 GPS course, 2 altitude (dm), 3 confidence.
  blackbox_set_debug(BBOX_DEBUG_NAVIGATION, 4, (int16_t)(state.rth_state));
  blackbox_set_debug(BBOX_DEBUG_NAVIGATION, 5, (int16_t)(state.home_distance)); // meters
  blackbox_set_debug(BBOX_DEBUG_NAVIGATION, 6, (int16_t)(state.home_bearing * 10)); // 0.1 deg
  // Publish applied requests in every phase, including acquisition and failure.
  blackbox_set_debug(BBOX_DEBUG_NAVIGATION, 14, (int16_t)(state.rx_override.pitch * 1000));
  blackbox_set_debug(BBOX_DEBUG_NAVIGATION, 15, (int16_t)(state.rx_override.roll * 1000));
}

#ifdef PIO_UNIT_TESTING
void nav_test_reset(void) {
  nav_rth_stop();
  nav = {};
}

void nav_test_update_horizontal_control(float dt) {
  nav_update_horizontal_control(dt);
}

void nav_test_update_gps_sanity(void) {
  nav_update_gps_sanity();
}

bool nav_test_gps_sane(void) {
  return nav.gps.valid;
}

void nav_test_set_home_valid(bool valid) {
  nav.home_valid = valid;
}

void nav_test_set_gps_sane(bool sane) {
  nav.gps.valid = sane;
}

void nav_test_set_rth_active(bool active) {
  state.rth_active = active;
  state.rth_state = active ? RTH_STATE_CLIMB : RTH_STATE_INACTIVE;
}

void nav_test_update_rth(void) { nav_update_rth(); }
void nav_test_altitude_control(float target, float rate, float dt) {
  nav_update_altitude_control(target, rate, dt);
}
#endif
