#include "control/multi/nav_control.h"

#include <math.h>

#include "control/control.h"
#include "control/imu.h"
#include "control/navigation.h"
#include "core/profile.h"
#include "io/blackbox.h"
#include "util/util.h"
#include "util/vector.h"

// Vertical control.
static constexpr float ALT_KP = 0.15f;               // relative to the estimated level hover throttle
static constexpr float ALT_KI = 0.05f;
static constexpr float ALT_POSITION_KP = 0.5f;       // m/s per meter of altitude error
static constexpr float ALT_HOLD_BAND = 1.0f;         // meters, the altitude needs no tighter hold
static constexpr float ALT_HOLD_SCALE = 0.3f;        // position gain within the band
static constexpr float ALT_ACCEL_LIMIT = 1.0f;       // m/s^2 target vertical acceleration
static constexpr float ALT_BRAKE_ACCEL_LIMIT = 2.5f; // m/s^2, stopping a climb or descent is never smoothed as gently
static constexpr float ALT_JERK_LIMIT = 4.0f;        // m/s^3 target vertical acceleration change
static constexpr float THROTTLE_HEADROOM = 0.15f;    // kept above the navigation throttle for attitude authority
static constexpr float THROTTLE_SLEW_RATE = 1.5f;    // per second

// Hover throttle learning.
static constexpr float HOVER_LEARN_TIME = 3.0f;         // seconds, estimate time constant
static constexpr float HOVER_LEARN_MIN_TIME = 2.0f;     // seconds of steady flight before the estimate is used
static constexpr float HOVER_LEARN_MIN_TILT = 0.94f;    // cos(20 degrees)
static constexpr float HOVER_LEARN_MAX_SPEED = 0.5f;    // m/s vertical
static constexpr float HOVER_LEARN_MIN_ALTITUDE = 1.5f; // meters above launch, clear of the ground and ground effect
static constexpr float HOVER_LEARN_MAX_ERROR = 0.1f;    // trust learned hover only this close to the takeover estimate
// Throttle range that learns: outside it the motors may saturate or idle.
static constexpr float HOVER_LEARN_THROTTLE_MIN = 0.05f;
static constexpr float HOVER_LEARN_THROTTLE_MAX = 0.95f;
// Lowest trimmed hover the vertical gains scale with.
static constexpr float EFFECTIVE_HOVER_MIN = 0.05f;

// Horizontal velocity control.
static constexpr float VEL_KP = 0.08f;
static constexpr float VEL_KI = 0.02f;
static constexpr float RATE_SETTLED = 0.02f;           // m/s, a shaped rate this close snaps onto its request
static constexpr float VEL_AWAY_DECAY_RATE = 2.0f;     // per second, integral decay while moving away
static constexpr float COMMAND_TIME_CONSTANT = 0.15f;  // seconds, smooth GPS-driven tilt changes
static constexpr float MIN_ANGLE_AUTHORITY = 0.25f;    // share of the tilt limit without heading confidence

// Along-track velocity gains at cruise speed, relative to VEL_KP and VEL_KI,
// ramped in between the two speeds. Slow flight and cross-track keep the full
// gains. An integral as fast as the reduced proportional term makes the speed
// loop resonate, pitching back and forth over seconds; it trims slowly within
// VEL_CRUISE_KI_BAND and at the full rate beyond.
static constexpr float VEL_CRUISE_KP_SCALE = 0.5f;
static constexpr float VEL_CRUISE_KI_SCALE = 0.75f;
static constexpr float VEL_CRUISE_KI_BAND = 1.0f;    // m/s along-track
static constexpr float VEL_CRUISE_SPEED_MIN = 2.0f;  // m/s
static constexpr float VEL_CRUISE_SPEED_FULL = 5.0f; // m/s

// Steady cruise moves the along-track integral into a drag trim over this
// time. Headwind learns as drag too; the integral corrects it on other legs.
static constexpr float DRAG_LEARN_TIME = 3.0f; // seconds
static constexpr float DRAG_LEARN_ERROR = 1.0f; // m/s along-track
static constexpr float DRAG_TRIM_MAX = 0.1f;    // radians per m/s

// Learned in flight and kept across engagements; disarming starts them over.
static struct {
  float hover_learned_time; // seconds of steady flight in state.hover_throttle
  float drag_trim;          // radians of tilt per m/s of desired velocity, learned in cruise
} trim;

nav_altitude_control_t nav_alt;
nav_velocity_control_t nav_velocity;

float nav_tilt() {
  return MAX(fastcos(state.attitude.roll) * fastcos(state.attitude.pitch), 0.5f);
}

static bool nav_hover_learned() {
  return trim.hover_learned_time >= HOVER_LEARN_MIN_TIME;
}

void nav_update_trims(float dt) {
  if (!flags.arm_state) {
    // The next flight may have a different battery, payload or airframe drag.
    trim = {};
    state.hover_throttle = 0.0f;
    return;
  }
  // in_air and on_ground do not prove the vehicle has lifted off; low
  // throttle on the ground would otherwise learn as a low hover.
  const bool airborne = flags.in_air && !flags.on_ground &&
                        flags.altitude_valid && state.altitude > HOVER_LEARN_MIN_ALTITUDE;
  const float tilt = nav_tilt();
  const bool steady = airborne && !flags.controls_override && !flags.failsafe &&
                      state.throttle > HOVER_LEARN_THROTTLE_MIN && state.throttle < HOVER_LEARN_THROTTLE_MAX &&
                      tilt > HOVER_LEARN_MIN_TILT &&
                      fabsf(state.velocity.up) < HOVER_LEARN_MAX_SPEED;
  if (!steady)
    return;
  const float sample = state.throttle * tilt;
  if (trim.hover_learned_time <= 0.0f)
    state.hover_throttle = sample;
  else
    state.hover_throttle += dt / (HOVER_LEARN_TIME + dt) * (sample - state.hover_throttle);
  trim.hover_learned_time = MIN(trim.hover_learned_time + dt, HOVER_LEARN_MIN_TIME);
}

float nav_throttle_min() {
  return constrain(profile.navigation.throttle_min, 0.0f, 1.0f);
}

float nav_throttle_max(float throttle_min) {
  return constrain(profile.navigation.throttle_max, throttle_min, MAX(throttle_min, 1.0f - THROTTLE_HEADROOM));
}

float nav_hover_throttle() {
  const float hover = nav_hover_learned() ? state.hover_throttle : profile.navigation.throttle_hover;
  return constrain(hover, nav_throttle_min(), nav_throttle_max());
}

void nav_altitude_start(float throttle) {
  const float throttle_min = nav_throttle_min();
  const float throttle_max = nav_throttle_max();
  nav_alt.throttle_command = constrain(throttle, throttle_min, throttle_max);
  // Infer hover from the applied throttle and the acceleration it produces
  // (thrust ~ throttle), so a dive does not read as a low hover.
  const float accel_ratio = constrain(vec3_dot(state.accel, state.GEstG) / (ACC_1G * ACC_1G), 0.5f, 1.5f);
  const float hover = constrain(throttle * nav_tilt() / accel_ratio, throttle_min, throttle_max);
  if (nav_hover_learned() && fabsf(state.hover_throttle - hover) <= HOVER_LEARN_MAX_ERROR) {
    // A learned hover that agrees with the takeover is the steadier trim.
    nav_alt.integral = 0.0f;
  } else {
    nav_alt.integral = hover - nav_hover_throttle();
  }
  nav_alt.desired_rate = constrain(state.velocity.up, -ALT_MAX_RATE, ALT_MAX_RATE);
  nav_alt.desired_accel = 0.0f;
  state.rx_override.throttle = nav_alt.throttle_command;
}

// How fast to close error: at gain p near zero and, farther out, as fast as
// still stops with limit, the second derivative. The two meet with one slope.
static float nav_sqrt_controller(float error, float p, float limit) {
  const float linear = limit / (p * p);
  const float magnitude = fabsf(error);
  const float output = magnitude <= linear ? p * magnitude : sqrtf(2.0f * limit * (magnitude - 0.5f * linear));
  return copysignf(output, error);
}

// Ease rate onto request: accelerate at most accel_max, changing that at
// most jerk, toward the acceleration that still eases out on the request.
// Below accel_max the target then changes no faster than jerk, so it is
// followed without overshoot. Settled, rate snaps onto request exactly.
static void nav_shape_rate(float *rate, float *accel, float request, float accel_max, float jerk, float dt) {
  const float kp = jerk / accel_max;
  const float error = request - *rate;
  if (fabsf(error) < RATE_SETTLED && fabsf(*accel) < kp * RATE_SETTLED) {
    *rate = request;
    *accel = 0.0f;
    return;
  }
  const float target = constrain(nav_sqrt_controller(error, kp, jerk), -accel_max, accel_max);
  *accel = slew(*accel, target, jerk * dt);
  *rate += *accel * dt;
}

void nav_update_altitude_control_above(float target_alt, float dt, float target_rate, float max_rate, float throttle_min) {
  const float throttle_max = nav_throttle_max(throttle_min);
  const float throttle_hover = constrain(nav_hover_throttle(), throttle_min, throttle_max);

  // Within ALT_HOLD_BAND, close the error gently instead of chasing barometer
  // noise with throttle; beyond it, the full gain applies to the excess.
  const float error = target_alt - state.altitude;
  const float hold_error = error - (1.0f - ALT_HOLD_SCALE) * constrain(error, -ALT_HOLD_BAND, ALT_HOLD_BAND);
  // Without altitude, hold vertical speed at zero on the accelerometer estimate.
  const float desired_rate = flags.altitude_valid ? constrain(target_rate + ALT_POSITION_KP * hold_error, -max_rate, max_rate) : 0.0f;
  // A takeover while climbing hard otherwise keeps climbing until the slew passes zero.
  const bool braking = (nav_alt.desired_rate < 0.0f && desired_rate > nav_alt.desired_rate) ||
                       (nav_alt.desired_rate > 0.0f && desired_rate < nav_alt.desired_rate);
  nav_shape_rate(&nav_alt.desired_rate, &nav_alt.desired_accel, desired_rate, braking ? ALT_BRAKE_ACCEL_LIMIT : ALT_ACCEL_LIMIT,
                 ALT_JERK_LIMIT, dt);
  const float rate_error = nav_alt.desired_rate - state.velocity.up;

  // A low-hover-throttle craft gets more acceleration per throttle unit.
  // Scale the velocity gains with the trimmed hover estimate to compensate.
  const float tilt = nav_tilt();
  const float effective_hover = constrain(throttle_hover + nav_alt.integral, MAX(throttle_min, EFFECTIVE_HOVER_MIN), throttle_max);
  // Thrust for the shaped acceleration, so the rate loop only trims what it misses.
  const float feedforward = effective_hover * nav_alt.desired_accel / GRAVITY;
  const auto throttle_for = [&](float integral) {
    return (throttle_hover + feedforward + ALT_KP * effective_hover * rate_error + integral) / tilt;
  };
  // Allow the integral to compensate a wrong hover setting across the usable
  // throttle range. A fixed +/-0.2 limit cannot trim 0.5 down to a 0.2 hover.
  const float integral = constrain(nav_alt.integral + ALT_KI * effective_hover * rate_error * dt,
                                   throttle_min * tilt - throttle_hover,
                                   throttle_max * tilt - throttle_hover);
  // The slew limiter is an actuator limit too. Do not wind up while the
  // applied throttle is still catching up with the proportional correction.
  // Without a barometer the velocity estimate drifts, so freeze the trim.
  const float throttle_step = THROTTLE_SLEW_RATE * dt;
  const float applied_min = constrain(nav_alt.throttle_command - throttle_step, throttle_min, throttle_max);
  const float applied_max = constrain(nav_alt.throttle_command + throttle_step, throttle_min, throttle_max);
  const float throttle = throttle_for(integral);
  if (flags.altitude_valid &&
      ((throttle >= applied_min && throttle <= applied_max) ||
       (throttle > applied_max && rate_error < 0.0f) ||
       (throttle < applied_min && rate_error > 0.0f))) {
    nav_alt.integral = integral;
  }
  nav_alt.throttle_command = constrain(throttle_for(nav_alt.integral), applied_min, applied_max);
  state.rx_override.throttle = nav_alt.throttle_command;
}

void nav_update_altitude_control(float target_alt, float dt, float target_rate) {
  nav_update_altitude_control_above(target_alt, dt, target_rate, ALT_MAX_RATE, nav_throttle_min());
}

// Ease the desired earth-frame velocity onto the request, as nav_shape_rate
// does along the velocity error, keeping the acceleration's direction.
static void nav_shape_velocity(vec2_t request, float dt) {
  auto &horizontal = nav_velocity;
  const vec2_t error_vector = request - horizontal.desired_velocity;
  const float error = vec2_magnitude(error_vector);
  constexpr float kp = VEL_JERK_LIMIT / VEL_ACCEL_LIMIT;
  if (error < RATE_SETTLED && vec2_magnitude(horizontal.desired_accel) < kp * RATE_SETTLED) {
    horizontal.desired_velocity = request;
    horizontal.desired_accel = {};
    return;
  }
  const float accel = MIN(nav_sqrt_controller(error, kp, VEL_JERK_LIMIT), VEL_ACCEL_LIMIT);
  const vec2_t change = error_vector * (accel / MAX(error, 0.0001f)) - horizontal.desired_accel;
  horizontal.desired_accel += change * MIN(1.0f, VEL_JERK_LIMIT * dt / MAX(vec2_magnitude(change), 0.0001f));
  horizontal.desired_velocity += horizontal.desired_accel * dt;
}

void nav_update_velocity_control(vec2_t request, bool moving_away, float authority_scale, float dt) {
  auto &horizontal = nav_velocity;
  nav_shape_velocity(request, dt);
  const vec2_t desired = horizontal.desired_velocity;
  const vec2_t velocity = vec3_horizontal(state.velocity);
  const vec2_t error = desired - velocity;

  blackbox_set_debug(BBOX_DEBUG_NAVIGATION, 10, (int16_t)(desired.north * 10));
  blackbox_set_debug(BBOX_DEBUG_NAVIGATION, 11, (int16_t)(desired.east * 10));
  blackbox_set_debug(BBOX_DEBUG_NAVIGATION, 12, (int16_t)(velocity.north * 10));
  blackbox_set_debug(BBOX_DEBUG_NAVIGATION, 13, (int16_t)(velocity.east * 10));

  if (moving_away)
    horizontal.integral *= expf(-VEL_AWAY_DECAY_RATE * dt);
  const float authority = authority_scale * (MIN_ANGLE_AUTHORITY + (1.0f - MIN_ANGLE_AUTHORITY) * state.heading_confidence);
  const float max_angle = MAX(profile.rate.level_max_angle, 0.0f) * DEGTORAD * authority;

  // At cruise speed, the exact speed matters less than a steady one. Scale the
  // along-track gains down there; cross-track and slow flight keep them.
  const float desired_speed = vec2_magnitude(desired);
  const vec2_t along = desired_speed > 0.01f ? desired * (1.0f / desired_speed) : vec2_t{};
  const float along_error = vec2_dot(error, along);
  const float cruise = constrain((desired_speed - VEL_CRUISE_SPEED_MIN) / (VEL_CRUISE_SPEED_FULL - VEL_CRUISE_SPEED_MIN), 0.0f, 1.0f);
  const float p_reduction = (1.0f - VEL_CRUISE_KP_SCALE) * cruise * along_error;
  // Only small speed errors trim slowly: a new leg or a lasting change of wind
  // still trims at the full rate, so the speed cannot settle far off.
  const float i_reduction = (1.0f - VEL_CRUISE_KI_SCALE) * cruise * constrain(along_error, -VEL_CRUISE_KI_BAND, VEL_CRUISE_KI_BAND);
  // Drag tilt follows the desired velocity: it turns with each leg and fades
  // while braking, leaving the earth-frame integral to trim the wind.
  // The tilt for the shaped acceleration flies it directly; the PI trims
  // what it misses.
  const float accel = vec2_magnitude(horizontal.desired_accel);
  const float accel_tilt = accel > 0.001f ? atan2approx_rad(accel, GRAVITY) / accel : 1.0f / GRAVITY;
  const vec2_t p = VEL_KP * (error - along * p_reduction) + trim.drag_trim * desired + accel_tilt * horizontal.desired_accel;
  // Cruise drag into a headwind can need most of the tilt: the trim may use
  // all of it rather than leaving the proportional term a standing error.
  vec2_t integral = horizontal.integral + VEL_KI * dt * (error - along * i_reduction);
  integral *= MIN(1.0f, max_angle / MAX(vec2_magnitude(integral), 0.001f));
  const vec2_t candidate = p + integral;
  if (!moving_away && (vec2_magnitude(candidate) <= max_angle || vec2_dot(candidate, error) < 0.0f))
    horizontal.integral = integral;
  if (cruise > 0.0f && !moving_away && fabsf(along_error) < DRAG_LEARN_ERROR) {
    // Move the along-track trim into the drag trim; the total tilt is unchanged.
    const float along_integral = vec2_dot(horizontal.integral, along);
    const float drag_trim = constrain(trim.drag_trim + dt / (DRAG_LEARN_TIME + dt) * along_integral / desired_speed,
                                      0.0f, DRAG_TRIM_MAX);
    const float moved = (drag_trim - trim.drag_trim) * desired_speed;
    trim.drag_trim = drag_trim;
    horizontal.integral = horizontal.integral - along * moved;
  }
  vec2_t angle = p + horizontal.integral;
  angle *= MIN(1.0f, max_angle / MAX(vec2_magnitude(angle), 0.001f));
  const float cos_heading = fastcos(state.heading * DEGTORAD);
  const float sin_heading = fastsin(state.heading * DEGTORAD);
  const float pitch_angle = angle.north * cos_heading + angle.east * sin_heading;
  const float roll_angle = angle.east * cos_heading - angle.north * sin_heading;

  // Angle mode interprets stick input using level_max_angle, so scale by that
  // rather than by the navigation angle limit.
  const float level_max_angle = MAX(profile.rate.level_max_angle, 1.0f) * DEGTORAD;
  const float slew_step = ANGLE_SLEW_RATE * dt;
  const float command_gain = dt / (COMMAND_TIME_CONSTANT + dt);
  horizontal.pitch_command += constrain(command_gain * (pitch_angle / level_max_angle - horizontal.pitch_command), -slew_step, slew_step);
  horizontal.roll_command += constrain(command_gain * (roll_angle / level_max_angle - horizontal.roll_command), -slew_step, slew_step);
  state.rx_override.pitch = horizontal.pitch_command;
  state.rx_override.roll = horizontal.roll_command;
}

#ifdef PIO_UNIT_TESTING
void nav_test_reset_control() {
  trim = {};
  nav_alt = {};
  nav_velocity = {};
}

void nav_test_set_hover_learned(float throttle) {
  state.hover_throttle = throttle;
  trim.hover_learned_time = throttle > 0.0f ? HOVER_LEARN_MIN_TIME : 0.0f;
}
#endif
