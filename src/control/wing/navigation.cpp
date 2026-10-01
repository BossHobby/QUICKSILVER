#include "control/wing/navigation.h"

#include <math.h>

#include "control/control.h"
#include "control/navigation.h"
#include "core/profile.h"
#include "driver/time.h"
#include "io/blackbox.h"
#include "rx/rx.h"
#include "util/util.h"

#define GRAVITY 9.80665f                     // m/s^2
#define LOITER_L1_PERIOD 15.0f               // seconds, lateral guidance response period
#define LOITER_L1_DAMPING 0.75f
#define LOITER_CENTER_MIN_SPEED 3.0f         // m/s, slower GPS course does not orient the circle
#define LOITER_FALLBACK_BANK (20.0f * DEGTORAD)
#define LOITER_BANK_SLEW_RATE (45.0f * DEGTORAD) // rad/s
#define LOITER_PITCH_SLEW_RATE (20.0f * DEGTORAD) // rad/s
#define LOITER_THROTTLE_SLEW_RATE 0.5f       // per second

wing_nav_command_t wing_nav_command;

static struct {
  gps_coord_t origin; // GPS position when the circle was placed
  float center_north; // circle center relative to origin, meters
  float center_east;
  uint32_t updated_us;
} loiter;

static float nav_slew(float current, float target, float step) {
  return constrain(target, current - step, current + step);
}

static float loiter_direction() {
  return profile.wing.navigation.loiter_direction == WING_LOITER_LEFT ? -1.0f : 1.0f;
}

static bool loiter_requested() {
  // Launch owns attitude and throttle until it finishes; failsafe owns them
  // from stage 1. Loiter never starts on the ground.
  const bool launching = state.wing_launch_available && state.wing_launch_state < WING_LAUNCH_DONE;
  return rx_aux_on(AUX_LOITER) && flags.arm_state && flags.in_air && !flags.failsafe && !launching;
}

static void loiter_place_circle() {
  // Offset the center to the turn side of the ground track so the aircraft
  // rolls straight onto the circle instead of first flying out from its center.
  loiter.origin = state.gps_coord;
  loiter.center_north = 0.0f;
  loiter.center_east = 0.0f;
  const float speed = hypotf(state.gps_vel_north, state.gps_vel_east);
  if (speed < LOITER_CENTER_MIN_SPEED)
    return;
  const float scale = loiter_direction() * profile.wing.navigation.loiter_radius / speed;
  loiter.center_north = -state.gps_vel_east * scale;
  loiter.center_east = state.gps_vel_north * scale;
}

// L1 loiter guidance on the GPS ground track, as in ArduPlane's
// AP_L1_Control::update_loiter(). Returns lateral acceleration in m/s^2,
// positive to the right.
static float loiter_lateral_accel() {
  float north, east;
  nav_position_delta(loiter.origin, state.gps_coord, &north, &east);
  north -= loiter.center_north;
  east -= loiter.center_east;
  const float distance = hypotf(north, east);

  const float vel_north = state.gps_vel_north;
  const float vel_east = state.gps_vel_east;
  const float speed = hypotf(vel_north, vel_east);

  // Unit vector from the center to the aircraft.
  float radial_north = 1.0f;
  float radial_east = 0.0f;
  if (distance > 0.1f) {
    radial_north = north / distance;
    radial_east = east / distance;
  } else if (speed > 0.1f) {
    radial_north = vel_north / speed;
    radial_east = vel_east / speed;
  }

  const float direction = loiter_direction();
  const float radius = MAX(profile.wing.navigation.loiter_radius, 1.0f);
  const float omega = 2.0f * M_PI_F / LOITER_L1_PERIOD;
  const float l1_distance = MAX(LOITER_L1_DAMPING * LOITER_L1_PERIOD * speed / M_PI_F, 1.0f);

  // Capture: steer the velocity vector toward the center from outside.
  const float cross_velocity = radial_north * vel_east - radial_east * vel_north;
  const float inward_velocity = -(radial_north * vel_north + radial_east * vel_east);
  const float nu = constrain(atan2f(cross_velocity, inward_velocity), -M_PI_F / 2.0f, M_PI_F / 2.0f);
  const float capture = 4.0f * LOITER_L1_DAMPING * LOITER_L1_DAMPING * speed * speed / l1_distance * sinf(nu);

  // Circle: PD on the radial error plus the centripetal demand of the turn.
  const float radial_error = distance - radius;
  const float tangent_velocity = cross_velocity * direction;
  float correction = radial_error * omega * omega - inward_velocity * 2.0f * LOITER_L1_DAMPING * omega;
  if (inward_velocity < 0.0f && tangent_velocity < 0.0f)
    correction = MAX(correction, 0.0f);
  const float centripetal = tangent_velocity * tangent_velocity / MAX(0.5f * radius, radius + radial_error);
  const float circle = direction * (correction + centripetal);

  blackbox_set_debug(BBOX_DEBUG_NAVIGATION, 5, (int16_t)constrain(distance, 0.0f, 32767.0f)); // meters

  if (radial_error > 0.0f && direction * capture < direction * circle)
    return capture;
  return circle;
}

void nav_update_loiter(bool gps_valid) {
  const uint32_t now = time_micros();
  const float dt = constrain((now - loiter.updated_us) * 1e-6f, 0.0f, 0.1f);
  loiter.updated_us = now;

  if (!loiter_requested()) {
    state.wing_loiter_state = WING_LOITER_INACTIVE;
    blackbox_set_debug(BBOX_DEBUG_NAVIGATION, 4, state.wing_loiter_state);
    return;
  }

  const float max_bank = constrain(profile.wing.navigation.max_bank_angle, 0.0f, 80.0f) * DEGTORAD;
  if (state.wing_loiter_state == WING_LOITER_INACTIVE) {
    // Take over from the current attitude and throttle; the slew limits
    // below move them to the loiter targets.
    wing_nav_command.roll = constrain(state.attitude.roll, -max_bank, max_bank);
    wing_nav_command.pitch = state.attitude.pitch;
    wing_nav_command.throttle = state.throttle;
  }

  float roll = loiter_direction() * LOITER_FALLBACK_BANK;
  if (gps_valid) {
    if (state.wing_loiter_state != WING_LOITER_CIRCLE)
      loiter_place_circle();
    state.wing_loiter_state = WING_LOITER_CIRCLE;
    roll = atanf(loiter_lateral_accel() / GRAVITY);
  } else {
    // Position is unobservable: keep turning to stay near where GPS was lost.
    state.wing_loiter_state = WING_LOITER_BANK;
  }

  roll = constrain(roll, -max_bank, max_bank);
  wing_nav_command.roll = nav_slew(wing_nav_command.roll, roll, LOITER_BANK_SLEW_RATE * dt);
  wing_nav_command.pitch = nav_slew(wing_nav_command.pitch, 0.0f, LOITER_PITCH_SLEW_RATE * dt);
  const float throttle = constrain(profile.wing.navigation.cruise_throttle, 0.0f, 1.0f);
  wing_nav_command.throttle = nav_slew(wing_nav_command.throttle, throttle, LOITER_THROTTLE_SLEW_RATE * dt);

  blackbox_set_debug(BBOX_DEBUG_NAVIGATION, 4, state.wing_loiter_state);
  blackbox_set_debug(BBOX_DEBUG_NAVIGATION, 14, (int16_t)(wing_nav_command.pitch * RADTODEG * 10.0f)); // 0.1 deg
  blackbox_set_debug(BBOX_DEBUG_NAVIGATION, 15, (int16_t)(wing_nav_command.roll * RADTODEG * 10.0f));  // 0.1 deg
}
