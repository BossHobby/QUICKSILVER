#pragma once

#include <stdint.h>

#include "util/vector.h"

// Multirotor navigation controllers, run by multirotor navigation in Flight:
// altitude by throttle, earth-frame velocity by tilt, and the trims they
// learn. Navigation guidance chooses the requests and owns the phases.

static constexpr float ALT_MAX_RATE = 2.0f;    // m/s climb or descent
static constexpr float VEL_ACCEL_LIMIT = 2.0f; // m/s^2 desired velocity change
static constexpr float VEL_JERK_LIMIT = 4.0f;  // m/s^3 desired acceleration change
static constexpr float ANGLE_SLEW_RATE = 2.0f; // normalized stick/s

// Vertical controller history. Shared by the failsafe hold and RTH so a
// failsafe that escalates to RTH keeps its trim. Guidance winds the integral
// down onto the ground while landing.
typedef struct {
  float integral;
  float throttle_command;
  float desired_rate;
  float desired_accel; // m/s^2 change of desired_rate
} nav_altitude_control_t;

// Horizontal controller history, reset on every engagement. Guidance seeds
// the desired velocity from the ground track, reads it to steer and to judge
// divergence, and eases the commands level where the ground may hold the craft.
typedef struct {
  vec2_t integral;         // radians of tilt toward north/east
  vec2_t desired_velocity; // m/s north/east
  vec2_t desired_accel;    // m/s^2 change of the desired velocity, flown as tilt
  float roll_command;
  float pitch_command;
} nav_velocity_control_t;

extern nav_altitude_control_t nav_alt;
extern nav_velocity_control_t nav_velocity;

// cos(roll) * cos(pitch) of the attitude, no less than 0.5.
float nav_tilt();
// Learn level-hover throttle while the pilot flies steadily. Unlike a
// configured value, this follows battery sag and payload changes. Disarmed,
// the hover and drag trims start over.
void nav_update_trims(float dt);
// Configured throttle range, below the headroom kept for attitude authority.
float nav_throttle_min();
float nav_throttle_max(float throttle_min = nav_throttle_min());
// Learned hover, or the configured one until learned.
float nav_hover_throttle();
// Take over vertical control from the currently applied throttle.
void nav_altitude_start(float throttle);
// target_rate is how fast a moving target changes, flown ahead of the error.
// The throttle never goes below throttle_min: throttle_min, or the lower
// landing floor.
void nav_update_altitude_control_above(float target_alt, float dt, float target_rate, float max_rate, float throttle_min);
void nav_update_altitude_control(float target_alt, float dt, float target_rate = 0.0f);
// Shape the desired earth-frame velocity toward the request and fly it with
// tilt, within authority_scale of the tilt the heading confidence allows.
// Navigation calls this every pass it steers, so the tilt and the wind and
// drag trim carry across state changes.
void nav_update_velocity_control(vec2_t request, bool moving_away, float authority_scale, float dt);

#ifdef PIO_UNIT_TESTING
void nav_test_reset_control();
void nav_test_set_hover_learned(float throttle);
#endif
