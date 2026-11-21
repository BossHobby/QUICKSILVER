#pragma once

#include "util/util.h"

// GPS ground speed or height above launch that shows a wing in flight.
static constexpr float WING_FLYING_MIN_SPEED = 3.5f;    // m/s
static constexpr float WING_FLYING_MIN_ALTITUDE = 5.0f; // meters

// Throttle per degree of climb that holds cruise airspeed without a sensor.
// Navigation feeds it forward; wing control uses it to estimate airspeed.
static constexpr float WING_THROTTLE_PER_CLIMB_DEG = 0.015f;

// Cruise-to-current airspeed ratio limits: of the throttle model's airspeed,
// and of the range the PID gains scale over.
static constexpr float WING_SPEED_SCALE_MIN = 0.6f;
static constexpr float WING_SPEED_SCALE_MAX = 1.25f;

// The steepest climb navigation commands, radians nose-up. Wing control
// lowers it while a measured airspeed is low.
static constexpr float WING_MAX_CLIMB_ANGLE = 20.0f * DEGTORAD;

// Attitude and throttle targets from wing navigation, updated at the 10 ms
// navigation cadence and applied by wing control while
// state.wing_nav_state is active. Both run in Flight.
typedef struct {
  float roll;     // radians, positive right bank
  float pitch;    // radians, positive nose-down like level-mode attitude targets
  float throttle; // 0..1
} wing_nav_command_t;

extern wing_nav_command_t wing_nav_command;

void nav_update_wing(float dt);
