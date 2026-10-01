#pragma once

// Attitude and throttle targets from wing navigation, updated at the 10 ms
// navigation cadence and applied by wing control while
// state.wing_loiter_state is active. Both run in Flight.
typedef struct {
  float roll;     // radians, positive right bank
  float pitch;    // radians, positive nose-down like level-mode attitude targets
  float throttle; // 0..1
} wing_nav_command_t;

extern wing_nav_command_t wing_nav_command;

void nav_update_loiter(bool gps_valid);
