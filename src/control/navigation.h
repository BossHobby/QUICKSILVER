#pragma once

#include <stdint.h>

#include "control/control.h"
#include "control/mission.h"
#include "util/util.h"

constexpr float EARTH_RADIUS = 6371000.0f; // meters
constexpr float METERS_PER_DEGREE_LAT = EARTH_RADIUS * M_PI_F / 180.0f;
constexpr uint32_t NAV_GPS_STALE_MS = 500;
constexpr uint8_t NAV_COMMAND_MISSION_RESUME = 0xFF; // nav_command_t.index: resume where the mission left off

// Ground station commands, delivered by the QUIC_RT link while the pilot grants
// guided flight with the mission switch.
typedef enum {
  NAV_COMMAND_GOTO,     // fly to waypoint, then hold there
  NAV_COMMAND_HOLD,     // hold the current position and altitude
  NAV_COMMAND_SET_ALT,  // change the guided target altitude
  NAV_COMMAND_RTH,      // return home; the mission switch still ends it
  NAV_COMMAND_MISSION,  // fly the stored mission from index
  NAV_COMMAND_MAX,
} nav_command_type_t;

typedef enum {
  NAV_RESULT_OK,
  NAV_RESULT_NO_AUTHORITY, // mission switch off, failsafe, RTH switch, disarmed or not airborne
  NAV_RESULT_INVALID,      // waypoint out of limits, or no guided target to change
  NAV_RESULT_TOO_FAR,      // beyond MISSION_START_MAX_DISTANCE of the vehicle
  NAV_RESULT_NO_MISSION,   // no stored waypoint at the requested index
  NAV_RESULT_UNSUPPORTED,  // the vehicle has no guided flight
} nav_command_result_t;

typedef struct {
  uint8_t type; // nav_command_type_t
  uint8_t id;   // chosen by the ground station; a repeated id is acknowledged, not applied again
  mission_waypoint_t waypoint; // GOTO; SET_ALT uses only altitude
  uint8_t index;               // MISSION: waypoint, or NAV_COMMAND_MISSION_RESUME
} nav_command_t;

// Ground station target while state.guided_active, written by vehicle navigation.
extern mission_waypoint_t nav_guided;

// Local displacement in meters; GPS coordinates are signed degrees * 1e7.
void nav_position_delta(gps_coord_t start, gps_coord_t end, float *north, float *east);

// The target, or the stored waypoint, lies within MISSION_START_MAX_DISTANCE
// of the vehicle.
bool nav_in_start_range(gps_coord_t target);
bool nav_mission_in_start_range(uint8_t index);
// The guided target, the active mission waypoint, or home.
gps_coord_t nav_target_coord();
// Publish nav_target_* toward nav_target_coord().
void nav_update_target();
// Vehicle navigation applies a ground station command in Flight, before its update.
nav_command_result_t nav_command_apply(const nav_command_t *cmd);

void nav_init();
void nav_update();

// Smoothed earth-vertical acceleration, m/s^2 up.
float nav_vertical_accel();

// Publish horizontal GPS acceleration for IMU gravity compensation. The IMU
// rotates it by heading, so vehicles update it only while heading is trusted.
void nav_update_gps_accel();
void nav_reset_gps_accel();
