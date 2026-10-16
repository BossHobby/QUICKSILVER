#pragma once

#include <cbor.h>
#include <stdint.h>

#include "control/control.h"

static constexpr uint8_t MISSION_WAYPOINT_MAX = 32;
// A first waypoint farther away than this belongs to a mission planned elsewhere.
static constexpr float MISSION_START_MAX_DISTANCE = 2000.0f; // meters

typedef enum {
  MISSION_END_HOLD, // hold position at the last waypoint
  MISSION_END_RTH,  // return home as if RTH had been requested there
  MISSION_END_MAX,
} __attribute__((__packed__)) mission_end_action_t;

typedef struct {
  gps_coord_t coord;
  float altitude; // meters above launch
  float speed;    // m/s horizontal; zero uses navigation.rth_cruise_speed
} mission_waypoint_t;

// Uploaded by USB while disarmed, saved in its own flash block and read by
// Flight navigation; the profile_mutex orders the two like profile edits.
typedef struct {
  uint8_t count;
  uint8_t end_action; // mission_end_action_t
  mission_waypoint_t waypoints[MISSION_WAYPOINT_MAX];
} mission_t;

extern mission_t mission;

cbor_result_t cbor_encode_mission_t(cbor_value_t *enc, const mission_t *m);
// Decodes into a zeroed mission; rejects more waypoints than fit rather than
// truncating the route. Check mission_valid() before using the result.
cbor_result_t cbor_decode_mission_t(cbor_value_t *dec, mission_t *m);

// Set, in coordinate range, within the vehicle's altitude limits above launch
// and at most 30 m/s.
bool mission_waypoint_valid(const mission_waypoint_t *wp);
// Every waypoint is valid and the end action is known.
bool mission_valid(const mission_t *m);
