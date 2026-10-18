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
  float speed;    // m/s horizontal; zero uses navigation.rth_cruise_speed. A wing flies at navigation.cruise_throttle instead.
} mission_waypoint_t;

// Uploaded by USB while disarmed, or replaced by a QUIC_RT commit applied in
// Flight at any time; saved in its own flash block and read by Flight
// navigation. The profile_mutex orders USB with disarmed Flight like profile
// edits; USB rejects uploads while armed.
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
// CRC-16/CCITT-FALSE over count, end_action and each waypoint as QUIC_RT sends
// it: lat, lon, altitude in cm (int32) and speed in cm/s (uint16), little-endian.
uint16_t mission_crc(const mission_t *m);

// Every writer of mission calls mission_changed() after the write, so readers
// on other threads can cache what they derive from it until the revision moves.
void mission_changed();
uint32_t mission_revision();

// Flight: the mission was replaced over QUIC_RT and is not in flash yet.
void mission_save_later();
// Ground maintenance, run by the USB thread: save a replaced mission once the
// vehicle is disarmed and still.
void mission_save_update();
