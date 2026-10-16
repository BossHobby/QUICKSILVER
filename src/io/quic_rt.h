#pragma once

#include <stdbool.h>
#include <stdint.h>

#include "control/navigation.h"

// QUIC_RT, the realtime ground station link in CRSF frame type 0x84. Unlike the
// QUIC session on 0x83, which exists only on the ground, it serves armed and
// disarmed vehicles alike with small self-contained messages: the latest one
// wins and nothing is retransmitted. Frames use the extended header like other
// addressed frames; the payload after it is <version> <type> <body...>,
// little-endian.
//
// STATUS carries only what standard CRSF telemetry cannot: position, speed
// and satellites come from GPS (0x02), altitude and vertical speed from
// barometer altitude (0x09, absent without a barometer), heading from
// attitude yaw (0x1E) and battery from 0x08.
//
// The ground station sends PING or COMMAND frames to the flight controller.
// While it has been heard recently, the flight controller sends STATUS back to
// its address. A COMMAND repeats its id until STATUS acknowledges that id; a
// repeated id is acknowledged again, not applied again, so a rejected command
// is retried under a new id. After 3 s of silence the next frame starts a new
// session, which forgets the acknowledgement and accepts any id.
static constexpr uint8_t QUIC_RT_VERSION = 1;
static constexpr uint8_t QUIC_RT_RESULT_NONE = 0xFF; // no command applied yet

typedef enum {
  QUIC_RT_STATUS = 0x01,  // flight controller -> ground station: quic_rt_status_t
  QUIC_RT_COMMAND = 0x02, // ground station -> flight controller: <command type> <id> <args>
  QUIC_RT_PING = 0x03,    // ground station -> flight controller: request STATUS, no body
} quic_rt_type_t;

// COMMAND arguments by nav_command_type_t; HOLD and RTH carry none.
typedef struct {
  int32_t lat;         // degrees * 1e7
  int32_t lon;         // degrees * 1e7
  int32_t altitude_cm; // above launch
  uint8_t speed_dms;   // 0.1 m/s horizontal; zero uses navigation.rth_cruise_speed
} __attribute__((__packed__)) quic_rt_goto_t;

typedef struct {
  int32_t altitude_cm; // above launch
} __attribute__((__packed__)) quic_rt_set_alt_t;

typedef struct {
  uint8_t index; // waypoint, or NAV_COMMAND_MISSION_RESUME
} __attribute__((__packed__)) quic_rt_mission_t;

typedef enum {
  QUIC_RT_FLAG_ARMED = 1 << 0,
  QUIC_RT_FLAG_IN_AIR = 1 << 1,
  QUIC_RT_FLAG_FAILSAFE = 1 << 2,
  QUIC_RT_FLAG_GPS_LOCK = 1 << 3,
  QUIC_RT_FLAG_RTH = 1 << 4,          // guided flight owns the controls (multirotor RTH, mission or guided)
  QUIC_RT_FLAG_MISSION = 1 << 5,      // state.mission_active
  QUIC_RT_FLAG_GUIDED = 1 << 6,       // state.guided_active
  QUIC_RT_FLAG_GUIDED_READY = 1 << 7, // state.guided_ready
} quic_rt_flags_t;

typedef enum {
  QUIC_RT_VEHICLE_MULTI,
  QUIC_RT_VEHICLE_ROVER,
  QUIC_RT_VEHICLE_WING,
} quic_rt_vehicle_t;

typedef struct {
  uint8_t command_id;     // last acknowledged command
  uint8_t command_result; // nav_command_result_t, or QUIC_RT_RESULT_NONE
  uint8_t flags;          // quic_rt_flags_t
  uint8_t vehicle;        // quic_rt_vehicle_t
  uint8_t nav_state;      // rth_state_t on multirotors, wing_nav_state_t on wings
  uint8_t mission_index;  // active waypoint while flying the mission
  uint8_t mission_count;
  int32_t home_lat;   // degrees * 1e7; the launch point while armed, else the craft
  int32_t home_lon;   // degrees * 1e7
  int32_t target_lat; // degrees * 1e7; guided target, mission waypoint or home
  int32_t target_lon; // degrees * 1e7
} __attribute__((__packed__)) quic_rt_status_t;

// Payload of a STATUS frame: version, type and quic_rt_status_t.
static constexpr uint8_t QUIC_RT_STATUS_SIZE = 2 + sizeof(quic_rt_status_t);

// IO: a frame payload after the extended header, from the ground station at origin.
void quic_rt_receive(uint8_t origin, const uint8_t *payload, uint8_t size);
// IO: when STATUS is due, fills QUIC_RT_STATUS_SIZE bytes of payload, sets
// destination to the ground station address and returns that size; else 0.
uint8_t quic_rt_status(uint8_t *payload, uint8_t *destination);

// Flight: the latest command not yet taken, unless it resends the applied
// one; a newer command replaces an unread one.
bool quic_rt_take(nav_command_t *cmd);
// Flight: acknowledge an applied command in the next STATUS.
void quic_rt_complete(uint8_t id, nav_command_result_t result);
