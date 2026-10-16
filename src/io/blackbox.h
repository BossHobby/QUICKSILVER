#pragma once

#include "core/profile.h"
#include "util/util.h"

// BLACKBOX version changelog (log format compatibility):
// 0.3.2: cpu_load is a percentage 0-100 (was microseconds).
//        add state field with flags, aux switches, failsafe phase, RTH, wing, mission and guided states.
//        add the UTC start datetime to each directory entry.
// 0.3.1 (95765fe1): add GPS coordinate, home and altitude fields; grow debug to 16 entries.
//        replace the four-motor field with per-output values (a1887d4f).
// 0.2.0 (7343f574): delta-encode I/P frames, scale 100 (was 1000), store the profile with each log
//        and widen the file blackbox_rate to uint32.
// 0.1.0: initial versioned blackbox format.
#define BLACKBOX_VERSION MAKE_SEMVER(0, 3, 2)

#define BLACKBOX_SCALE 100
#define BLACKBOX_DEBUG_SIZE 16

typedef enum {
  BBOX_FIELD_LOOP,
  BBOX_FIELD_TIME,
  BBOX_FIELD_PID_P_TERM,
  BBOX_FIELD_PID_I_TERM,
  BBOX_FIELD_PID_D_TERM,
  BBOX_FIELD_RX,
  BBOX_FIELD_SETPOINT,
  BBOX_FIELD_ACCEL_RAW,
  BBOX_FIELD_ACCEL_FILTER,
  BBOX_FIELD_GYRO_RAW,
  BBOX_FIELD_GYRO_FILTER,
  BBOX_FIELD_OUTPUT,
  BBOX_FIELD_CPU_LOAD,
  BBOX_FIELD_GPS_COORD,
  BBOX_FIELD_GPS_HOME,
  BBOX_FIELD_ALTITUDE,
  BBOX_FIELD_DEBUG,
  BBOX_FIELD_STATE,

  BBOX_FIELD_MAX,
} blackbox_field_t;

typedef enum {
  BBOX_DEBUG_DYN_NOTCH = 0x1 << 0,
  BBOX_DEBUG_ROVER = 0x1 << 1,
  BBOX_DEBUG_NAVIGATION = 0x1 << 2,
  BBOX_DEBUG_WING = 0x1 << 3,
} blackbox_debug_flag_t;

// State word. Bits 0-7 are common; bits from 8 belong to the recording
// vehicle, whose modes derive from these states and the aux bits.
typedef enum {
  BBOX_STATE_IN_AIR = 0x1 << 0,
  BBOX_STATE_ON_GROUND = 0x1 << 1,
  BBOX_STATE_FAILSAFE = 0x1 << 2,
  BBOX_STATE_RX_LOST = 0x1 << 3,
  BBOX_STATE_LOWBATT = 0x1 << 4,
  // failsafe_phase_t, 3 bits at BBOX_STATE_FAILSAFE_PHASE_SHIFT

#if defined(VEHICLE_WING)
  BBOX_STATE_WING_NAV_FAILSAFE = 0x1 << 8,
  // wing_nav_state_t, wing_launch_state_t and wing_autotrim_state_t at their shifts
#elif defined(VEHICLE_MULTI)
  BBOX_STATE_RTH_ACTIVE = 0x1 << 8,
  BBOX_STATE_RTH_FAILSAFE = 0x1 << 9,
  BBOX_STATE_FAILSAFE_HOLD = 0x1 << 10,
  // rth_state_t at BBOX_STATE_RTH_SHIFT
#endif
#ifndef VEHICLE_ROVER
  // At the same bits on multirotors and wings, with the waypoint index at
  // BBOX_STATE_MISSION_INDEX_SHIFT.
  BBOX_STATE_MISSION_ACTIVE = 0x1 << 20,
  BBOX_STATE_GUIDED_ACTIVE = 0x1 << 26, // following a ground station target
#endif
} blackbox_state_flag_t;

static constexpr uint32_t BBOX_STATE_FAILSAFE_PHASE_SHIFT = 5;
#if defined(VEHICLE_WING)
static constexpr uint32_t BBOX_STATE_WING_NAV_SHIFT = 9;       // 4 bits
static constexpr uint32_t BBOX_STATE_WING_LAUNCH_SHIFT = 13;   // 4 bits
static constexpr uint32_t BBOX_STATE_WING_AUTOTRIM_SHIFT = 17; // 2 bits
#elif defined(VEHICLE_MULTI)
static constexpr uint32_t BBOX_STATE_RTH_SHIFT = 11; // 4 bits
#endif
#ifndef VEHICLE_ROVER
static constexpr uint32_t BBOX_STATE_MISSION_INDEX_SHIFT = 21; // 5 bits, active waypoint from 0
#endif

typedef struct {
  int16_t axis[MOTOR_PIN_MAX];
} compact_output_t;

cbor_result_t cbor_encode_compact_output_t(cbor_value_t *enc, const compact_output_t *output);

typedef struct {
  uint32_t loop;
  uint32_t time;

  compact_vec3_t pid_p_term;
  compact_vec3_t pid_i_term;
  compact_vec3_t pid_d_term;

  compact_vec4_t rx;
  compact_vec4_t setpoint;

  compact_vec3_t accel_raw;
  compact_vec3_t accel_filter;

  compact_vec3_t gyro_raw;
  compact_vec3_t gyro_filter;

  compact_output_t output;

  uint16_t cpu_load; // system load percentage 0-100 (since blackbox 0.3.2)

  int32_t gps_coord[2]; // latitude, longitude; degrees * 1e7
  int32_t gps_home[2];  // same order and units; captured home used by navigation
  int16_t altitude;     // state.altitude, decimeters above launch; saturated

  int16_t debug[BLACKBOX_DEBUG_SIZE];

  // State field; both stay below 2^31, so P-frame deltas fit int32.
  uint32_t state; // blackbox_state_flag_t bits and packed states
  uint32_t aux;   // state.aux_active, bit per aux_function_t of the recording vehicle
} blackbox_t;

// Frame types for I-frame/P-frame encoding
typedef enum {
  BLACKBOX_FRAME_I = 0,  // Intra frame - complete data
  BLACKBOX_FRAME_P = 1,  // Predicted frame - delta from previous
} blackbox_frame_type_t;

// Special flag to indicate frame type is stored in upper bit of field flags
#define BLACKBOX_FRAME_TYPE_BIT (1UL << 31)

// Blackbox fields (should align with above structure)

cbor_result_t cbor_encode_blackbox_frame(cbor_value_t *enc, const blackbox_t *current, const blackbox_t *previous, blackbox_frame_type_t frame_type, const uint32_t field_flags);

void blackbox_init();
void blackbox_thread(void *);
void blackbox_reset();
void blackbox_set_debug(blackbox_debug_flag_t flag, uint8_t index, int16_t data);

// Flight captures after control; the Blackbox task encodes and services storage.
void blackbox_capture();
