#include "control/mission.h"

#include <atomic>
#include <math.h>
#include <string.h>

#include "core/flash.h"
#include "core/profile.h"
#include "core/tasks.h"
#include "util/cbor_helper.h"
#include "util/crc.h"
#include "util/mutex.h"
#include "util/util.h"

static constexpr int32_t MISSION_LAT_LIMIT = 900000000;  // degrees * 1e7
static constexpr int32_t MISSION_LON_LIMIT = 1800000000; // degrees * 1e7
// Meters above launch. Baro drift and uneven ground make lower targets
// unsafe, and a wing has to cruise clear of anything near launch level.
#ifdef VEHICLE_WING
static constexpr float MISSION_ALTITUDE_MIN = 20.0f;
#else
static constexpr float MISSION_ALTITUDE_MIN = 5.0f;
#endif
static constexpr float MISSION_ALTITUDE_MAX = 5000.0f; // launching from a mountain leaves the valley far below
static constexpr float MISSION_SPEED_MAX = 30.0f; // m/s
static constexpr float MISSION_SAVE_MAX_SPEED = 3.5f; // m/s; faster, a disarmed vehicle may still be flying

mission_t mission;

// Set by Flight after a QUIC_RT commit and cleared by the USB thread while it
// holds profile_mutex, which keeps disarmed Flight from committing meanwhile.
static std::atomic<bool> save_pending{false};
static std::atomic<uint32_t> revision{0};
static_assert(std::atomic<uint32_t>::is_always_lock_free);

// Waypoints are flat on the wire: {lat, lon, alt, speed}.
static cbor_result_t cbor_encode_mission_waypoint_t(cbor_value_t *enc, const mission_waypoint_t *wp) {
  cbor_result_t res = CBOR_OK;
  CBOR_CHECK_ERROR(res = cbor_encode_map_indefinite(enc));
  CBOR_CHECK_ERROR(res = cbor_encode_str(enc, "lat"));
  CBOR_CHECK_ERROR(res = cbor_encode_int32_t(enc, &wp->coord.lat));
  CBOR_CHECK_ERROR(res = cbor_encode_str(enc, "lon"));
  CBOR_CHECK_ERROR(res = cbor_encode_int32_t(enc, &wp->coord.lon));
  CBOR_CHECK_ERROR(res = cbor_encode_str(enc, "alt"));
  CBOR_CHECK_ERROR(res = cbor_encode_float(enc, &wp->altitude));
  CBOR_CHECK_ERROR(res = cbor_encode_str(enc, "speed"));
  CBOR_CHECK_ERROR(res = cbor_encode_float(enc, &wp->speed));
  return cbor_encode_end_indefinite(enc);
}

static cbor_result_t cbor_decode_mission_waypoint_t(cbor_value_t *dec, mission_waypoint_t *wp) {
  cbor_result_t res = CBOR_OK;
  cbor_container_t map;
  CBOR_CHECK_ERROR(res = cbor_decode_map(dec, &map));
  for (uint32_t i = 0; i < cbor_decode_map_size(dec, &map); i++) {
    const uint8_t *name;
    uint32_t name_len;
    CBOR_CHECK_ERROR(res = cbor_decode_tstr(dec, &name, &name_len));
    if (buf_equal_string(name, name_len, "lat")) {
      CBOR_CHECK_ERROR(res = cbor_decode_int32_t(dec, &wp->coord.lat));
    } else if (buf_equal_string(name, name_len, "lon")) {
      CBOR_CHECK_ERROR(res = cbor_decode_int32_t(dec, &wp->coord.lon));
    } else if (buf_equal_string(name, name_len, "alt")) {
      CBOR_CHECK_ERROR(res = cbor_decode_float(dec, &wp->altitude));
    } else if (buf_equal_string(name, name_len, "speed")) {
      CBOR_CHECK_ERROR(res = cbor_decode_float(dec, &wp->speed));
    } else {
      CBOR_CHECK_ERROR(res = cbor_decode_skip(dec));
    }
  }
  return res;
}

bool mission_waypoint_valid(const mission_waypoint_t *wp) {
  const gps_coord_t coord = wp->coord;
  // 0/0 is the coordinate of an unset waypoint, never a planned one.
  if (coord.lat == 0 && coord.lon == 0)
    return false;
  if (coord.lat < -MISSION_LAT_LIMIT || coord.lat > MISSION_LAT_LIMIT ||
      coord.lon < -MISSION_LON_LIMIT || coord.lon > MISSION_LON_LIMIT)
    return false;
  if (!float_is_finite(wp->altitude) || wp->altitude < MISSION_ALTITUDE_MIN || wp->altitude > MISSION_ALTITUDE_MAX)
    return false;
  return float_is_finite(wp->speed) && wp->speed >= 0.0f && wp->speed <= MISSION_SPEED_MAX;
}

bool mission_valid(const mission_t *m) {
  if (m->count > MISSION_WAYPOINT_MAX || m->end_action >= MISSION_END_MAX)
    return false;
  for (uint32_t i = 0; i < m->count; i++) {
    if (!mission_waypoint_valid(&m->waypoints[i]))
      return false;
  }
  return true;
}

uint16_t mission_crc(const mission_t *m) {
  const uint8_t header[2] = {m->count, m->end_action};
  uint16_t crc = crc16_ccitt_data(0xFFFF, header, sizeof(header));
  for (uint32_t i = 0; i < m->count && i < MISSION_WAYPOINT_MAX; i++) {
    const mission_waypoint_t *wp = &m->waypoints[i];
    const int32_t altitude_cm = lroundf(wp->altitude * 100.0f);
    const uint16_t speed_cms = lroundf(wp->speed * 100.0f);
    uint8_t item[14];
    memcpy(item, &wp->coord.lat, 4);
    memcpy(item + 4, &wp->coord.lon, 4);
    memcpy(item + 8, &altitude_cm, 4);
    memcpy(item + 12, &speed_cms, 2);
    crc = crc16_ccitt_data(crc, item, sizeof(item));
  }
  return crc;
}

void mission_changed() {
  revision.fetch_add(1, std::memory_order_release);
}

uint32_t mission_revision() {
  return revision.load(std::memory_order_acquire);
}

void mission_save_later() {
  save_pending.store(true, std::memory_order_relaxed);
}

void mission_save_update() {
  if (!save_pending.load(std::memory_order_relaxed))
    return;
  // A disarmed wing may still be gliding, and the save masks interrupts.
  if (state.gps_lock && state.gps_speed >= MISSION_SAVE_MAX_SPEED)
    return;
  mutex_guard_t configuration(profile_mutex);
  // Flight may have armed while this thread waited for the lock.
  if (flags.arm_state)
    return;
  flight_reset_runtime();
  flash_save();
  save_pending.store(false, std::memory_order_relaxed);
}

SLOW_FLASH cbor_result_t cbor_encode_mission_t(cbor_value_t *enc, const mission_t *m) {
  cbor_result_t res = CBOR_OK;
  CBOR_CHECK_ERROR(res = cbor_encode_map_indefinite(enc));

  const uint8_t max_waypoints = MISSION_WAYPOINT_MAX;
  CBOR_CHECK_ERROR(res = cbor_encode_str(enc, "max_waypoints"));
  CBOR_CHECK_ERROR(res = cbor_encode_uint8_t(enc, &max_waypoints));

  // Report the limits mission_valid() enforces so the planner never guesses them.
  CBOR_CHECK_ERROR(res = cbor_encode_str(enc, "min_alt"));
  CBOR_CHECK_ERROR(res = cbor_encode_float(enc, &MISSION_ALTITUDE_MIN));
  CBOR_CHECK_ERROR(res = cbor_encode_str(enc, "max_alt"));
  CBOR_CHECK_ERROR(res = cbor_encode_float(enc, &MISSION_ALTITUDE_MAX));
  CBOR_CHECK_ERROR(res = cbor_encode_str(enc, "max_speed"));
  CBOR_CHECK_ERROR(res = cbor_encode_float(enc, &MISSION_SPEED_MAX));

  CBOR_CHECK_ERROR(res = cbor_encode_str(enc, "end_action"));
  CBOR_CHECK_ERROR(res = cbor_encode_uint8_t(enc, &m->end_action));

  CBOR_CHECK_ERROR(res = cbor_encode_str(enc, "waypoints"));
  CBOR_CHECK_ERROR(res = cbor_encode_array(enc, m->count));
  for (uint32_t i = 0; i < m->count; i++) {
    CBOR_CHECK_ERROR(res = cbor_encode_mission_waypoint_t(enc, &m->waypoints[i]));
  }

  return cbor_encode_end_indefinite(enc);
}

SLOW_FLASH cbor_result_t cbor_decode_mission_t(cbor_value_t *dec, mission_t *m) {
  cbor_result_t res = CBOR_OK;
  cbor_container_t map;
  CBOR_CHECK_ERROR(res = cbor_decode_map(dec, &map));
  for (uint32_t i = 0; i < cbor_decode_map_size(dec, &map); i++) {
    const uint8_t *name;
    uint32_t name_len;
    CBOR_CHECK_ERROR(res = cbor_decode_tstr(dec, &name, &name_len));

    if (buf_equal_string(name, name_len, "end_action")) {
      CBOR_CHECK_ERROR(res = cbor_decode_uint8_t(dec, &m->end_action));
      continue;
    }
    if (buf_equal_string(name, name_len, "waypoints")) {
      cbor_container_t array;
      CBOR_CHECK_ERROR(res = cbor_decode_array(dec, &array));
      const uint32_t count = cbor_decode_array_size(dec, &array);
      // A truncated mission would fly a different route; reject it instead.
      if (count > MISSION_WAYPOINT_MAX)
        return cbor_handle_error(CBOR_ERR_OVERFLOW);
      for (uint32_t j = 0; j < count; j++) {
        CBOR_CHECK_ERROR(res = cbor_decode_mission_waypoint_t(dec, &m->waypoints[j]));
      }
      m->count = count;
      continue;
    }
    // Ignore read-only fields such as max_waypoints and unknown additions.
    CBOR_CHECK_ERROR(res = cbor_decode_skip(dec));
  }
  return res;
}
