#include "control/mission.h"

#include "util/cbor_helper.h"
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

mission_t mission;

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
