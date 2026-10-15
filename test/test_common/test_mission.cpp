#include <math.h>
#include <stdlib.h>
#include <string.h>
#include <unistd.h>
#include <unity.h>

#include "control/mission.h"
#include "core/flash.h"
#include "core/profile.h"
#include "io/quic.h"
#include "util/cbor_helper.h"

static mission_t mission_test_route(uint8_t count) {
  mission_t route = {.count = count, .end_action = MISSION_END_RTH};
  for (uint8_t i = 0; i < count; i++) {
    route.waypoints[i] = {
        .coord = {.lon = 1511234000 + i * 100, .lat = -338765000 - i * 100},
        .altitude = 30.0f + i,
        .speed = i % 2 ? 0.0f : 5.0f,
    };
  }
  return route;
}

// Encode count waypoints from route without speed, including more than the
// codec accepts.
static size_t mission_test_encode(uint8_t *buffer, size_t size, const mission_t *route, uint8_t count) {
  cbor_value_t enc;
  cbor_encoder_init(&enc, buffer, size);
  TEST_ASSERT_TRUE(cbor_encode_map_indefinite(&enc) >= CBOR_OK);
  TEST_ASSERT_TRUE(cbor_encode_str(&enc, "end_action") >= CBOR_OK);
  TEST_ASSERT_TRUE(cbor_encode_uint8_t(&enc, &route->end_action) >= CBOR_OK);
  TEST_ASSERT_TRUE(cbor_encode_str(&enc, "waypoints") >= CBOR_OK);
  TEST_ASSERT_TRUE(cbor_encode_array(&enc, count) >= CBOR_OK);
  for (uint8_t i = 0; i < count; i++) {
    const mission_waypoint_t *wp = &route->waypoints[i % MISSION_WAYPOINT_MAX];
    TEST_ASSERT_TRUE(cbor_encode_map_indefinite(&enc) >= CBOR_OK);
    TEST_ASSERT_TRUE(cbor_encode_str(&enc, "lat") >= CBOR_OK);
    TEST_ASSERT_TRUE(cbor_encode_int32_t(&enc, &wp->coord.lat) >= CBOR_OK);
    TEST_ASSERT_TRUE(cbor_encode_str(&enc, "lon") >= CBOR_OK);
    TEST_ASSERT_TRUE(cbor_encode_int32_t(&enc, &wp->coord.lon) >= CBOR_OK);
    TEST_ASSERT_TRUE(cbor_encode_str(&enc, "alt") >= CBOR_OK);
    TEST_ASSERT_TRUE(cbor_encode_float(&enc, &wp->altitude) >= CBOR_OK);
    TEST_ASSERT_TRUE(cbor_encode_end_indefinite(&enc) >= CBOR_OK);
  }
  TEST_ASSERT_TRUE(cbor_encode_end_indefinite(&enc) >= CBOR_OK);
  return cbor_encoder_len(&enc);
}

void test_mission_full_route_roundtrip(void) {
  const mission_t source = mission_test_route(MISSION_WAYPOINT_MAX);
  uint8_t buffer[2048] = {};
  cbor_value_t codec;
  cbor_encoder_init(&codec, buffer, sizeof(buffer));
  TEST_ASSERT_TRUE(cbor_encode_mission_t(&codec, &source) >= CBOR_OK);
  const size_t size = cbor_encoder_len(&codec);

  mission_t decoded = {};
  cbor_decoder_init(&codec, buffer, size);
  TEST_ASSERT_TRUE(cbor_decode_mission_t(&codec, &decoded) >= CBOR_OK);
  TEST_ASSERT_TRUE(mission_valid(&decoded));
  TEST_ASSERT_EQUAL_UINT8(MISSION_WAYPOINT_MAX, decoded.count);
  TEST_ASSERT_EQUAL_UINT8(MISSION_END_RTH, decoded.end_action);
  for (unsigned i = 0; i < MISSION_WAYPOINT_MAX; i++) {
    TEST_ASSERT_EQUAL_INT32(source.waypoints[i].coord.lat, decoded.waypoints[i].coord.lat);
    TEST_ASSERT_EQUAL_INT32(source.waypoints[i].coord.lon, decoded.waypoints[i].coord.lon);
    TEST_ASSERT_EQUAL_FLOAT(source.waypoints[i].altitude, decoded.waypoints[i].altitude);
    TEST_ASSERT_EQUAL_FLOAT(source.waypoints[i].speed, decoded.waypoints[i].speed);
  }
}

void test_mission_rejects_oversized_and_out_of_range_routes(void) {
  const mission_t route = mission_test_route(MISSION_WAYPOINT_MAX);
  uint8_t buffer[2048] = {};
  cbor_value_t codec;

  // A truncated route would fly a different mission.
  const size_t size = mission_test_encode(buffer, sizeof(buffer), &route, MISSION_WAYPOINT_MAX + 1);
  mission_t decoded = {};
  cbor_decoder_init(&codec, buffer, size);
  TEST_ASSERT_TRUE(cbor_decode_mission_t(&codec, &decoded) < CBOR_OK);

  // Omitted speed decodes as the configured cruise speed.
  const size_t short_size = mission_test_encode(buffer, sizeof(buffer), &route, 2);
  decoded = {};
  cbor_decoder_init(&codec, buffer, short_size);
  TEST_ASSERT_TRUE(cbor_decode_mission_t(&codec, &decoded) >= CBOR_OK);
  TEST_ASSERT_EQUAL_UINT8(2, decoded.count);
  TEST_ASSERT_EQUAL_FLOAT(0.0f, decoded.waypoints[0].speed);
  TEST_ASSERT_TRUE(mission_valid(&decoded));

  const mission_t empty = {};
  TEST_ASSERT_TRUE(mission_valid(&empty));

  mission_t invalid = mission_test_route(3);
  invalid.end_action = MISSION_END_MAX;
  TEST_ASSERT_FALSE(mission_valid(&invalid));

  invalid = mission_test_route(3);
  invalid.waypoints[2].coord = {};
  TEST_ASSERT_FALSE(mission_valid(&invalid));

  invalid = mission_test_route(3);
  invalid.waypoints[1].coord.lat = 900000001;
  TEST_ASSERT_FALSE(mission_valid(&invalid));

  const float bad_altitudes[] = {1.0f, 5001.0f, NAN, INFINITY};
  for (float altitude : bad_altitudes) {
    invalid = mission_test_route(3);
    invalid.waypoints[1].altitude = altitude;
    TEST_ASSERT_FALSE(mission_valid(&invalid));
  }

  const float bad_speeds[] = {-1.0f, 31.0f, NAN};
  for (float speed : bad_speeds) {
    invalid = mission_test_route(3);
    invalid.waypoints[0].speed = speed;
    TEST_ASSERT_FALSE(mission_valid(&invalid));
  }
}

static quic_flag mission_test_reply;
static mission_t mission_test_reply_mission;

static quic_flag mission_test_command(quic_mission_command cmd, const mission_t *route = NULL) {
  uint8_t request[2048] = {QUIC_MAGIC, QUIC_CMD_MISSION};
  cbor_value_t enc;
  cbor_encoder_init(&enc, request + QUIC_HEADER_LEN, sizeof(request) - QUIC_HEADER_LEN);
  const uint8_t value = cmd;
  TEST_ASSERT_TRUE(cbor_encode_uint8_t(&enc, &value) >= CBOR_OK);
  if (route)
    TEST_ASSERT_TRUE(cbor_encode_mission_t(&enc, route) >= CBOR_OK);
  const uint32_t size = cbor_encoder_len(&enc);
  request[2] = size >> 8;
  request[3] = size & 0xff;

  quic_t quic = {.priv_data = NULL, .send = [](uint8_t *data, uint32_t size, void *) {
                   TEST_ASSERT_TRUE(size >= QUIC_HEADER_LEN);
                   mission_test_reply = static_cast<quic_flag>(data[1] >> 5);
                   mission_test_reply_mission = {};
                   if (mission_test_reply != QUIC_FLAG_NONE)
                     return;
                   // Replies carry the stored mission, as Get returns it.
                   cbor_value_t dec;
                   cbor_decoder_init(&dec, data + QUIC_HEADER_LEN, size - QUIC_HEADER_LEN);
                   TEST_ASSERT_TRUE(cbor_decode_mission_t(&dec, &mission_test_reply_mission) >= CBOR_OK);
                 }};
  mission_test_reply = QUIC_FLAG_EXIT;
  TEST_ASSERT_TRUE(quic_process(&quic, request, size + QUIC_HEADER_LEN));
  return mission_test_reply;
}

void test_mission_upload_persists_and_rejects_invalid_routes(void) {
  char original_dir[4096];
  char directory[] = "/tmp/qs-mission-XXXXXX";
  TEST_ASSERT_NOT_NULL(getcwd(original_dir, sizeof(original_dir)));
  TEST_ASSERT_NOT_NULL(mkdtemp(directory));
  TEST_ASSERT_EQUAL_INT(0, chdir(directory));
  profile_mutex_init();
  mission = {};

  const mission_t route = mission_test_route(MISSION_WAYPOINT_MAX);
  TEST_ASSERT_EQUAL(QUIC_FLAG_NONE, mission_test_command(QUIC_MISSION_SET, &route));
  TEST_ASSERT_EQUAL_UINT8(MISSION_WAYPOINT_MAX, mission_test_reply_mission.count);
  mission = {};
  flash_load();
  TEST_ASSERT_EQUAL_UINT8(MISSION_WAYPOINT_MAX, mission.count);
  TEST_ASSERT_EQUAL_INT32(route.waypoints[31].coord.lat, mission.waypoints[31].coord.lat);
  TEST_ASSERT_EQUAL(QUIC_FLAG_NONE, mission_test_command(QUIC_MISSION_GET));
  TEST_ASSERT_EQUAL_FLOAT(route.waypoints[31].altitude, mission_test_reply_mission.waypoints[31].altitude);

  // A rejected upload keeps the stored route, in RAM and in flash.
  mission_t invalid = mission_test_route(2);
  invalid.waypoints[1].altitude = 0.0f;
  TEST_ASSERT_EQUAL(QUIC_FLAG_ERROR, mission_test_command(QUIC_MISSION_SET, &invalid));
  TEST_ASSERT_EQUAL_UINT8(MISSION_WAYPOINT_MAX, mission.count);
  mission = {};
  flash_load();
  TEST_ASSERT_EQUAL_UINT8(MISSION_WAYPOINT_MAX, mission.count);

  // Armed, only reading is allowed.
  flags.arm_state = 1;
  TEST_ASSERT_EQUAL(QUIC_FLAG_ERROR, mission_test_command(QUIC_MISSION_CLEAR));
  TEST_ASSERT_EQUAL(QUIC_FLAG_NONE, mission_test_command(QUIC_MISSION_GET));
  flags.arm_state = 0;
  TEST_ASSERT_EQUAL_UINT8(MISSION_WAYPOINT_MAX, mission.count);

  TEST_ASSERT_EQUAL(QUIC_FLAG_NONE, mission_test_command(QUIC_MISSION_CLEAR));
  TEST_ASSERT_EQUAL_UINT8(0, mission_test_reply_mission.count);
  flash_load();
  TEST_ASSERT_EQUAL_UINT8(0, mission.count);

  unlink("flash.bin");
  TEST_ASSERT_EQUAL_INT(0, chdir(original_dir));
  rmdir(directory);
}
