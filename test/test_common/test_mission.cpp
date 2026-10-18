#include <math.h>
#include <stdlib.h>
#include <string.h>
#include <unistd.h>
#include <unity.h>

#include "control/mission.h"
#include "control/navigation.h"
#include "core/flash.h"
#include "core/profile.h"
#include "driver/fmc.h"
#include "driver/time.h"
#include "io/quic.h"
#include "io/quic_rt.h"
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

// The ground station computes the same CRC from the values it sends.
void test_mission_crc_covers_the_sent_route(void) {
  mission_t route = mission_test_route(2);
  route.waypoints[1].altitude = 31.25f;
  TEST_ASSERT_EQUAL_HEX16(0x4BAE, mission_crc(&route));
  const mission_t empty = {};
  TEST_ASSERT_EQUAL_HEX16(0x1D0F, mission_crc(&empty));

  // Waypoints past the end are not part of the route.
  mission_t padded = route;
  padded.waypoints[2] = mission_test_route(3).waypoints[2];
  TEST_ASSERT_EQUAL_HEX16(mission_crc(&route), mission_crc(&padded));

  mission_t changed = route;
  changed.end_action = MISSION_END_HOLD;
  TEST_ASSERT_NOT_EQUAL(mission_crc(&route), mission_crc(&changed));
  changed = route;
  changed.waypoints[1].speed = 0.01f;
  TEST_ASSERT_NOT_EQUAL(mission_crc(&route), mission_crc(&changed));
}

static uint8_t mission_link_id;

static void mission_link_send(nav_command_type_t type, const void *args, uint8_t size) {
  uint8_t frame[5 + NAV_COMMAND_ITEMS_MAX * sizeof(quic_rt_mission_item_t)] = {QUIC_RT_VERSION, QUIC_RT_COMMAND, (uint8_t)type, ++mission_link_id};
  memcpy(frame + 4, args, size);
  quic_rt_receive(0x10, frame, 4 + size);
  time_test_advance_us(10000);
  nav_update();
}

static void mission_link_items(uint8_t index, const mission_waypoint_t *waypoints, uint8_t count) {
  uint8_t args[1 + NAV_COMMAND_ITEMS_MAX * sizeof(quic_rt_mission_item_t)] = {index};
  for (uint8_t i = 0; i < count; i++) {
    const quic_rt_mission_item_t item = {
        .lat = waypoints[i].coord.lat,
        .lon = waypoints[i].coord.lon,
        .altitude_cm = (int32_t)lroundf(waypoints[i].altitude * 100.0f),
        .speed_cms = (uint16_t)lroundf(waypoints[i].speed * 100.0f),
    };
    memcpy(args + 1 + i * sizeof(item), &item, sizeof(item));
  }
  mission_link_send(NAV_COMMAND_MISSION_ITEMS, args, 1 + count * sizeof(quic_rt_mission_item_t));
}

static void mission_link_commit(const mission_t *intended) {
  const quic_rt_mission_commit_t commit = {
      .count = intended->count,
      .end_action = intended->end_action,
      .crc = mission_crc(intended),
  };
  mission_link_send(NAV_COMMAND_MISSION_COMMIT, &commit, sizeof(commit));
}

static quic_rt_status_t mission_link_status(void) {
  time_test_advance_us(200000);
  uint8_t payload[QUIC_RT_STATUS_SIZE];
  uint8_t destination = 0;
  TEST_ASSERT_EQUAL_UINT8(QUIC_RT_STATUS_SIZE, quic_rt_status(payload, &destination));
  quic_rt_status_t status;
  memcpy(&status, payload + 2, sizeof(status));
  TEST_ASSERT_EQUAL_UINT8(mission_link_id, status.command_id);
  return status;
}

static uint8_t mission_link_result(void) {
  return mission_link_status().result_index >> QUIC_RT_RESULT_SHIFT;
}

static mission_t mission_flash(void) {
  mission_t stored = {};
  fmc_read_buf(MISSION_STORAGE_OFFSET + FMC_MAGIC_SIZE, (uint8_t *)&stored, sizeof(stored));
  return stored;
}

void test_mission_link_replaces_the_route_and_saves_it_on_the_ground(void) {
  char original_dir[4096];
  char directory[] = "/tmp/qs-mission-XXXXXX";
  TEST_ASSERT_NOT_NULL(getcwd(original_dir, sizeof(original_dir)));
  TEST_ASSERT_NOT_NULL(mkdtemp(directory));
  TEST_ASSERT_EQUAL_INT(0, chdir(directory));
  const auto saved_gps = profile.serial.gps;
  profile.serial.gps = SERIAL_PORT_INVALID;
  memset(&state, 0, sizeof(state));
  memset(&flags, 0, sizeof(flags));
  time_test_set_us(10000000);
  profile_mutex_init();
  nav_init();
  mission = mission_test_route(3);
  flags.arm_state = 1;
  time_test_advance_us(10000);
  nav_update();

  // IO stages and checks edits itself; only a valid commit reaches Flight.
  {
    const mission_t route = mission_test_route(3);
    uint8_t frame[4 + sizeof(quic_rt_mission_commit_t)] = {QUIC_RT_VERSION, QUIC_RT_COMMAND, NAV_COMMAND_MISSION_COMMIT, ++mission_link_id};
    const quic_rt_mission_commit_t bad = {.count = 3, .end_action = route.end_action, .crc = (uint16_t)(mission_crc(&route) ^ 1)};
    memcpy(frame + 4, &bad, sizeof(bad));
    quic_rt_receive(0x10, frame, sizeof(frame));
    nav_command_t taken;
    TEST_ASSERT_FALSE(quic_rt_take(&taken));
    TEST_ASSERT_EQUAL_UINT8(NAV_RESULT_MISMATCH, mission_link_result());
    const quic_rt_mission_commit_t good = {.count = 3, .end_action = route.end_action, .crc = mission_crc(&route)};
    frame[3] = ++mission_link_id;
    memcpy(frame + 4, &good, sizeof(good));
    quic_rt_receive(0x10, frame, sizeof(frame));
    TEST_ASSERT_TRUE(quic_rt_take(&taken));
    TEST_ASSERT_EQUAL_UINT8(NAV_COMMAND_MISSION_COMMIT, taken.type);
    TEST_ASSERT_EQUAL_HEX16(mission_crc(&route), mission_crc(quic_rt_staged()));
    // While Flight owns the staged mission, edits wait.
    uint8_t items[5 + sizeof(quic_rt_mission_item_t)] = {QUIC_RT_VERSION, QUIC_RT_COMMAND, NAV_COMMAND_MISSION_ITEMS, ++mission_link_id, 0};
    const quic_rt_mission_item_t item = {.lat = route.waypoints[0].coord.lat, .lon = route.waypoints[0].coord.lon, .altitude_cm = 9000};
    memcpy(items + 5, &item, sizeof(item));
    quic_rt_receive(0x10, items, sizeof(items));
    TEST_ASSERT_EQUAL_FLOAT(30.0f, quic_rt_staged()->waypoints[0].altitude);
    quic_rt_complete(taken.id, NAV_RESULT_OK);

    // A newer command replacing an unread commit returns staging to IO.
    frame[3] = ++mission_link_id;
    quic_rt_receive(0x10, frame, sizeof(frame));
    const uint8_t hold[] = {QUIC_RT_VERSION, QUIC_RT_COMMAND, NAV_COMMAND_HOLD, ++mission_link_id};
    quic_rt_receive(0x10, hold, sizeof(hold));
    TEST_ASSERT_TRUE(quic_rt_take(&taken));
    TEST_ASSERT_EQUAL_UINT8(NAV_COMMAND_HOLD, taken.type);
    quic_rt_complete(taken.id, NAV_RESULT_UNSUPPORTED);
    items[3] = ++mission_link_id;
    quic_rt_receive(0x10, items, sizeof(items));
    TEST_ASSERT_EQUAL_UINT8(NAV_RESULT_OK, mission_link_result());
    TEST_ASSERT_EQUAL_FLOAT(90.0f, quic_rt_staged()->waypoints[0].altitude);
    // End staging without changing the route.
    mission_link_commit(&route);
    TEST_ASSERT_EQUAL_UINT8(NAV_RESULT_MISMATCH, mission_link_result());
  }

  // Only the changed waypoint is sent; the others come from the stored route.
  mission_t intended = mission_test_route(4);
  intended.waypoints[1].altitude = 50.0f;
  mission_link_items(1, &intended.waypoints[1], 1);
  TEST_ASSERT_EQUAL_UINT8(NAV_RESULT_OK, mission_link_result());
  TEST_ASSERT_EQUAL_FLOAT(31.0f, mission.waypoints[1].altitude);
  mission_link_items(3, &intended.waypoints[3], 1);
  mission_link_commit(&intended);
  quic_rt_status_t status = mission_link_status();
  TEST_ASSERT_EQUAL_UINT8(NAV_RESULT_OK, status.result_index >> QUIC_RT_RESULT_SHIFT);
  TEST_ASSERT_EQUAL_UINT8(4, status.mission_count);
  TEST_ASSERT_EQUAL_HEX16(mission_crc(&intended), status.mission_crc);
  TEST_ASSERT_EQUAL_FLOAT(50.0f, mission.waypoints[1].altitude);

  // A lost item fails the CRC and leaves the stored route; staging then starts
  // over from it.
  mission_t next = intended;
  next.waypoints[0].altitude = 60.0f;
  next.waypoints[2].altitude = 60.0f;
  mission_link_items(0, &next.waypoints[0], 1);
  mission_link_commit(&next);
  TEST_ASSERT_EQUAL_UINT8(NAV_RESULT_MISMATCH, mission_link_result());
  TEST_ASSERT_EQUAL_FLOAT(30.0f, mission.waypoints[0].altitude);
  mission_link_items(2, &next.waypoints[2], 1);
  mission_link_commit(&next);
  TEST_ASSERT_EQUAL_UINT8(NAV_RESULT_MISMATCH, mission_link_result());
  mission_link_items(0, next.waypoints, 3);
  mission_link_commit(&next);
  TEST_ASSERT_EQUAL_UINT8(NAV_RESULT_OK, mission_link_result());
  TEST_ASSERT_EQUAL_FLOAT(60.0f, mission.waypoints[0].altitude);
  TEST_ASSERT_EQUAL_FLOAT(60.0f, mission.waypoints[2].altitude);

  // Out-of-limit waypoints and routes are rejected.
  mission_t low = next;
  low.waypoints[0].altitude = 1.0f;
  mission_link_items(0, low.waypoints, 1);
  TEST_ASSERT_EQUAL_UINT8(NAV_RESULT_INVALID, mission_link_result());
  mission_link_items(MISSION_WAYPOINT_MAX - 1, next.waypoints, 2);
  TEST_ASSERT_EQUAL_UINT8(NAV_RESULT_INVALID, mission_link_result());
  mission_t gap = next;
  gap.count = 5;
  mission_link_commit(&gap);
  TEST_ASSERT_EQUAL_UINT8(NAV_RESULT_INVALID, mission_link_result());
  TEST_ASSERT_EQUAL_UINT8(4, mission.count);

  // A shorter route leaves nothing of the longer one behind.
  mission_t shorter = next;
  shorter.count = 2;
  mission_link_commit(&shorter);
  TEST_ASSERT_EQUAL_UINT8(NAV_RESULT_OK, mission_link_result());
  TEST_ASSERT_EQUAL_UINT8(2, mission.count);
  TEST_ASSERT_EQUAL_INT32(0, mission.waypoints[2].coord.lat);

  // Flash keeps the old route until the vehicle is disarmed and still.
  mission_save_update();
  TEST_ASSERT_EQUAL_UINT8(0, mission_flash().count);
  flags.arm_state = 0;
  state.gps_lock = true;
  state.gps_speed = 5.0f;
  mission_save_update();
  TEST_ASSERT_EQUAL_UINT8(0, mission_flash().count);
  state.gps_speed = 0.0f;
  mission_save_update();
  const mission_t stored = mission_flash();
  TEST_ASSERT_EQUAL_UINT8(2, stored.count);
  TEST_ASSERT_EQUAL_HEX16(mission_crc(&shorter), mission_crc(&stored));

  unlink("flash.bin");
  TEST_ASSERT_EQUAL_INT(0, chdir(original_dir));
  rmdir(directory);
  profile.serial.gps = saved_gps;
  memset(&state, 0, sizeof(state));
  memset(&flags, 0, sizeof(flags));
}
