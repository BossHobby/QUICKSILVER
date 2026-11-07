#include <unity.h>
#include <string.h>

#include "control/control.h"
#include "core/profile.h"
#include "driver/serial.h"
#include "driver/time.h"
#include "io/gps.h"
#include "util/datetime.h"

extern serial_port_t serial_gps;

static void gps_test_queue_packet(uint8_t msg_class, uint8_t msg_id, const uint8_t *payload, uint16_t size) {
  uint8_t packet[256 + 8] = {0xb5, 0x62, msg_class, msg_id, uint8_t(size), uint8_t(size >> 8)};
  memcpy(packet + 6, payload, size);
  uint8_t a = 0, b = 0;
  for (unsigned i = 2; i < 6u + size; i++) {
    a += packet[i];
    b += a;
  }
  packet[6 + size] = a;
  packet[7 + size] = b;
  ring_buffer_write_multi(serial_gps.rx_buffer, packet, size + 8);
}

static void gps_test_queue_solution(uint8_t fix, uint8_t satellites, uint8_t flags, uint32_t itow) {
  uint8_t pvt[92] = {};
  memcpy(pvt, &itow, sizeof(itow));
  pvt[20] = fix;
  pvt[21] = flags;
  pvt[23] = satellites;
  gps_test_queue_packet(1, 7, pvt, sizeof(pvt));
}

static void gps_test_feed_ack(bool ack) {
  const uint8_t acked[] = {0x06, 0x8a};
  gps_test_queue_packet(0x05, ack ? 0x01 : 0x00, acked, sizeof(acked));
  gps_task();
}

// A receiver delivering each epoch without delay.
static uint32_t gps_test_itow() {
  return time_millis() + 5000;
}

static void gps_test_feed_solution(uint8_t fix, uint8_t satellites, uint8_t flags) {
  gps_test_queue_solution(fix, satellites, flags, gps_test_itow());
  gps_task();
}

static void gps_test_reset(uint32_t version) {
  profile.serial.gps = SERIAL_PORT1;
  gps_init();
  ring_buffer_clear(serial_gps.tx_buffer);
  ring_buffer_clear(serial_gps.rx_buffer);
  serial_gps.tx_done = true;
  gps_status.version = version;
}

// Value of a key in a CFG-VALSET packet, or -1 when the packet lacks it.
static int64_t gps_test_cfg_value(const uint8_t *packet, uint32_t key) {
  TEST_ASSERT_EQUAL_HEX8(0x06, packet[2]);
  TEST_ASSERT_EQUAL_HEX8(0x8a, packet[3]);
  const uint32_t size = packet[4] | (packet[5] << 8);
  for (uint32_t offset = 4; offset < size;) {
    uint32_t item_key;
    memcpy(&item_key, packet + 6 + offset, sizeof(item_key));
    const uint32_t size_id = (item_key >> 28) & 0x7;
    const uint32_t width = size_id >= 3 ? 1u << (size_id - 2) : 1;
    uint32_t value = 0;
    memcpy(&value, packet + 6 + offset + 4, width);
    if (item_key == key)
      return value;
    offset += 4 + width;
  }
  return -1;
}

static uint32_t gps_test_read_packet(uint8_t *packet) {
  TEST_ASSERT_EQUAL_UINT32(6, ring_buffer_read_multi(serial_gps.tx_buffer, packet, 6));
  const uint32_t payload_size = packet[4] | (packet[5] << 8);
  TEST_ASSERT_TRUE(payload_size <= 120);
  TEST_ASSERT_EQUAL_UINT32(payload_size + 2, ring_buffer_read_multi(serial_gps.tx_buffer, packet + 6, payload_size + 2));
  const uint32_t size = payload_size + 8;
  TEST_ASSERT_EQUAL_HEX8(0xb5, packet[0]);
  TEST_ASSERT_EQUAL_HEX8(0x62, packet[1]);
  TEST_ASSERT_EQUAL_UINT32(size - 8, packet[4] | (packet[5] << 8));
  uint8_t a = 0, b = 0;
  for (uint32_t i = 2; i < size - 2; i++) {
    a += packet[i];
    b += a;
  }
  TEST_ASSERT_EQUAL_UINT8(a, packet[size - 2]);
  TEST_ASSERT_EQUAL_UINT8(b, packet[size - 1]);
  return size;
}

void test_gps_burst_resumes_partial_packet() {
  gps_test_reset(0x000A0000U);
  flags.arm_state = true;
  gps_status.state = GPS_RUNNING_NAV_SAT_OFF;
  state.gps_sats = 0;
  const uint8_t noise[100] = {};
  TEST_ASSERT_EQUAL_UINT32(sizeof(noise), ring_buffer_write_multi(serial_gps.rx_buffer, noise, sizeof(noise)));
  gps_test_queue_solution(GPS_FIX_3D, 12, 1, gps_test_itow());
  gps_test_queue_solution(GPS_FIX_3D, 10, 1, gps_test_itow() + 100);

  gps_task(); // Byte budget ends inside the first solution.
  TEST_ASSERT_EQUAL_UINT32(172, ring_buffer_available(serial_gps.rx_buffer));
  TEST_ASSERT_EQUAL_UINT8(0, state.gps_sats);
  gps_task(); // Finish that solution, leaving the next packet queued.
  TEST_ASSERT_EQUAL_UINT32(100, ring_buffer_available(serial_gps.rx_buffer));
  TEST_ASSERT_EQUAL_UINT8(12, state.gps_sats);
  TEST_ASSERT_TRUE(state.gps_health.lock);
  gps_task();
  TEST_ASSERT_EQUAL_UINT32(0, ring_buffer_available(serial_gps.rx_buffer));
  TEST_ASSERT_EQUAL_UINT8(10, state.gps_sats);
  TEST_ASSERT_TRUE(state.gps_health.lock);

  profile.serial.gps = SERIAL_PORT_INVALID;
  gps_init();
  flags.arm_state = false;
}

void test_gps_configuration_and_fix_validity() {
  profile_set_defaults(&profile);
  profile_output_update();
  TEST_ASSERT_EQUAL_UINT8(GPS_CONSTELLATION_GPS | GPS_CONSTELLATION_GALILEO, profile.gps.constellations);
  uint8_t encoded[4096];
  cbor_value_t encoder;
  cbor_encoder_init(&encoder, encoded, sizeof(encoded));
  TEST_ASSERT_TRUE(cbor_encode_profile_t(&encoder, &profile) >= CBOR_OK);
  profile_t decoded = {};
  cbor_value_t decoder;
  cbor_decoder_init(&decoder, encoded, sizeof(encoded));
  TEST_ASSERT_TRUE(cbor_decode_profile_t(&decoder, &decoded) >= CBOR_OK);
  TEST_ASSERT_EQUAL_UINT8(profile.gps.constellations, decoded.gps.constellations);
  uint8_t packet[128];
  const struct {
    uint32_t version;
    uint16_t interval_ms;
  } receivers[] = {{0x00080000U, 100}, {0x00190000U, 40}, {0x000A0000U, 100}};
  for (const auto &receiver : receivers) {
    gps_test_reset(receiver.version);
    gps_status.state = GPS_CONFIG_CONSTELLATIONS;
    gps_task();
    if (receiver.version == 0x00080000U) {
      TEST_ASSERT_EQUAL_UINT32(0, ring_buffer_read_multi(serial_gps.tx_buffer, packet, sizeof(packet)));
    } else {
      gps_test_read_packet(packet);
      TEST_ASSERT_EQUAL(1, gps_test_cfg_value(packet, 0x1031001f));  // GPS
      TEST_ASSERT_EQUAL(1, gps_test_cfg_value(packet, 0x10310001));  // GPS L1C/A
      TEST_ASSERT_EQUAL(0, gps_test_cfg_value(packet, 0x10310025));  // GLONASS
      TEST_ASSERT_EQUAL(1, gps_test_cfg_value(packet, 0x10310021));  // Galileo
      TEST_ASSERT_EQUAL(0, gps_test_cfg_value(packet, 0x10310022));  // BeiDou
      TEST_ASSERT_EQUAL(receiver.version == 0x000A0000U ? 0 : -1, gps_test_cfg_value(packet, 0x1031000f)); // B1C key is M10-only
      gps_test_feed_ack(true);
    }
    TEST_ASSERT_EQUAL(GPS_CONFIG_UPDATE_RATE, gps_status.state);
    gps_task();
    gps_test_read_packet(packet);
    TEST_ASSERT_EQUAL_HEX8(0x06, packet[2]);
    if (receiver.version == 0x00080000U) {
      TEST_ASSERT_EQUAL_HEX8(0x08, packet[3]);
      const uint8_t rate[] = {100, 0, 1, 0, 1, 0};
      TEST_ASSERT_EQUAL_MEMORY(rate, packet + 6, sizeof(rate));
    } else {
      TEST_ASSERT_EQUAL(receiver.interval_ms, gps_test_cfg_value(packet, 0x30210001));
      TEST_ASSERT_EQUAL(1, gps_test_cfg_value(packet, 0x30210002));
      TEST_ASSERT_EQUAL(1, gps_test_cfg_value(packet, 0x20210003));
    }
    flags.arm_state = false;
    state.gps_speed = 0;
    time_test_advance_us(600000);
    gps_task(); // Initial stationary acquisition model.
    gps_test_read_packet(packet);
    if (receiver.version == 0x00080000U) {
      TEST_ASSERT_EQUAL_HEX8(0x24, packet[3]);
      TEST_ASSERT_EQUAL_UINT8(1, packet[6]); // Only dynamic-model mask.
      TEST_ASSERT_EQUAL_UINT8(2, packet[8]);
    } else {
      const uint8_t model[] = {0, 1, 0, 0, 0x21, 0, 0x11, 0x20, 2};
      TEST_ASSERT_EQUAL_HEX8(0x8a, packet[3]);
      TEST_ASSERT_EQUAL_MEMORY(model, packet + 6, sizeof(model));
    }
  }

  // Feed real UBX packets through the serial parser, including valid-checksum
  // 2D/time-only solutions that must not authorize heading or home capture.
  gps_test_reset(0x000A0000U);
  for (uint8_t fix = 0; fix <= 5; fix++) {
    gps_status.state = GPS_RUNNING_NAV_SAT_OFF;
    gps_test_feed_solution(fix, 12, 1);
    TEST_ASSERT_EQUAL(fix == GPS_FIX_3D || fix == GPS_FIX_GNSS_DR, state.gps_health.lock);
  }
  profile.serial.gps = SERIAL_PORT_INVALID;
  gps_init();
}

void test_gps_m10_constellations_select_rate_and_fall_back() {
  uint8_t packet[128];
  const struct {
    uint8_t selection;
    uint16_t interval_ms;
  } cases[] = {
      {GPS_CONSTELLATION_GPS, 56},
      {GPS_CONSTELLATION_GPS | GPS_CONSTELLATION_GALILEO | GPS_CONSTELLATION_BEIDOU, 125},
      {GPS_CONSTELLATION_GPS | GPS_CONSTELLATION_GALILEO | GPS_CONSTELLATION_GLONASS, 167},
      {0x0f, 250},
  };
  for (const auto &test : cases) {
    gps_test_reset(0x000A0000U);
    profile.gps.constellations = test.selection;
    gps_status.state = GPS_CONFIG_CONSTELLATIONS;
    gps_task();
    gps_test_read_packet(packet);
    const bool beidou = test.selection & GPS_CONSTELLATION_BEIDOU;
    TEST_ASSERT_EQUAL(0, gps_test_cfg_value(packet, 0x1031000d)); // never B1I on the M10
    TEST_ASSERT_EQUAL(beidou, gps_test_cfg_value(packet, 0x1031000f));
    TEST_ASSERT_EQUAL((test.selection & GPS_CONSTELLATION_GLONASS) != 0, gps_test_cfg_value(packet, 0x10310018));
    gps_test_feed_ack(true);
    gps_task();
    gps_test_read_packet(packet);
    TEST_ASSERT_EQUAL(test.interval_ms, gps_test_cfg_value(packet, 0x30210001));
  }

  // A rejected selection falls back to the plain GPS + Galileo switches.
  gps_test_reset(0x000A0000U);
  profile.gps.constellations = 0x0f;
  gps_status.state = GPS_CONFIG_CONSTELLATIONS;
  gps_task();
  gps_test_read_packet(packet);
  gps_test_feed_ack(false);
  gps_test_read_packet(packet);
  TEST_ASSERT_EQUAL(1, gps_test_cfg_value(packet, 0x1031001f));
  TEST_ASSERT_EQUAL(0, gps_test_cfg_value(packet, 0x10310025));
  TEST_ASSERT_EQUAL(1, gps_test_cfg_value(packet, 0x10310021));
  TEST_ASSERT_EQUAL(0, gps_test_cfg_value(packet, 0x10310022));
  TEST_ASSERT_EQUAL(-1, gps_test_cfg_value(packet, 0x1031000f));
  TEST_ASSERT_EQUAL(GPS_CONFIG_CONSTELLATIONS, gps_status.state);
  gps_test_feed_ack(false); // Nothing left to fall back to; continue.
  TEST_ASSERT_EQUAL(GPS_CONFIG_UPDATE_RATE, gps_status.state);
  gps_task();
  gps_test_read_packet(packet);
  // The receiver kept an unknown selection: run at the rate all four sustain.
  TEST_ASSERT_EQUAL(250, gps_test_cfg_value(packet, 0x30210001));

  // An unanswered selection is not confirmed either.
  gps_test_reset(0x000A0000U);
  profile.gps.constellations = GPS_CONSTELLATION_GPS;
  gps_status.state = GPS_CONFIG_CONSTELLATIONS;
  gps_task();
  gps_test_read_packet(packet);
  time_test_advance_us(600000);
  gps_task();
  TEST_ASSERT_EQUAL(GPS_CONFIG_UPDATE_RATE, gps_status.state);
  gps_task();
  gps_test_read_packet(packet);
  TEST_ASSERT_EQUAL(250, gps_test_cfg_value(packet, 0x30210001));

  // Interference reports follow the navigation rate down to 1 Hz.
  time_test_advance_us(600000);
  gps_task(); // dynamic model
  gps_test_read_packet(packet);
  time_test_advance_us(600000);
  gps_task(); // SBAS
  gps_test_read_packet(packet);
  time_test_advance_us(600000);
  gps_task(); // messages
  gps_test_read_packet(packet); // NAV-PVT
  gps_test_read_packet(packet); // NAV-SAT
  TEST_ASSERT_EQUAL(2, gps_test_cfg_value(packet, 0x20910016));
  gps_test_read_packet(packet);
  TEST_ASSERT_EQUAL(1, gps_test_cfg_value(packet, 0x1041000d));
  TEST_ASSERT_EQUAL(4, gps_test_cfg_value(packet, 0x2091035a));
  gps_test_read_packet(packet);
  TEST_ASSERT_EQUAL(4, gps_test_cfg_value(packet, 0x20910635));
  TEST_ASSERT_EQUAL(GPS_WAITING_FOR_LOCK, gps_status.state);

  profile_set_defaults(&profile);
  profile.serial.gps = SERIAL_PORT_INVALID;
  gps_init();
}

static void gps_test_feed_mon_rf(uint8_t jamming, uint8_t antenna) {
  uint8_t rf[4 + 24] = {0, 1};
  uint8_t *block = rf + 4;
  block[1] = jamming;
  block[2] = antenna;
  block[12] = 0x34; // noise 0x0134
  block[13] = 0x01;
  block[14] = 0x00; // AGC 0x1000
  block[15] = 0x10;
  block[16] = 42;
  gps_test_queue_packet(0x0a, 0x38, rf, sizeof(rf));
  gps_task();
}

void test_gps_interference_reports() {
  gps_test_reset(0x00190000U);
  gps_status.state = GPS_WAITING_FOR_LOCK;
  flags.arm_state = false;
  time_test_set_us(10000000);
  gps_test_feed_mon_rf(GPS_JAMMING_WARNING, GPS_ANTENNA_OK);
  TEST_ASSERT_EQUAL_UINT8(GPS_JAMMING_WARNING, state.gps_health.jamming);
  TEST_ASSERT_EQUAL_UINT8(GPS_ANTENNA_OK, gps_status.antenna_status);
  TEST_ASSERT_EQUAL_UINT16(0x0134, gps_status.noise_per_ms);
  TEST_ASSERT_EQUAL_UINT16(0x1000, gps_status.agc_count);
  TEST_ASSERT_EQUAL_UINT8(42, gps_status.cw_jamming);
  time_test_advance_us(3001000);
  gps_task();
  TEST_ASSERT_EQUAL_UINT8(GPS_JAMMING_UNKNOWN, state.gps_health.jamming); // stale

  // An M10 without SEC-SIG (SPG 5.10) reports jamming in MON-RF.
  gps_test_reset(0x000A0000U);
  gps_status.state = GPS_WAITING_FOR_LOCK;
  gps_test_feed_mon_rf(GPS_JAMMING_CRITICAL, GPS_ANTENNA_SHORT);
  TEST_ASSERT_EQUAL_UINT8(GPS_JAMMING_CRITICAL, state.gps_health.jamming);
  TEST_ASSERT_EQUAL_UINT8(GPS_ANTENNA_SHORT, gps_status.antenna_status);

  // Where SEC-SIG reports, it owns jamming and adds spoofing, in both layouts.
  const uint8_t v1[12] = {1, 0, 0, 0, 0x01 | (GPS_JAMMING_OK << 1), 0, 0, 0, 0x01 | (GPS_SPOOFING_INDICATED << 1)};
  gps_test_queue_packet(0x27, 0x09, v1, sizeof(v1));
  gps_task();
  TEST_ASSERT_EQUAL_UINT8(GPS_JAMMING_OK, state.gps_health.jamming);
  TEST_ASSERT_EQUAL_UINT8(GPS_SPOOFING_INDICATED, state.gps_health.spoofing);
  const uint8_t v2[4] = {2, uint8_t(0x01 | (GPS_JAMMING_CRITICAL << 1) | 0x08 | (GPS_SPOOFING_OK << 4)), 0, 0};
  gps_test_queue_packet(0x27, 0x09, v2, sizeof(v2));
  gps_task();
  TEST_ASSERT_EQUAL_UINT8(GPS_JAMMING_CRITICAL, state.gps_health.jamming);
  TEST_ASSERT_EQUAL_UINT8(GPS_SPOOFING_OK, state.gps_health.spoofing);
  gps_test_feed_mon_rf(GPS_JAMMING_OK, GPS_ANTENNA_OK);
  TEST_ASSERT_EQUAL_UINT8(GPS_JAMMING_CRITICAL, state.gps_health.jamming);
  const uint8_t reserved[4] = {2, uint8_t(0x08 | (6 << 4)), 0, 0};
  gps_test_queue_packet(0x27, 0x09, reserved, sizeof(reserved));
  gps_task();
  TEST_ASSERT_EQUAL_UINT8(GPS_SPOOFING_UNKNOWN, state.gps_health.spoofing);
  time_test_advance_us(3001000);
  gps_task();
  TEST_ASSERT_EQUAL_UINT8(GPS_JAMMING_UNKNOWN, state.gps_health.jamming);

  profile.serial.gps = SERIAL_PORT_INVALID;
  gps_init();
}

void test_gps_ground_and_airborne_configuration() {
  gps_test_reset(0x000A0000U);
  gps_status.state = GPS_WAITING_FOR_LOCK;
  flags.arm_state = false;
  state.gps_health.lock = false;
  uint8_t packet[128];
  const auto expect_model = [&](uint8_t model) {
    gps_test_read_packet(packet);
    TEST_ASSERT_EQUAL_HEX8(0x8a, packet[3]);
    TEST_ASSERT_EQUAL_HEX8(0x21, packet[10]); // Dynamic-model key.
    TEST_ASSERT_EQUAL_UINT8(model, packet[14]);
    TEST_ASSERT_EQUAL_UINT32(0, ring_buffer_read_multi(serial_gps.tx_buffer, packet, sizeof(packet)));
  };
  const auto expect_nav_sat = [&](uint8_t rate) {
    gps_test_read_packet(packet);
    TEST_ASSERT_EQUAL_HEX8(0x16, packet[10]); // NAV-SAT UART1 key.
    TEST_ASSERT_EQUAL_UINT8(rate, packet[14]);
  };

  // The model goes alone; satellite diagnostics follow its acknowledgement.
  gps_task();
  expect_model(2);
  gps_test_feed_ack(true);
  expect_nav_sat(5);
  state.gps_health.lock = true;
  gps_task();
  TEST_ASSERT_EQUAL_UINT32(0, ring_buffer_read_multi(serial_gps.tx_buffer, packet, sizeof(packet)));

  // Arm without a fix: the stationary model must not remain active in
  // flight, so a lost acknowledgement sends the airborne model again.
  flags.arm_state = true;
  state.gps_health.lock = false;
  gps_task();
  expect_model(8);
  time_test_advance_us(600000);
  gps_task();
  TEST_ASSERT_EQUAL_UINT32(0, ring_buffer_read_multi(serial_gps.tx_buffer, packet, sizeof(packet)));
  gps_task();
  expect_model(8);
  gps_test_feed_ack(true);
  expect_nav_sat(0);

  // A rejection does not settle the model either.
  flags.arm_state = false;
  gps_task();
  expect_model(2);
  gps_test_feed_ack(false);
  TEST_ASSERT_EQUAL_UINT32(0, ring_buffer_read_multi(serial_gps.tx_buffer, packet, sizeof(packet)));
  gps_task();
  expect_model(2);
  gps_test_feed_ack(true);
  expect_nav_sat(5);
  profile.serial.gps = SERIAL_PORT_INVALID;
  gps_init();
}

void test_gps_satellite_and_fix_loss_remain_visible_armed() {
  gps_test_reset(0x000A0000U);
  flags.arm_state = true;
  gps_status.state = GPS_RUNNING_NAV_SAT_OFF;
  gps_test_feed_solution(GPS_FIX_3D, 12, 1);
  TEST_ASSERT_TRUE(state.gps_health.lock);
  TEST_ASSERT_EQUAL_UINT8(12, state.gps_sats);
  time_test_advance_us(100000);
  gps_test_feed_solution(GPS_FIX_3D, 3, 1);
  TEST_ASSERT_FALSE(state.gps_health.lock);
  TEST_ASSERT_EQUAL_UINT8(3, state.gps_sats);
  gps_test_feed_solution(GPS_FIX_NONE, 0, 0);
  TEST_ASSERT_FALSE(state.gps_health.lock);
  TEST_ASSERT_EQUAL_UINT8(0, state.gps_sats);
  gps_test_feed_solution(GPS_FIX_3D, 12, 0);
  TEST_ASSERT_FALSE(state.gps_health.lock); // Satellites alone do not establish a fix.
  gps_test_feed_solution(GPS_FIX_3D, 12, 1);
  TEST_ASSERT_TRUE(state.gps_health.lock);
  time_test_advance_us(501000);
  gps_task();
  TEST_ASSERT_FALSE(state.gps_health.lock); // A silent receiver cannot leave LOCKED set.
  gps_test_feed_solution(GPS_FIX_3D, 10, 1);
  TEST_ASSERT_TRUE(state.gps_health.lock);
  TEST_ASSERT_EQUAL_UINT8(10, state.gps_sats);
  profile.serial.gps = SERIAL_PORT_INVALID;
  gps_init();
  flags.arm_state = false;
}

void test_gps_stamps_solutions_by_receiver_epoch() {
  gps_test_reset(0x000A0000U);
  flags.arm_state = true;
  gps_status.state = GPS_RUNNING_NAV_SAT_OFF;
  time_test_set_us(1000000);

  gps_test_queue_solution(GPS_FIX_3D, 12, 1, 5000);
  gps_task();
  TEST_ASSERT_EQUAL_UINT32(1000, state.gps_last_update_ms);

  // Delivery jitter does not move the measurement spacing.
  time_test_set_us(1104000);
  gps_test_queue_solution(GPS_FIX_3D, 12, 1, 5100);
  gps_task();
  TEST_ASSERT_UINT32_WITHIN(1, 1100, state.gps_last_update_ms);
  TEST_ASSERT_EQUAL_UINT8(10, gps_status.update_rate);

  // A burst after a stall keeps the age of each solution.
  time_test_set_us(1480000);
  gps_test_queue_solution(GPS_FIX_3D, 12, 1, 5200);
  gps_test_queue_solution(GPS_FIX_3D, 12, 1, 5300);
  gps_task();
  TEST_ASSERT_UINT32_WITHIN(2, 1200, state.gps_last_update_ms);
  gps_task();
  TEST_ASSERT_UINT32_WITHIN(3, 1300, state.gps_last_update_ms);
  TEST_ASSERT_TRUE(state.gps_health.lock);
  time_test_set_us(1840000);
  gps_task();
  TEST_ASSERT_FALSE(state.gps_health.lock); // Measured over 500 ms ago, though delivered 360 ms ago.

  // A faster delivery re-anchors without stamping ahead of arrival.
  time_test_set_us(1850000);
  gps_test_queue_solution(GPS_FIX_3D, 12, 1, 5850);
  gps_task();
  TEST_ASSERT_EQUAL_UINT32(1850, state.gps_last_update_ms);
  TEST_ASSERT_TRUE(state.gps_health.lock);

  // Week rollover re-anchors on arrival.
  time_test_set_us(1950000);
  gps_test_queue_solution(GPS_FIX_3D, 12, 1, 0);
  gps_task();
  TEST_ASSERT_EQUAL_UINT32(1950, state.gps_last_update_ms);

  profile.serial.gps = SERIAL_PORT_INVALID;
  gps_init();
  flags.arm_state = false;
}

void test_datetime_epoch_conversion() {
  const struct {
    datetime_t dt;
    int64_t epoch_ms;
  } cases[] = {
      {{1970, 1, 1, 0, 0, 0, 0}, 0},
      {{2000, 2, 29, 12, 34, 56, 789}, 951827696789LL},
      {{2024, 12, 31, 23, 59, 59, 999}, 1735689599999LL},
      {{2100, 3, 1, 0, 0, 0, 0}, 4107542400000LL},
  };
  for (const auto &test : cases) {
    TEST_ASSERT_EQUAL_INT64(test.epoch_ms, datetime_to_epoch_ms(&test.dt));
    datetime_t decoded = {};
    datetime_from_epoch_ms(test.epoch_ms, &decoded);
    TEST_ASSERT_EQUAL_MEMORY(&test.dt, &decoded, sizeof(decoded));
  }
}

static void gps_test_feed_time(uint8_t valid, int32_t nano) {
  uint8_t pvt[100] = {0xb5, 0x62, 1, 7, 92, 0};
  uint8_t *payload = pvt + 6;
  payload[4] = 2026 & 0xff;
  payload[5] = 2026 >> 8;
  payload[6] = 10;
  payload[7] = 21;
  payload[8] = 23;
  payload[9] = 59;
  payload[10] = 59;
  payload[11] = valid;
  memcpy(payload + 16, &nano, sizeof(nano));
  uint8_t a = 0, b = 0;
  for (unsigned i = 2; i < sizeof(pvt) - 2; i++) {
    a += pvt[i];
    b += a;
  }
  pvt[98] = a;
  pvt[99] = b;
  ring_buffer_write_multi(serial_gps.rx_buffer, pvt, sizeof(pvt));
  gps_task();
}

void test_gps_time_sets_free_running_clock() {
  gps_test_reset(0x000A0000U);
  datetime_test_reset();
  time_test_set_us(1000000);

  // Date and time without full resolution may still be off by whole seconds.
  gps_test_feed_time(0x03, 0);
  TEST_ASSERT_FALSE(datetime_valid());

  gps_test_feed_time(0x07, 250000000);
  datetime_t now = {};
  TEST_ASSERT_TRUE(datetime_now(&now));
  const datetime_t expected = {2026, 10, 21, 23, 59, 59, 250};
  TEST_ASSERT_EQUAL_MEMORY(&expected, &now, sizeof(now));

  // Keeps counting from the last anchor once GPS goes quiet, across midnight.
  time_test_advance_us(1500000);
  TEST_ASSERT_TRUE(datetime_now(&now));
  const datetime_t later = {2026, 10, 22, 0, 0, 0, 750};
  TEST_ASSERT_EQUAL_MEMORY(&later, &now, sizeof(now));

  // Negative nano belongs to the rounded-up second.
  gps_test_feed_time(0x07, -100000000);
  TEST_ASSERT_TRUE(datetime_now(&now));
  const datetime_t rounded = {2026, 10, 21, 23, 59, 58, 900};
  TEST_ASSERT_EQUAL_MEMORY(&rounded, &now, sizeof(now));
}
