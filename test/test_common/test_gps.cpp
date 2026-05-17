#include <unity.h>
#include <string.h>

#include "control/control.h"
#include "core/profile.h"
#include "driver/serial.h"
#include "driver/time.h"
#include "io/gps.h"

extern serial_port_t serial_gps;

static void gps_test_feed_solution(uint8_t fix, uint8_t satellites, uint8_t flags) {
  uint8_t pvt[100] = {0xb5, 0x62, 1, 7, 92, 0};
  pvt[6 + 20] = fix;
  pvt[6 + 21] = flags;
  pvt[6 + 23] = satellites;
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

static void gps_test_reset(uint32_t version) {
  profile.serial.gps = SERIAL_PORT1;
  gps_init();
  ring_buffer_clear(serial_gps.tx_buffer);
  ring_buffer_clear(serial_gps.rx_buffer);
  serial_gps.tx_done = true;
  gps_status.version = version;
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

void test_gps_configuration_and_fix_validity() {
  profile_set_defaults();
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
  const uint32_t versions[] = {VER_M8, VER_M9, VER_M10};
  for (uint32_t version : versions) {
    gps_test_reset(version);
    gps_status.state = GPS_CONFIG_CONSTELLATIONS;
    gps_task();
    if (version == VER_M8) {
      TEST_ASSERT_EQUAL_UINT32(0, ring_buffer_read_multi(serial_gps.tx_buffer, packet, sizeof(packet)));
    } else {
      gps_test_read_packet(packet);
      const uint8_t selection[] = {0, 1, 0, 0,
          0x1f, 0, 0x31, 0x10, 1, 0x25, 0, 0x31, 0x10, 0,
          0x21, 0, 0x31, 0x10, 1, 0x22, 0, 0x31, 0x10, 0};
      TEST_ASSERT_EQUAL_HEX8(0x8a, packet[3]);
      TEST_ASSERT_EQUAL_MEMORY(selection, packet + 6, sizeof(selection));
    }
    gps_status.state = GPS_CONFIG_UPDATE_RATE;
    gps_task();
    gps_test_read_packet(packet);
    TEST_ASSERT_EQUAL_HEX8(0x06, packet[2]);
    if (version == VER_M8) {
      TEST_ASSERT_EQUAL_HEX8(0x08, packet[3]);
      const uint8_t rate[] = {100, 0, 1, 0, 1, 0};
      TEST_ASSERT_EQUAL_MEMORY(rate, packet + 6, sizeof(rate));
    } else {
      TEST_ASSERT_EQUAL_HEX8(0x8a, packet[3]);
      const uint8_t config[] = {0, 1, 0, 0, 1, 0, 0x21, 0x30, 100, 0,
          2, 0, 0x21, 0x30, 1, 0, 3, 0, 0x21, 0x20, 1};
      TEST_ASSERT_EQUAL_MEMORY(config, packet + 6, sizeof(config));
    }
    flags.arm_state = false;
    state.gps_speed = 0;
    time_test_advance_us(600000);
    gps_task(); // Initial stationary acquisition model.
    gps_test_read_packet(packet);
    if (version == VER_M8) {
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
  gps_test_reset(VER_M10);
  profile.gps.constellations = 0;
  gps_status.state = GPS_CONFIG_CONSTELLATIONS;
  gps_task();
  gps_test_read_packet(packet);
  TEST_ASSERT_EQUAL_HEX8(0x8a, packet[3]);
  const uint8_t default_selection[] = {0, 1, 0, 0,
      0x1f, 0, 0x31, 0x10, 1, 0x25, 0, 0x31, 0x10, 0,
      0x21, 0, 0x31, 0x10, 1, 0x22, 0, 0x31, 0x10, 0};
  TEST_ASSERT_EQUAL_MEMORY(default_selection, packet + 6, sizeof(default_selection));
  for (uint8_t fix = 0; fix <= 5; fix++) {
    gps_status.state = GPS_RUNNING_NAV_SAT_OFF;
    gps_test_feed_solution(fix, 12, 1);
    TEST_ASSERT_EQUAL(fix == GPS_FIX_3D || fix == GPS_FIX_GNSS_DR, state.gps_lock);
  }
  profile.serial.gps = SERIAL_PORT_INVALID;
  gps_init();
}

void test_gps_ground_and_airborne_configuration() {
  gps_test_reset(VER_M10);
  gps_status.state = GPS_WAITING_FOR_LOCK;
  flags.arm_state = false;
  state.gps_lock = false;
  gps_task();
  uint8_t packet[128];
  gps_test_read_packet(packet);
  TEST_ASSERT_EQUAL_HEX8(0x8a, packet[3]);
  TEST_ASSERT_EQUAL_HEX8(0x21, packet[10]); // Dynamic-model key.
  TEST_ASSERT_EQUAL_UINT8(2, packet[14]);
  gps_test_read_packet(packet);
  TEST_ASSERT_EQUAL_HEX8(0x16, packet[10]); // NAV-SAT UART1 key.
  TEST_ASSERT_EQUAL_UINT8(5, packet[14]);
  state.gps_lock = true;
  gps_task();
  TEST_ASSERT_EQUAL_UINT32(0, ring_buffer_read_multi(serial_gps.tx_buffer, packet, sizeof(packet)));

  // Arm without a fix: the stationary model must not remain active in flight.
  flags.arm_state = true;
  state.gps_lock = false;
  gps_task();
  gps_test_read_packet(packet);
  TEST_ASSERT_EQUAL_UINT8(8, packet[14]);
  gps_test_read_packet(packet);
  TEST_ASSERT_EQUAL_UINT8(0, packet[14]);

  flags.arm_state = false;
  gps_task();
  gps_test_read_packet(packet);
  TEST_ASSERT_EQUAL_UINT8(2, packet[14]);
  gps_test_read_packet(packet);
  TEST_ASSERT_EQUAL_UINT8(5, packet[14]);
  profile.serial.gps = SERIAL_PORT_INVALID;
  gps_init();
}

void test_gps_satellite_and_fix_loss_remain_visible_armed() {
  gps_test_reset(VER_M10);
  flags.arm_state = true;
  gps_status.state = GPS_RUNNING_NAV_SAT_OFF;
  gps_test_feed_solution(GPS_FIX_3D, 12, 1);
  TEST_ASSERT_TRUE(state.gps_lock);
  TEST_ASSERT_EQUAL_UINT8(12, state.gps_sats);
  time_test_advance_us(100000);
  gps_test_feed_solution(GPS_FIX_3D, 3, 1);
  TEST_ASSERT_FALSE(state.gps_lock);
  TEST_ASSERT_EQUAL_UINT8(3, state.gps_sats);
  gps_test_feed_solution(GPS_FIX_NONE, 0, 0);
  TEST_ASSERT_FALSE(state.gps_lock);
  TEST_ASSERT_EQUAL_UINT8(0, state.gps_sats);
  gps_test_feed_solution(GPS_FIX_3D, 12, 0);
  TEST_ASSERT_FALSE(state.gps_lock); // Satellites alone do not establish a fix.
  gps_test_feed_solution(GPS_FIX_3D, 12, 1);
  TEST_ASSERT_TRUE(state.gps_lock);
  time_test_advance_us(501000);
  gps_task();
  TEST_ASSERT_FALSE(state.gps_lock); // A silent receiver cannot leave LOCKED set.
  gps_test_feed_solution(GPS_FIX_3D, 10, 1);
  TEST_ASSERT_TRUE(state.gps_lock);
  TEST_ASSERT_EQUAL_UINT8(10, state.gps_sats);
  profile.serial.gps = SERIAL_PORT_INVALID;
  gps_init();
  flags.arm_state = false;
}
