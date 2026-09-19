#include "core/target.h"

#include <string.h>

#include <unity.h>

#include "core/flash.h"
#include "core/profile.h"
#include "driver/fmc.h"
#include "driver/resource.h"
#include "driver/serial.h"
#include "rx/rx.h"

// CBOR fixtures produced the same way as the build injects targets:
// cbor2 over the parsed YAML in script/target_inject.py.

void test_target_defaults_cbor_decode() {
  // defaults:
  //   serial: {rx: 3, smart_audio: 2, hdzero: 4, gps: 101}
  //   receiver: {protocol: crsf}
  //   vtx: {protocol: smart_audio}
  uint8_t populated[] = {
      0xa1, 0x68, 'd', 'e', 'f', 'a', 'u', 'l', 't', 's', 0xa3,
      0x66, 's', 'e', 'r', 'i', 'a', 'l', 0xa4,
      0x62, 'r', 'x', 0x03,
      0x6b, 's', 'm', 'a', 'r', 't', '_', 'a', 'u', 'd', 'i', 'o', 0x02,
      0x66, 'h', 'd', 'z', 'e', 'r', 'o', 0x04,
      0x63, 'g', 'p', 's', 0x18, 0x65,
      0x68, 'r', 'e', 'c', 'e', 'i', 'v', 'e', 'r', 0xa1,
      0x68, 'p', 'r', 'o', 't', 'o', 'c', 'o', 'l', 0x64, 'c', 'r', 's', 'f',
      0x63, 'v', 't', 'x', 0xa1,
      0x68, 'p', 'r', 'o', 't', 'o', 'c', 'o', 'l', 0x6b, 's', 'm', 'a', 'r', 't', '_', 'a', 'u', 'd', 'i', 'o'};

  target_t decoded = {};
  cbor_value_t codec;
  cbor_decoder_init(&codec, populated, sizeof(populated));
  TEST_ASSERT_TRUE(cbor_decode_target_t(&codec, &decoded) >= CBOR_OK);

  TEST_ASSERT_EQUAL(SERIAL_PORT3, decoded.defaults.serial.rx);
  TEST_ASSERT_EQUAL(SERIAL_PORT2, decoded.defaults.serial.smart_audio);
  TEST_ASSERT_EQUAL(SERIAL_PORT4, decoded.defaults.serial.hdzero);
  TEST_ASSERT_EQUAL(SERIAL_SOFT_PORT1, decoded.defaults.serial.gps);
  TEST_ASSERT_EQUAL(RX_PROTOCOL_CRSF, decoded.defaults.receiver.protocol);
  TEST_ASSERT_EQUAL(VTX_PROTOCOL_SMART_AUDIO, decoded.defaults.vtx.protocol);

  // The other explicit serial provider decodes to its top-level protocol
  // value; target_defaults_apply maps it onto the unified serial bind union.
  uint8_t sbus[] = {
      0xa1, 0x68, 'd', 'e', 'f', 'a', 'u', 'l', 't', 's', 0xa1,
      0x68, 'r', 'e', 'c', 'e', 'i', 'v', 'e', 'r', 0xa1,
      0x68, 'p', 'r', 'o', 't', 'o', 'c', 'o', 'l', 0x64, 's', 'b', 'u', 's'};

  decoded = {};
  cbor_decoder_init(&codec, sbus, sizeof(sbus));
  TEST_ASSERT_TRUE(cbor_decode_target_t(&codec, &decoded) >= CBOR_OK);
  TEST_ASSERT_EQUAL(RX_PROTOCOL_SBUS, decoded.defaults.receiver.protocol);

  // The legacy "unified_serial" value and unknown protocol strings are
  // ignored, never guessed; an absent receiver block means autodetection.
  uint8_t legacy[] = {
      0xa1, 0x68, 'd', 'e', 'f', 'a', 'u', 'l', 't', 's', 0xa1,
      0x68, 'r', 'e', 'c', 'e', 'i', 'v', 'e', 'r', 0xa1,
      0x68, 'p', 'r', 'o', 't', 'o', 'c', 'o', 'l', 0x6e, 'u', 'n', 'i', 'f', 'i', 'e', 'd', '_', 's', 'e', 'r', 'i', 'a', 'l'};

  decoded = {};
  cbor_decoder_init(&codec, legacy, sizeof(legacy));
  TEST_ASSERT_TRUE(cbor_decode_target_t(&codec, &decoded) >= CBOR_OK);
  TEST_ASSERT_EQUAL(RX_PROTOCOL_INVALID, decoded.defaults.receiver.protocol);

  // Unknown protocol strings are ignored, never guessed; the serial port
  // assignment still lands.
  uint8_t unknown[] = {
      0xa1, 0x68, 'd', 'e', 'f', 'a', 'u', 'l', 't', 's', 0xa3,
      0x66, 's', 'e', 'r', 'i', 'a', 'l', 0xa1,
      0x62, 'r', 'x', 0x01,
      0x68, 'r', 'e', 'c', 'e', 'i', 'v', 'e', 'r', 0xa1,
      0x68, 'p', 'r', 'o', 't', 'o', 'c', 'o', 'l', 0x6e, 's', 'o', 'm', 'e', '_', 's', 'p', 'i', '_', 't', 'h', 'i', 'n', 'g',
      0x63, 'v', 't', 'x', 0xa1,
      0x68, 'p', 'r', 'o', 't', 'o', 'c', 'o', 'l', 0x67, 'm', 's', 'p', '_', 'v', 't', 'x'};

  decoded = {};
  cbor_decoder_init(&codec, unknown, sizeof(unknown));
  TEST_ASSERT_TRUE(cbor_decode_target_t(&codec, &decoded) >= CBOR_OK);

  TEST_ASSERT_EQUAL(SERIAL_PORT1, decoded.defaults.serial.rx);
  TEST_ASSERT_EQUAL(RX_PROTOCOL_INVALID, decoded.defaults.receiver.protocol);
  TEST_ASSERT_EQUAL(VTX_PROTOCOL_INVALID, decoded.defaults.vtx.protocol);

  // Encoder round-trip, including the all-invalid (omitted) shape.
  for (uint32_t i = 0; i < 2; i++) {
    target_t source = {};
    source.defaults.serial.rx = SERIAL_PORT1;
    source.defaults.serial.smart_audio = SERIAL_SOFT_PORT2;
    source.defaults.receiver.protocol = RX_PROTOCOL_CRSF;
    source.defaults.vtx.protocol = VTX_PROTOCOL_TRAMP;
    if (i == 1) {
      source.defaults = {};
    }

    uint8_t buffer[1024];
    cbor_encoder_init(&codec, buffer, sizeof(buffer));
    TEST_ASSERT_TRUE(cbor_encode_target_t(&codec, &source) >= CBOR_OK);

    target_t roundtrip = {};
    cbor_decoder_init(&codec, buffer, cbor_encoder_len(&codec));
    TEST_ASSERT_TRUE(cbor_decode_target_t(&codec, &roundtrip) >= CBOR_OK);
    TEST_ASSERT_EQUAL_MEMORY(&source.defaults, &roundtrip.defaults, sizeof(target_defaults_t));
  }
}

void test_target_defaults_apply() {
  const target_t backup = target;
  target = {};

  // Ports the target defines: UART2 (tx only), UART3, soft port 1.
  // Soft serial arrays are indexed by port - SERIAL_SOFT_START, matching
  // serial_get_dev(); the index field carries the YAML soft port number.
  target.serial_ports[SERIAL_PORT2] = {.index = SERIAL_PORT2, .tx = PIN_A2};
  target.serial_ports[SERIAL_PORT3] = {.index = SERIAL_PORT3, .rx = PIN_B11, .tx = PIN_B10};
  target.serial_soft_ports[1] = {.index = 1, .tx = PIN_A8};

  target.defaults.serial.rx = SERIAL_PORT3;
  target.defaults.serial.smart_audio = SERIAL_PORT2;
  target.defaults.serial.hdzero = SERIAL_PORT5; // not defined by the target
  target.defaults.serial.gps = SERIAL_SOFT_PORT1;
  target.defaults.vtx.protocol = VTX_PROTOCOL_SMART_AUDIO;

  profile_t resolved = {};

  // An explicit serial provider selects the unified serial decoder via the
  // bind union; the top-level protocol stays unified serial and the selection
  // is saved.
  target.defaults.receiver.protocol = RX_PROTOCOL_CRSF;
  memset(&resolved, 0, sizeof(profile_t));
  profile_set_defaults(&resolved);
  target_defaults_apply(&resolved);

  TEST_ASSERT_EQUAL(SERIAL_PORT3, resolved.serial.rx);
  TEST_ASSERT_EQUAL(SERIAL_PORT2, resolved.serial.smart_audio);
  // UART5 is not a target port: the assignment is skipped.
  TEST_ASSERT_EQUAL(SERIAL_PORT_INVALID, resolved.serial.hdzero);
  TEST_ASSERT_EQUAL(SERIAL_SOFT_PORT1, resolved.serial.gps);

  TEST_ASSERT_EQUAL(RX_PROTOCOL_UNIFIED_SERIAL, resolved.receiver.protocol);
  TEST_ASSERT_EQUAL(RX_SERIAL_PROTOCOL_CRSF, resolved.receiver.bind.unified.protocol);
  TEST_ASSERT_EQUAL_UINT8(1, resolved.receiver.bind.bind_saved);

  TEST_ASSERT_EQUAL(VTX_PROTOCOL_SMART_AUDIO, resolved.vtx.protocol);

  target.defaults.receiver.protocol = RX_PROTOCOL_SBUS;
  memset(&resolved, 0, sizeof(profile_t));
  profile_set_defaults(&resolved);
  target_defaults_apply(&resolved);

  TEST_ASSERT_EQUAL(RX_PROTOCOL_UNIFIED_SERIAL, resolved.receiver.protocol);
  TEST_ASSERT_EQUAL(RX_SERIAL_PROTOCOL_SBUS, resolved.receiver.bind.unified.protocol);
  TEST_ASSERT_EQUAL_UINT8(1, resolved.receiver.bind.bind_saved);

  // The legacy unified_serial value no longer carries a default: apply
  // leaves autodetection untouched, matching an absent receiver block.
  target.defaults.receiver.protocol = RX_PROTOCOL_UNIFIED_SERIAL;
  memset(&resolved, 0, sizeof(profile_t));
  profile_set_defaults(&resolved);
  target_defaults_apply(&resolved);

  TEST_ASSERT_EQUAL(RX_PROTOCOL_UNIFIED_SERIAL, resolved.receiver.protocol);
  TEST_ASSERT_EQUAL(RX_SERIAL_PROTOCOL_INVALID, resolved.receiver.bind.unified.protocol);
  TEST_ASSERT_EQUAL_UINT8(0, resolved.receiver.bind.bind_saved);

  // An absent default leaves the generic defaults in place: unified serial
  // with autodetection (empty bind union) and no VTX protocol.
  target.defaults.receiver = {};
  target.defaults.vtx = {};
  memset(&resolved, 0, sizeof(profile_t));
  profile_set_defaults(&resolved);
  target_defaults_apply(&resolved);

  TEST_ASSERT_EQUAL(RX_PROTOCOL_UNIFIED_SERIAL, resolved.receiver.protocol);
  TEST_ASSERT_EQUAL(RX_SERIAL_PROTOCOL_INVALID, resolved.receiver.bind.unified.protocol);
  TEST_ASSERT_EQUAL_UINT8(0, resolved.receiver.bind.bind_saved);
  TEST_ASSERT_EQUAL(VTX_PROTOCOL_INVALID, resolved.vtx.protocol);

  target = backup;
}

void test_target_defaults_storage_capacity() {
  // Worst case: long name, every port/pin/output populated and all defaults
  // set. DMA entries cannot be populated by native builds (no DMA_STREAMS);
  // bound their contribution instead: 15 devices of at most a 10-byte name
  // key and a 50-byte map entry (uint32 + tag + stream strings + breaks).
  target_t full = {};
  memset(full.name, 'N', sizeof(full.name));
  memset(full.manufacturer, 'M', sizeof(full.manufacturer));

  for (uint32_t i = 1; i < SERIAL_PORT_MAX; i++) {
    full.serial_ports[i] = {.index = (uint8_t)i, .rx = PIN_A0, .tx = PIN_A1, .inverter = PIN_A2};
  }
  for (uint32_t i = 0; i < SERIAL_SOFT_COUNT; i++) {
    full.serial_soft_ports[i] = {.index = (uint8_t)(i + 1), .rx = PIN_A3, .tx = PIN_A4};
  }
  for (uint32_t i = 1; i < SPI_PORT_MAX; i++) {
    full.spi_ports[i] = {.index = (uint8_t)i, .miso = PIN_A5, .mosi = PIN_A6, .sck = PIN_A7};
  }
  for (uint32_t i = 1; i < SDIO_PORT_MAX; i++) {
    full.sdio_ports[i] = {.index = (uint8_t)i, .clk = PIN_C12, .cmd = PIN_D2, .d0 = PIN_C8, .d1 = PIN_C9, .d2 = PIN_C10, .d3 = PIN_C11};
  }
  for (uint32_t i = 1; i < I2C_PORT_MAX; i++) {
    full.i2c_ports[i] = {.index = (uint8_t)i, .sda = PIN_B6, .scl = PIN_B7};
  }
  for (uint32_t i = 0; i < LED_MAX; i++) {
    full.leds[i] = {.pin = PIN_A15, .invert = true};
  }
  full.gyro = {.port = SPI_PORT1, .nss = PIN_A4, .exti = PIN_B0};
  full.osd = {.port = SPI_PORT2, .nss = PIN_B12};
  full.flash = {.port = SPI_PORT3, .nss = PIN_B3};
  full.rx_spi = {.port = SPI_PORT1, .nss = PIN_A4, .exti = PIN_A1, .ant_sel = PIN_A2, .lna_en = PIN_A3, .tx_en = PIN_A5, .busy = PIN_A6, .busy_exti = true, .reset = PIN_A7};
  full.baro = {.port = I2C_PORT1};
  full.usb_detect = PIN_C13;
  full.fpv = PIN_C14;
  full.vbat = PIN_C15;
  full.ibat = PIN_C0;
  full.rgb_led = PIN_C1;
  full.sdcard_detect = {.pin = PIN_C2, .invert = true};
  full.buzzer = {.pin = PIN_C3, .invert = true};
  for (uint32_t i = 0; i < MOTOR_PIN_MAX; i++) {
    full.outputs[i] = {.pin = (gpio_pins_t)(PIN_A0 + i), .caps = OUTPUT_CAP_PWM | OUTPUT_CAP_DSHOT | OUTPUT_CAP_BRUSHED};
  }
  full.vehicles = VEHICLE_TYPE_MULTI | VEHICLE_TYPE_ROVER | VEHICLE_TYPE_WING;
  full.vbat_scale = 0xFFFF;
  full.ibat_scale = 0xFFFF;

  full.defaults.serial.rx = (serial_ports_t)(SERIAL_PORT_MAX - 1);
  full.defaults.serial.smart_audio = SERIAL_SOFT_PORT1;
  full.defaults.serial.hdzero = SERIAL_SOFT_PORT3;
  full.defaults.serial.gps = SERIAL_SOFT_PORT2;
  full.defaults.receiver.protocol = RX_PROTOCOL_CRSF;
  full.defaults.vtx.protocol = VTX_PROTOCOL_SMART_AUDIO;

  uint8_t buffer[TARGET_STORAGE_SIZE];
  cbor_value_t codec;
  cbor_encoder_init(&codec, buffer, TARGET_STORAGE_SIZE);
  TEST_ASSERT_TRUE(cbor_encode_target_t(&codec, &full) >= CBOR_OK);

  constexpr uint32_t dma_worst_case = DMA_DEVICE_MAX * 60;
  TEST_ASSERT_LESS_OR_EQUAL_UINT32(TARGET_STORAGE_SIZE - FMC_MAGIC_SIZE, dma_worst_case + cbor_encoder_len(&codec));
}
