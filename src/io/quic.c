#include "quic.h"

#include <stdarg.h>
#include <stdio.h>
#include <string.h>

#include "core/debug.h"
#include "core/flash.h"
#include "core/profile.h"
#include "driver/serial.h"
#include "driver/serial_4way.h"
#include "driver/spi_max7456.h"
#include "driver/usb.h"
#include "flight/control.h"
#include "flight/sixaxis.h"
#include "io/blackbox_device.h"
#include "io/usb_configurator.h"
#include "io/vtx.h"
#include "osd/render.h"
#include "util/cbor_helper.h"

#define ENCODE_BUFFER_SIZE 2048

#define quic_errorf(cmd, args...) quic_send_strf(quic, cmd, QUIC_FLAG_ERROR, args)

static uint8_t frame_encode_buffer[ENCODE_BUFFER_SIZE + QUIC_HEADER_LEN];
static uint8_t *encode_buffer = frame_encode_buffer + QUIC_HEADER_LEN;

#define check_cbor_error(cmd)               \
  if (res < CBOR_OK) {                      \
    quic_errorf(cmd, "CBOR ERROR %d", res); \
    return;                                 \
  }

static cbor_result_t cbor_encode_motor_test_t(cbor_value_t *enc, const motor_test_t *b) {
  CBOR_CHECK_ERROR(cbor_result_t res = cbor_encode_map_indefinite(enc));

  CBOR_CHECK_ERROR(res = cbor_encode_str(enc, "active"));
  CBOR_CHECK_ERROR(res = cbor_encode_uint8(enc, &b->active));

  CBOR_CHECK_ERROR(res = cbor_encode_str(enc, "value"));
  CBOR_CHECK_ERROR(res = cbor_encode_float_array(enc, b->value, 4));

  CBOR_CHECK_ERROR(res = cbor_encode_end_indefinite(enc));

  return res;
}

static void quic_send_header(quic_t *quic, quic_command cmd, quic_flag flag, uint32_t len) {
  frame_encode_buffer[0] = QUIC_MAGIC;
  frame_encode_buffer[1] = (cmd & (0xff >> 3)) | (flag & (0xff >> 5)) << 5;
  frame_encode_buffer[2] = (len >> 8) & 0xFF;
  frame_encode_buffer[3] = len & 0xFF;

  if (quic->send) {
    quic->send(frame_encode_buffer, QUIC_HEADER_LEN, quic->priv_data);
  }
}

static void quic_send(quic_t *quic, quic_command cmd, quic_flag flag, uint8_t *data, uint32_t len) {
  frame_encode_buffer[0] = QUIC_MAGIC;
  frame_encode_buffer[1] = (cmd & (0xff >> 3)) | (flag & (0xff >> 5)) << 5;
  frame_encode_buffer[2] = (len >> 8) & 0xFF;
  frame_encode_buffer[3] = len & 0xFF;

  if ((frame_encode_buffer + QUIC_HEADER_LEN) != data) {
    memcpy(frame_encode_buffer + QUIC_HEADER_LEN, data, len);
  }

  if (quic->send) {
    quic->send(frame_encode_buffer, QUIC_HEADER_LEN + len, quic->priv_data);
  }
}

cbor_result_t quic_send_str(quic_t *quic, quic_command cmd, quic_flag flag, const char *str) {
  const uint32_t size = strlen(str) + 128;
  uint8_t buffer[size];

  cbor_value_t enc;
  cbor_encoder_init(&enc, buffer, size);

  cbor_result_t res = cbor_encode_str(&enc, str);
  if (res < CBOR_OK) {
    return res;
  }
  quic_send(quic, cmd, flag, buffer, cbor_encoder_len(&enc));
  return res;
}

static cbor_result_t quic_send_strf(quic_t *quic, quic_command cmd, quic_flag flag, const char *fmt, ...) {
  const uint32_t size = strlen(fmt) + 128;
  char str[size];

  memset(str, 0, size);

  va_list args;
  va_start(args, fmt);
  vsnprintf(str, size, fmt, args);
  va_end(args);
  return quic_send_str(quic, cmd, flag, str);
}

static void get_quic(quic_t *quic, cbor_value_t *dec) {
  cbor_result_t res = CBOR_OK;

  cbor_value_t enc;
  cbor_encoder_init(&enc, encode_buffer, ENCODE_BUFFER_SIZE);

  quic_values value = QUIC_CMD_INVALID;
  res = cbor_decode_uint8(dec, &value);
  check_cbor_error(QUIC_CMD_GET);

  res = cbor_encode_uint8(&enc, &value);
  check_cbor_error(QUIC_CMD_GET);

  switch (value) {
  case QUIC_VAL_PROFILE: {
    res = cbor_encode_profile_t(&enc, &profile);
    check_cbor_error(QUIC_CMD_GET);

    quic_send(quic, QUIC_CMD_GET, QUIC_FLAG_NONE, encode_buffer, cbor_encoder_len(&enc));
    break;
  }
  case QUIC_VAL_DEFAULT_PROFILE: {
    res = cbor_encode_profile_t(&enc, &default_profile);
    check_cbor_error(QUIC_CMD_GET);

    quic_send(quic, QUIC_CMD_GET, QUIC_FLAG_NONE, encode_buffer, cbor_encoder_len(&enc));
    break;
  }
  case QUIC_VAL_INFO:
    res = cbor_encode_target_info_t(&enc, &target_info);
    check_cbor_error(QUIC_CMD_GET);

    quic_send(quic, QUIC_CMD_GET, QUIC_FLAG_NONE, encode_buffer, cbor_encoder_len(&enc));
    break;
  case QUIC_VAL_STATE:
    res = cbor_encode_control_state_t(&enc, &state);
    check_cbor_error(QUIC_CMD_GET);

    quic_send(quic, QUIC_CMD_GET, QUIC_FLAG_NONE, encode_buffer, cbor_encoder_len(&enc));
    break;
  case QUIC_VAL_PID_RATE_PRESETS:
    res = cbor_encode_array(&enc, pid_rate_presets_count);
    check_cbor_error(QUIC_CMD_GET);
    for (uint32_t i = 0; i < pid_rate_presets_count; i++) {
      res = cbor_encode_pid_rate_preset_t(&enc, &pid_rate_presets[i]);
      check_cbor_error(QUIC_CMD_GET);
    }

    quic_send(quic, QUIC_CMD_GET, QUIC_FLAG_NONE, encode_buffer, cbor_encoder_len(&enc));
    break;
  case QUIC_VAL_VTX_SETTINGS:
    res = cbor_encode_vtx_settings_t(&enc, &vtx_settings);
    check_cbor_error(QUIC_CMD_GET);

    quic_send(quic, QUIC_CMD_GET, QUIC_FLAG_NONE, encode_buffer, cbor_encoder_len(&enc));
    break;
  case QUIC_VAL_OSD_FONT: {
    quic_send(quic, QUIC_CMD_GET, QUIC_FLAG_STREAMING, encode_buffer, cbor_encoder_len(&enc));

    uint8_t font[54];

    for (uint16_t i = 0; i < 256; i++) {
      osd_read_character(i, font, 54);

      cbor_encoder_init(&enc, encode_buffer, ENCODE_BUFFER_SIZE);
      res = cbor_encode_bstr(&enc, font, 54);
      check_cbor_error(QUIC_CMD_GET);

      quic_send(quic, QUIC_CMD_GET, QUIC_FLAG_STREAMING, encode_buffer, cbor_encoder_len(&enc));
    }

    quic_send_header(quic, QUIC_CMD_GET, QUIC_FLAG_STREAMING, 0);
    break;
  }
  case QUIC_VAL_BLHEL_SETTINGS: {
    quic_send(quic, QUIC_CMD_GET, QUIC_FLAG_STREAMING, encode_buffer, cbor_encoder_len(&enc));

    const uint8_t count = serial_4way_init();
    time_delay_ms(500);

    for (uint8_t i = 0; i < count; i++) {
      blheli_settings_t settings;
      serial_esc4way_ack_t ack = serial_4way_read_settings(&settings, i);
      if (ack != ESC4WAY_ACK_OK) {
        continue;
      }

      cbor_encoder_init(&enc, encode_buffer, ENCODE_BUFFER_SIZE);
      res = cbor_encode_blheli_settings_t(&enc, &settings);
      check_cbor_error(QUIC_CMD_GET);

      quic_send(quic, QUIC_CMD_GET, QUIC_FLAG_STREAMING, encode_buffer, cbor_encoder_len(&enc));
    }

    serial_4way_release();

    quic_send_header(quic, QUIC_CMD_GET, QUIC_FLAG_STREAMING, 0);
    break;
  }
  case QUIC_VAL_BIND_INFO: {
    res = cbor_encode_rx_bind_storage_t(&enc, &bind_storage);
    check_cbor_error(QUIC_CMD_GET);

    quic_send(quic, QUIC_CMD_GET, QUIC_FLAG_NONE, encode_buffer, cbor_encoder_len(&enc));
    break;
  }
#ifdef DEBUG
  case QUIC_VAL_PERF_COUNTERS: {
    res = cbor_encode_perf_counters(&enc);
    check_cbor_error(QUIC_CMD_GET);
    quic_send(quic, QUIC_CMD_GET, QUIC_FLAG_NONE, encode_buffer, cbor_encoder_len(&enc));
    break;
  }
#endif
  case QUIC_VAL_BLACKBOX_PRESETS: {
    res = cbor_encode_array(&enc, blackbox_presets_count);
    check_cbor_error(QUIC_CMD_GET);
    for (uint32_t i = 0; i < blackbox_presets_count; i++) {
      res = cbor_encode_blackbox_preset_t(&enc, &blackbox_presets[i]);
      check_cbor_error(QUIC_CMD_GET);
    }

    quic_send(quic, QUIC_CMD_GET, QUIC_FLAG_NONE, encode_buffer, cbor_encoder_len(&enc));
    break;
  }
  default:
    quic_errorf(QUIC_CMD_GET, "INVALID VALUE %d", value);
    break;
  }
}

static void set_quic(quic_t *quic, cbor_value_t *dec) {
  cbor_result_t res = CBOR_OK;

  cbor_value_t enc;
  cbor_encoder_init(&enc, encode_buffer, ENCODE_BUFFER_SIZE);

  quic_values value;
  res = cbor_decode_uint8(dec, &value);
  check_cbor_error(QUIC_CMD_SET);

  res = cbor_encode_uint8(&enc, &value);
  check_cbor_error(QUIC_CMD_SET);

  switch (value) {
  case QUIC_VAL_PROFILE: {
    res = cbor_decode_profile_t(dec, &profile);
    check_cbor_error(QUIC_CMD_SET);

    flash_save();

    osd_clear();
    osd_display_reset();

    res = cbor_encode_profile_t(&enc, &profile);
    check_cbor_error(QUIC_CMD_SET);

    quic_send(quic, QUIC_CMD_SET, QUIC_FLAG_NONE, encode_buffer, cbor_encoder_len(&enc));
    break;
  }
  case QUIC_VAL_VTX_SETTINGS: {
    vtx_settings_t settings;

    res = cbor_decode_vtx_settings_t(dec, &settings);
    check_cbor_error(QUIC_CMD_SET);

    vtx_set(&settings);
    flash_save();

    res = cbor_encode_vtx_settings_t(&enc, &vtx_settings);
    check_cbor_error(QUIC_CMD_SET);

    quic_send(quic, QUIC_CMD_SET, QUIC_FLAG_NONE, encode_buffer, cbor_encoder_len(&enc));
    break;
  }
  case QUIC_VAL_OSD_FONT: {
    uint32_t len = 54;

    for (uint16_t i = 0; i < 256; i++) {
      const uint8_t *ptr = NULL;

      res = cbor_decode_bstr(dec, &ptr, &len);
      check_cbor_error(QUIC_CMD_SET);

      osd_write_character(i, ptr, 54);
    }

    res = cbor_encode_str(&enc, "OK");
    check_cbor_error(QUIC_CMD_SET);

    quic_send(quic, QUIC_CMD_SET, QUIC_FLAG_NONE, encode_buffer, cbor_encoder_len(&enc));
    break;
  }
  case QUIC_VAL_BLHEL_SETTINGS: {
    uint8_t count = serial_4way_init();
    time_delay_ms(500);

    for (uint8_t i = 0; i < count; i++) {
      blheli_settings_t settings;

      res = cbor_decode_blheli_settings_t(dec, &settings);
      check_cbor_error(QUIC_CMD_SET);

      serial_esc4way_ack_t ack = serial_4way_write_settings(&settings, i);
      if (ack != ESC4WAY_ACK_OK) {
        break;
      }
    }

    serial_4way_release();

    res = cbor_encode_str(&enc, "OK");
    check_cbor_error(QUIC_CMD_SET);

    quic_send(quic, QUIC_CMD_SET, QUIC_FLAG_NONE, encode_buffer, cbor_encoder_len(&enc));
    break;
  }
  case QUIC_VAL_BIND_INFO: {
    res = cbor_decode_rx_bind_storage_t(dec, &bind_storage);
    check_cbor_error(QUIC_CMD_SET);

    flash_save();
    rx_init();

    res = cbor_encode_rx_bind_storage_t(&enc, &bind_storage);
    check_cbor_error(QUIC_CMD_SET);

    quic_send(quic, QUIC_CMD_SET, QUIC_FLAG_NONE, encode_buffer, cbor_encoder_len(&enc));
    break;
  }
  default:
    quic_errorf(QUIC_CMD_SET, "INVALID VALUE %d", value);
    break;
  }
}

static void process_blackbox(quic_t *quic, cbor_value_t *dec) {
  cbor_result_t res = CBOR_OK;

  cbor_value_t enc;
  cbor_encoder_init(&enc, encode_buffer, ENCODE_BUFFER_SIZE);

  quic_blackbox_command cmd;
  res = cbor_decode_uint8(dec, &cmd);
  check_cbor_error(QUIC_CMD_BLACKBOX);

  switch (cmd) {
  case QUIC_BLACKBOX_RESET:
    blackbox_device_reset();
    quic_send(quic, QUIC_CMD_BLACKBOX, QUIC_FLAG_NONE, NULL, 0);
    break;
  case QUIC_BLACKBOX_LIST:
    res = cbor_encode_map_indefinite(&enc);
    check_cbor_error(QUIC_CMD_BLACKBOX);

    res = cbor_encode_str(&enc, "flash_size");
    check_cbor_error(QUIC_CMD_BLACKBOX);
    const uint32_t size_in_kb = blackbox_bounds.total_size / 1024;
    res = cbor_encode_uint32(&enc, &size_in_kb);
    check_cbor_error(QUIC_CMD_BLACKBOX);

    res = cbor_encode_str(&enc, "file_num");
    check_cbor_error(QUIC_CMD_BLACKBOX);
    res = cbor_encode_uint8(&enc, &blackbox_device_header.file_num);
    check_cbor_error(QUIC_CMD_BLACKBOX);

    res = cbor_encode_str(&enc, "files");
    check_cbor_error(QUIC_CMD_BLACKBOX);
    res = cbor_encode_array(&enc, blackbox_device_header.file_num);
    check_cbor_error(QUIC_CMD_BLACKBOX);

    for (uint8_t i = 0; i < blackbox_device_header.file_num; i++) {
      res = cbor_encode_blackbox_device_file_t(&enc, &blackbox_device_header.files[i]);
      check_cbor_error(QUIC_CMD_BLACKBOX);
    }

    res = cbor_encode_end_indefinite(&enc);
    check_cbor_error(QUIC_CMD_BLACKBOX);

    quic_send(quic, QUIC_CMD_BLACKBOX, QUIC_FLAG_NONE, encode_buffer, cbor_encoder_len(&enc));
    break;
  case QUIC_BLACKBOX_GET: {
    extern blackbox_device_header_t blackbox_device_header;

    uint8_t file_index;
    res = cbor_decode_uint8(dec, &file_index);
    check_cbor_error(QUIC_CMD_BLACKBOX);

    quic_send(quic, QUIC_CMD_BLACKBOX, QUIC_FLAG_STREAMING, encode_buffer, cbor_encoder_len(&enc));

    if (blackbox_device_header.file_num > file_index) {
      const blackbox_device_file_t *file = &blackbox_device_header.files[file_index];

      uint32_t offset = 0;
      while (offset < file->size) {
        const uint32_t size = min(file->size - offset, ENCODE_BUFFER_SIZE);

        blackbox_device_read(file_index, offset, encode_buffer, size);
        quic_send(quic, QUIC_CMD_BLACKBOX, QUIC_FLAG_STREAMING, encode_buffer, size);

        offset += size;
      }
    }

    quic_send_header(quic, QUIC_CMD_BLACKBOX, QUIC_FLAG_STREAMING, 0);

    break;
  }
  default:
    quic_errorf(QUIC_CMD_BLACKBOX, "INVALID CMD %d", cmd);
    break;
  }
}

static void process_motor_test(quic_t *quic, cbor_value_t *dec) {
  cbor_result_t res = CBOR_OK;

  cbor_value_t enc;
  cbor_encoder_init(&enc, encode_buffer, ENCODE_BUFFER_SIZE);

  quic_motor_command cmd;
  res = cbor_decode_uint8(dec, &cmd);
  check_cbor_error(QUIC_CMD_MOTOR);

  switch (cmd) {
  case QUIC_MOTOR_TEST_STATUS:
    res = cbor_encode_motor_test_t(&enc, &motor_test);
    check_cbor_error(QUIC_CMD_MOTOR);

    quic_send(quic, QUIC_CMD_MOTOR, QUIC_FLAG_NONE, encode_buffer, cbor_encoder_len(&enc));
    break;

  case QUIC_MOTOR_TEST_ENABLE:
    motor_test.active = 1;

    res = cbor_encode_uint8(&enc, &motor_test.active);
    check_cbor_error(QUIC_CMD_MOTOR);

    quic_send(quic, QUIC_CMD_MOTOR, QUIC_FLAG_NONE, encode_buffer, cbor_encoder_len(&enc));
    break;

  case QUIC_MOTOR_TEST_DISABLE:
    motor_test.active = 0;

    res = cbor_encode_uint8(&enc, &motor_test.active);
    check_cbor_error(QUIC_CMD_MOTOR);

    quic_send(quic, QUIC_CMD_MOTOR, QUIC_FLAG_NONE, encode_buffer, cbor_encoder_len(&enc));
    break;

  case QUIC_MOTOR_TEST_SET_VALUE:
    res = cbor_decode_float_array(dec, motor_test.value, 4);
    check_cbor_error(QUIC_CMD_MOTOR);

    res = cbor_encode_float_array(&enc, motor_test.value, 4);
    check_cbor_error(QUIC_CMD_MOTOR);

    quic_send(quic, QUIC_CMD_MOTOR, QUIC_FLAG_NONE, encode_buffer, cbor_encoder_len(&enc));
    break;
  case QUIC_MOTOR_ESC4WAY_IF: {
    uint8_t count = serial_4way_init();

    res = cbor_encode_uint8(&enc, &count);
    check_cbor_error(QUIC_CMD_MOTOR);

    quic_send(quic, QUIC_CMD_MOTOR, QUIC_FLAG_EXIT, encode_buffer, cbor_encoder_len(&enc));

    serial_4way_process();
    break;
  }
  default:
    quic_errorf(QUIC_CMD_MOTOR, "INVALID CMD %d", cmd);
    break;
  }
}

static void process_serial(quic_t *quic, cbor_value_t *dec) {
  cbor_result_t res = CBOR_OK;

  cbor_value_t enc;
  cbor_encoder_init(&enc, encode_buffer, ENCODE_BUFFER_SIZE);

  quic_motor_command cmd;
  res = cbor_decode_uint8(dec, &cmd);
  check_cbor_error(QUIC_CMD_SERIAL);

  switch (cmd) {
  case QUIC_SERIAL_ENABLE: {
    serial_ports_t port = SERIAL_PORT_INVALID;
    res = cbor_decode_uint8(dec, &port);
    check_cbor_error(QUIC_CMD_SERIAL);

    uint32_t baudrate = 0;
    res = cbor_decode_uint32(dec, &baudrate);
    check_cbor_error(QUIC_CMD_SERIAL);

    uint8_t half_duplex = 0;
    res = cbor_decode_uint8(dec, &half_duplex);

    uint8_t stop_bits = 1;
    res = cbor_decode_uint8(dec, &stop_bits);

    res = cbor_encode_uint8(&enc, &port);
    check_cbor_error(QUIC_CMD_SERIAL);

    quic_send(quic, QUIC_CMD_SERIAL, QUIC_FLAG_NONE, encode_buffer, cbor_encoder_len(&enc));

    usb_serial_passthrough(port, baudrate, stop_bits, half_duplex);
    break;
  }

  default:
    quic_errorf(QUIC_CMD_SERIAL, "INVALID CMD %d", cmd);
    break;
  }
}

bool quic_process(quic_t *quic, uint8_t *data, uint32_t size) {
  if (size < 4) {
    return false;
  }

  if (data[0] != QUIC_MAGIC) {
    quic_errorf(QUIC_CMD_INVALID, "INVALID MAGIC %d", data[0]);
    return true;
  }

  const uint8_t cmd = data[1];
  if (cmd == QUIC_CMD_INVALID) {
    quic_errorf(QUIC_CMD_INVALID, "INVALID CMD %d", cmd);
    return true;
  }

  const uint16_t payload_size = (uint16_t)data[2] << 8 | data[3];
  if (size < (payload_size + QUIC_HEADER_LEN)) {
    return false;
  }

  cbor_value_t dec;
  cbor_decoder_init(&dec, data + QUIC_HEADER_LEN, payload_size);

  switch (cmd) {
  case QUIC_CMD_GET:
    get_quic(quic, &dec);
    break;
  case QUIC_CMD_SET:
    set_quic(quic, &dec);
    break;
  case QUIC_CMD_CAL_IMU:
    sixaxis_gyro_cal();
    sixaxis_acc_cal();

    flash_save();
    flash_load();

    quic_send(quic, QUIC_CMD_CAL_IMU, QUIC_FLAG_NONE, NULL, 0);
    break;
  case QUIC_CMD_BLACKBOX:
    process_blackbox(quic, &dec);
    break;
  case QUIC_CMD_MOTOR:
    process_motor_test(quic, &dec);
    break;
  case QUIC_CMD_CAL_STICKS:
    stick_wizard_start(true);
    quic_send(quic, QUIC_CMD_CAL_STICKS, QUIC_FLAG_NONE, NULL, 0);
    break;
  case QUIC_CMD_SERIAL:
    process_serial(quic, &dec);
    break;
  default:
    quic_errorf(QUIC_CMD_INVALID, "INVALID CMD %d", cmd);
    break;
  }

  return true;
}