#include "usb_configurator.h"

#include <stdarg.h>
#include <stdio.h>
#include <string.h>

#include "blackbox.h"
#include "data_flash.h"
#include "debug.h"
#include "drv_serial.h"
#include "drv_serial_4way.h"
#include "drv_serial_soft.h"
#include "drv_spi_max7456.h"
#include "drv_time.h"
#include "drv_usb.h"
#include "flash.h"
#include "flight/control.h"
#include "flight/sixaxis.h"
#include "led.h"
#include "osd_render.h"
#include "profile.h"
#include "project.h"
#include "util/cbor_helper.h"
#include "util/util.h"
#include "vtx.h"

#define QUIC_HEADER_LEN 4

#define quic_errorf(cmd, args...) send_quic_strf(cmd, QUIC_FLAG_ERROR, args)

usb_motor_test_t usb_motor_test = {
    .active = 0,
    .value = {0, 0, 0, 0},
};

extern profile_t profile;
extern profile_t default_profile;

extern pid_rate_preset_t pid_rate_presets[];
extern uint32_t pid_rate_presets_count;

extern vtx_settings_t vtx_settings;

extern uint8_t blackbox_override;
extern uint32_t blackbox_rate;

extern uint8_t encode_buffer[USB_BUFFER_SIZE];
extern uint8_t decode_buffer[USB_BUFFER_SIZE];

cbor_result_t cbor_encode_usb_motor_test_t(cbor_value_t *enc, const usb_motor_test_t *b) {
  CBOR_CHECK_ERROR(cbor_result_t res = cbor_encode_map_indefinite(enc));

  CBOR_CHECK_ERROR(res = cbor_encode_str(enc, "active"));
  CBOR_CHECK_ERROR(res = cbor_encode_uint8(enc, &b->active));

  CBOR_CHECK_ERROR(res = cbor_encode_str(enc, "value"));
  CBOR_CHECK_ERROR(res = cbor_encode_float_array(enc, b->value, 4));

  CBOR_CHECK_ERROR(res = cbor_encode_end_indefinite(enc));

  return res;
}

void send_quic_header(quic_command cmd, quic_flag flag, int16_t len) {
  static uint8_t frame[QUIC_HEADER_LEN];

  frame[0] = USB_MAGIC_QUIC;
  frame[1] = (cmd & (0xff >> 3)) | (flag & (0xff >> 5)) << 5;
  frame[2] = (len >> 8) & 0xFF;
  frame[3] = len & 0xFF;

  usb_serial_write(frame, QUIC_HEADER_LEN);
}

void send_quic(quic_command cmd, quic_flag flag, uint8_t *data, uint16_t len) {
  send_quic_header(cmd, flag, len);
  usb_serial_write(data, len);
}

cbor_result_t send_quic_str(quic_command cmd, quic_flag flag, const char *str) {
  const uint32_t size = strlen(str) + 128;
  uint8_t buffer[size];

  cbor_value_t enc;
  cbor_encoder_init(&enc, buffer, size);

  cbor_result_t res = cbor_encode_str(&enc, str);
  if (res < CBOR_OK) {
    return res;
  }
  send_quic(cmd, flag, buffer, cbor_encoder_len(&enc));
  return res;
}

cbor_result_t send_quic_strf(quic_command cmd, quic_flag flag, const char *fmt, ...) {
  const uint32_t size = strlen(fmt) + 128;
  char str[size];

  memset(str, 0, size);

  va_list args;
  va_start(args, fmt);
  vsnprintf(str, size, fmt, args);
  va_end(args);
  return send_quic_str(cmd, flag, str);
}

#define check_cbor_error(cmd)               \
  if (res < CBOR_OK) {                      \
    quic_errorf(cmd, "CBOR ERROR %d", res); \
    return;                                 \
  }

void get_quic(uint8_t *data, uint32_t len) {
  cbor_result_t res = CBOR_OK;

  cbor_value_t dec;
  cbor_decoder_init(&dec, data, len);

  cbor_value_t enc;
  cbor_encoder_init(&enc, encode_buffer, USB_BUFFER_SIZE);

  quic_values value = QUIC_CMD_INVALID;
  res = cbor_decode_uint8(&dec, &value);
  check_cbor_error(QUIC_CMD_GET);

  res = cbor_encode_uint8(&enc, &value);
  check_cbor_error(QUIC_CMD_GET);

  switch (value) {
  case QUIC_VAL_PROFILE: {
    res = cbor_encode_profile_t(&enc, &profile);
    check_cbor_error(QUIC_CMD_GET);

    send_quic(QUIC_CMD_GET, QUIC_FLAG_NONE, encode_buffer, cbor_encoder_len(&enc));
    break;
  }
  case QUIC_VAL_DEFAULT_PROFILE: {
    res = cbor_encode_profile_t(&enc, &default_profile);
    check_cbor_error(QUIC_CMD_GET);

    send_quic(QUIC_CMD_GET, QUIC_FLAG_NONE, encode_buffer, cbor_encoder_len(&enc));
    break;
  }
  case QUIC_VAL_INFO:
    res = cbor_encode_target_info_t(&enc, &target_info);
    check_cbor_error(QUIC_CMD_GET);

    send_quic(QUIC_CMD_GET, QUIC_FLAG_NONE, encode_buffer, cbor_encoder_len(&enc));
    break;
  case QUIC_VAL_STATE:
    res = cbor_encode_control_state_t(&enc, &state);
    check_cbor_error(QUIC_CMD_GET);

    send_quic(QUIC_CMD_GET, QUIC_FLAG_NONE, encode_buffer, cbor_encoder_len(&enc));
    break;
  case QUIC_VAL_PID_RATE_PRESETS:
    res = cbor_encode_array(&enc, pid_rate_presets_count);
    check_cbor_error(QUIC_CMD_GET);
    for (uint32_t i = 0; i < pid_rate_presets_count; i++) {
      res = cbor_encode_pid_rate_preset_t(&enc, &pid_rate_presets[i]);
      check_cbor_error(QUIC_CMD_GET);
    }

    send_quic(QUIC_CMD_GET, QUIC_FLAG_NONE, encode_buffer, cbor_encoder_len(&enc));
    break;
  case QUIC_VAL_VTX_SETTINGS:
    res = cbor_encode_vtx_settings_t(&enc, &vtx_settings);
    check_cbor_error(QUIC_CMD_GET);

    send_quic(QUIC_CMD_GET, QUIC_FLAG_NONE, encode_buffer, cbor_encoder_len(&enc));
    break;
#ifdef ENABLE_OSD
  case QUIC_VAL_OSD_FONT: {
    send_quic(QUIC_CMD_GET, QUIC_FLAG_STREAMING, encode_buffer, cbor_encoder_len(&enc));

    uint8_t font[54];

    for (uint16_t i = 0; i < 256; i++) {
      osd_read_character(i, font, 54);

      cbor_encoder_init(&enc, encode_buffer, USB_BUFFER_SIZE);
      res = cbor_encode_bstr(&enc, font, 54);
      check_cbor_error(QUIC_CMD_GET);

      send_quic(QUIC_CMD_GET, QUIC_FLAG_STREAMING, encode_buffer, cbor_encoder_len(&enc));
    }

    send_quic_header(QUIC_CMD_GET, QUIC_FLAG_STREAMING, 0);
    break;
  }
#endif
#ifdef USE_SERIAL_4WAY_BLHELI_INTERFACE
  case QUIC_VAL_BLHEL_SETTINGS: {
    send_quic(QUIC_CMD_GET, QUIC_FLAG_STREAMING, encode_buffer, cbor_encoder_len(&enc));

    const uint8_t count = serial_4way_init();
    time_delay_ms(500);

    for (uint8_t i = 0; i < count; i++) {
      blheli_settings_t settings;
      serial_esc4way_ack_t ack = serial_4way_read_settings(&settings, i);
      if (ack != ESC4WAY_ACK_OK) {
        continue;
      }

      cbor_encoder_init(&enc, encode_buffer, USB_BUFFER_SIZE);
      res = cbor_encode_blheli_settings_t(&enc, &settings);
      check_cbor_error(QUIC_CMD_GET);

      send_quic(QUIC_CMD_GET, QUIC_FLAG_STREAMING, encode_buffer, cbor_encoder_len(&enc));
    }

    serial_4way_release();

    send_quic_header(QUIC_CMD_GET, QUIC_FLAG_STREAMING, 0);
    break;
  }
#endif
  case QUIC_VAL_BIND_INFO: {
    res = cbor_encode_rx_bind_storage_t(&enc, &bind_storage);
    check_cbor_error(QUIC_CMD_GET);

    send_quic(QUIC_CMD_GET, QUIC_FLAG_NONE, encode_buffer, cbor_encoder_len(&enc));
    break;
  }
#ifdef DEBUG
  case QUIC_VAL_PERF_COUNTERS: {
    res = cbor_encode_perf_counters(&enc);
    check_cbor_error(QUIC_CMD_GET);
    send_quic(QUIC_CMD_GET, QUIC_FLAG_NONE, encode_buffer, cbor_encoder_len(&enc));
    break;
  }
#endif
  default:
    quic_errorf(QUIC_CMD_GET, "INVALID VALUE %d", value);
    break;
  }
}

void set_quic(uint8_t *data, uint32_t len) {
  cbor_result_t res = CBOR_OK;

  cbor_value_t dec;
  cbor_decoder_init(&dec, data, len);

  cbor_value_t enc;
  cbor_encoder_init(&enc, encode_buffer, USB_BUFFER_SIZE);

  quic_values value;
  res = cbor_decode_uint8(&dec, &value);
  check_cbor_error(QUIC_CMD_SET);

  res = cbor_encode_uint8(&enc, &value);
  check_cbor_error(QUIC_CMD_SET);

  switch (value) {
  case QUIC_VAL_PROFILE: {
    res = cbor_decode_profile_t(&dec, &profile);
    check_cbor_error(QUIC_CMD_SET);

    flash_save();

#ifdef ENABLE_OSD
    osd_clear();
    osd_display_reset();
#endif

    res = cbor_encode_profile_t(&enc, &profile);
    check_cbor_error(QUIC_CMD_SET);

    send_quic(QUIC_CMD_SET, QUIC_FLAG_NONE, encode_buffer, cbor_encoder_len(&enc));
    break;
  }
  case QUIC_VAL_VTX_SETTINGS: {
    vtx_settings_t settings;

    res = cbor_decode_vtx_settings_t(&dec, &settings);
    check_cbor_error(QUIC_CMD_SET);

    vtx_set(&settings);
    flash_save();

    res = cbor_encode_vtx_settings_t(&enc, &vtx_settings);
    check_cbor_error(QUIC_CMD_SET);

    send_quic(QUIC_CMD_SET, QUIC_FLAG_NONE, encode_buffer, cbor_encoder_len(&enc));
    break;
  }
#ifdef ENABLE_OSD
  case QUIC_VAL_OSD_FONT: {
    uint32_t len = 54;

    for (uint16_t i = 0; i < 256; i++) {
      const uint8_t *ptr = NULL;

      res = cbor_decode_bstr(&dec, &ptr, &len);
      check_cbor_error(QUIC_CMD_SET);

      osd_write_character(i, ptr, 54);
    }

    res = cbor_encode_str(&enc, "OK");
    check_cbor_error(QUIC_CMD_SET);

    send_quic(QUIC_CMD_SET, QUIC_FLAG_NONE, encode_buffer, cbor_encoder_len(&enc));
    break;
  }
#endif
#ifdef USE_SERIAL_4WAY_BLHELI_INTERFACE
  case QUIC_VAL_BLHEL_SETTINGS: {
    uint8_t count = serial_4way_init();
    time_delay_ms(500);

    for (uint8_t i = 0; i < count; i++) {
      blheli_settings_t settings;

      res = cbor_decode_blheli_settings_t(&dec, &settings);
      check_cbor_error(QUIC_CMD_SET);

      serial_esc4way_ack_t ack = serial_4way_write_settings(&settings, i);
      if (ack != ESC4WAY_ACK_OK) {
        break;
      }
    }

    serial_4way_release();

    res = cbor_encode_str(&enc, "OK");
    check_cbor_error(QUIC_CMD_SET);

    send_quic(QUIC_CMD_SET, QUIC_FLAG_NONE, encode_buffer, cbor_encoder_len(&enc));
    break;
  }
#endif
  case QUIC_VAL_BIND_INFO: {
    res = cbor_decode_rx_bind_storage_t(&dec, &bind_storage);
    check_cbor_error(QUIC_CMD_SET);

    flash_save();

    res = cbor_encode_rx_bind_storage_t(&enc, &bind_storage);
    check_cbor_error(QUIC_CMD_SET);

    send_quic(QUIC_CMD_SET, QUIC_FLAG_NONE, encode_buffer, cbor_encoder_len(&enc));
    break;
  }
  default:
    quic_errorf(QUIC_CMD_SET, "INVALID VALUE %d", value);
    break;
  }
}

#ifdef ENABLE_BLACKBOX
void process_blackbox(uint8_t *data, uint32_t len) {
  cbor_result_t res = CBOR_OK;

  cbor_value_t dec;
  cbor_decoder_init(&dec, data, len);

  cbor_value_t enc;
  cbor_encoder_init(&enc, encode_buffer, USB_BUFFER_SIZE);

  quic_blackbox_command cmd;
  res = cbor_decode_uint8(&dec, &cmd);
  check_cbor_error(QUIC_CMD_BLACKBOX);

  extern data_flash_header_t data_flash_header;
  extern data_flash_bounds_t bounds;

  switch (cmd) {
  case QUIC_BLACKBOX_RESET:
    data_flash_reset();
    send_quic(QUIC_CMD_BLACKBOX, QUIC_FLAG_NONE, NULL, 0);
    break;
  case QUIC_BLACKBOX_LIST:
    res = cbor_encode_map_indefinite(&enc);
    check_cbor_error(QUIC_CMD_BLACKBOX);

    res = cbor_encode_str(&enc, "flash_size");
    check_cbor_error(QUIC_CMD_BLACKBOX);
    const uint32_t size_in_kb = bounds.total_size / 1024;
    res = cbor_encode_uint32(&enc, &size_in_kb);
    check_cbor_error(QUIC_CMD_BLACKBOX);

    res = cbor_encode_str(&enc, "file_num");
    check_cbor_error(QUIC_CMD_BLACKBOX);
    res = cbor_encode_uint16(&enc, &data_flash_header.file_num);
    check_cbor_error(QUIC_CMD_BLACKBOX);

    res = cbor_encode_str(&enc, "files");
    check_cbor_error(QUIC_CMD_BLACKBOX);
    res = cbor_encode_array(&enc, data_flash_header.file_num);
    check_cbor_error(QUIC_CMD_BLACKBOX);

    for (uint8_t i = 0; i < data_flash_header.file_num; i++) {
      res = cbor_encode_uint32(&enc, &data_flash_header.files[i].entries);
      check_cbor_error(QUIC_CMD_BLACKBOX);
    }

    res = cbor_encode_end_indefinite(&enc);
    check_cbor_error(QUIC_CMD_BLACKBOX);

    send_quic(QUIC_CMD_BLACKBOX, QUIC_FLAG_NONE, encode_buffer, cbor_encoder_len(&enc));
    break;
  case QUIC_BLACKBOX_GET: {
    extern data_flash_header_t data_flash_header;

    uint8_t file_index;
    res = cbor_decode_uint8(&dec, &file_index);
    check_cbor_error(QUIC_CMD_BLACKBOX);

    send_quic(QUIC_CMD_BLACKBOX, QUIC_FLAG_STREAMING, encode_buffer, cbor_encoder_len(&enc));
    cbor_encoder_init(&enc, encode_buffer, USB_BUFFER_SIZE);

    if (data_flash_header.file_num > file_index) {
      blackbox_t blackbox[8];
      const data_flash_file_t *file = &data_flash_header.files[file_index];
      for (uint32_t i = 0; i < file->entries; i += 8) {
        res = data_flash_read_backbox(file_index, i, blackbox, 8);
        if (res < CBOR_OK) {
          continue;
        }
        for (uint32_t j = 0; j < 8; j++) {
          const uint32_t len = cbor_encoder_len(&enc);
          res = cbor_encode_blackbox_t(&enc, &blackbox[j]);
          if (res == CBOR_ERR_EOF) {
            send_quic(QUIC_CMD_BLACKBOX, QUIC_FLAG_STREAMING, encode_buffer, len);
            cbor_encoder_init(&enc, encode_buffer, USB_BUFFER_SIZE);

            res = cbor_encode_blackbox_t(&enc, &blackbox[j]);
          }
          check_cbor_error(QUIC_CMD_BLACKBOX);
        }
      }
    }

    send_quic_header(QUIC_CMD_BLACKBOX, QUIC_FLAG_STREAMING, 0);

    break;
  }
  default:
    quic_errorf(QUIC_CMD_BLACKBOX, "INVALID CMD %d", cmd);
    break;
  }
}
#endif

void process_motor_test(uint8_t *data, uint32_t len) {
  cbor_result_t res = CBOR_OK;

  cbor_value_t dec;
  cbor_decoder_init(&dec, data, len);

  cbor_value_t enc;
  cbor_encoder_init(&enc, encode_buffer, USB_BUFFER_SIZE);

  quic_motor_command cmd;
  res = cbor_decode_uint8(&dec, &cmd);
  check_cbor_error(QUIC_CMD_MOTOR);

  switch (cmd) {
  case QUIC_MOTOR_TEST_STATUS:
    res = cbor_encode_usb_motor_test_t(&enc, &usb_motor_test);
    check_cbor_error(QUIC_CMD_MOTOR);

    send_quic(QUIC_CMD_MOTOR, QUIC_FLAG_NONE, encode_buffer, cbor_encoder_len(&enc));
    break;

  case QUIC_MOTOR_TEST_ENABLE:
    usb_motor_test.active = 1;

    res = cbor_encode_uint8(&enc, &usb_motor_test.active);
    check_cbor_error(QUIC_CMD_MOTOR);

    send_quic(QUIC_CMD_MOTOR, QUIC_FLAG_NONE, encode_buffer, cbor_encoder_len(&enc));
    break;

  case QUIC_MOTOR_TEST_DISABLE:
    usb_motor_test.active = 0;

    res = cbor_encode_uint8(&enc, &usb_motor_test.active);
    check_cbor_error(QUIC_CMD_MOTOR);

    send_quic(QUIC_CMD_MOTOR, QUIC_FLAG_NONE, encode_buffer, cbor_encoder_len(&enc));
    break;

  case QUIC_MOTOR_TEST_SET_VALUE:
    res = cbor_decode_float_array(&dec, usb_motor_test.value, 4);
    check_cbor_error(QUIC_CMD_MOTOR);

    res = cbor_encode_float_array(&enc, usb_motor_test.value, 4);
    check_cbor_error(QUIC_CMD_MOTOR);

    send_quic(QUIC_CMD_MOTOR, QUIC_FLAG_NONE, encode_buffer, cbor_encoder_len(&enc));
    break;
#ifdef USE_SERIAL_4WAY_BLHELI_INTERFACE
  case QUIC_MOTOR_ESC4WAY_IF: {
    uint8_t count = serial_4way_init();

    res = cbor_encode_uint8(&enc, &count);
    check_cbor_error(QUIC_CMD_MOTOR);

    send_quic(QUIC_CMD_MOTOR, QUIC_FLAG_EXIT, encode_buffer, cbor_encoder_len(&enc));

    serial_4way_process();
    break;
  }
#endif
  default:
    quic_errorf(QUIC_CMD_MOTOR, "INVALID CMD %d", cmd);
    break;
  }
}

void process_serial(uint8_t *data, uint32_t len) {
  cbor_result_t res = CBOR_OK;

  cbor_value_t dec;
  cbor_decoder_init(&dec, data, len);

  cbor_value_t enc;
  cbor_encoder_init(&enc, encode_buffer, USB_BUFFER_SIZE);

  quic_motor_command cmd;
  res = cbor_decode_uint8(&dec, &cmd);
  check_cbor_error(QUIC_CMD_SERIAL);

  switch (cmd) {
  case QUIC_SERIAL_ENABLE: {
    usart_ports_t port = USART_PORT_INVALID;
    res = cbor_decode_uint8(&dec, &port);
    check_cbor_error(QUIC_CMD_SERIAL);

    uint32_t baudrate = 0;
    res = cbor_decode_uint32(&dec, &baudrate);
    check_cbor_error(QUIC_CMD_SERIAL);

    res = cbor_encode_uint8(&enc, &port);
    check_cbor_error(QUIC_CMD_SERIAL);

    send_quic(QUIC_CMD_SERIAL, QUIC_FLAG_NONE, encode_buffer, cbor_encoder_len(&enc));

    serial_enable_rcc(port);
    serial_init(port, baudrate, false);

    while (1) {
      uint8_t data = 0;

      while (usb_serial_read(&data, 1)) {
        serial_write_bytes(port, &data, 1);
      }

      while (serial_read_bytes(port, &data, 1)) {
        usb_serial_write(&data, 1);
      }
    }

    break;
  }

  default:
    quic_errorf(QUIC_CMD_SERIAL, "INVALID CMD %d", cmd);
    break;
  }
}

void usb_process_quic() {
  const uint8_t cmd = usb_serial_read_byte();
  if (cmd == QUIC_CMD_INVALID) {
    quic_errorf(QUIC_CMD_INVALID, "INVALID CMD %d", cmd);
    return;
  }

  const uint16_t size = (uint16_t)usb_serial_read_byte() << 8 | usb_serial_read_byte();
  uint8_t buffer[size];

  if (size != 0) {
    uint32_t len = usb_serial_read(buffer, size);
    for (uint32_t timeout = 0x2000; len < size && timeout > 0;) {
      uint32_t n = usb_serial_read(buffer + len, size - len);
      if (n == 0) {
        timeout--;
        __WFI();
      } else {
        len += n;
      }
    }
    if (len != size) {
      quic_errorf(cmd, "INVALID SIZE %d vs %d", len, size);
      return;
    }
  }

  switch (cmd) {
  case QUIC_CMD_GET:
    get_quic(buffer, size);
    break;
  case QUIC_CMD_SET:
    set_quic(buffer, size);
    break;
  case QUIC_CMD_CAL_IMU:
    sixaxis_gyro_cal(); // for flashing lights
    sixaxis_acc_cal();

    flash_save();
    flash_load();

    send_quic(QUIC_CMD_CAL_IMU, QUIC_FLAG_NONE, NULL, 0);
    break;
#ifdef ENABLE_BLACKBOX
  case QUIC_CMD_BLACKBOX:
    process_blackbox(buffer, size);
    break;
#endif
  case QUIC_CMD_MOTOR:
    process_motor_test(buffer, size);
    break;
  case QUIC_CMD_CAL_STICKS:
    request_stick_calibration_wizard();
    send_quic(QUIC_CMD_CAL_STICKS, QUIC_FLAG_NONE, NULL, 0);
    break;
  case QUIC_CMD_SERIAL:
    process_serial(buffer, size);
    break;
  default:
    quic_errorf(QUIC_CMD_INVALID, "INVALID CMD %d", cmd);
    break;
  }
}