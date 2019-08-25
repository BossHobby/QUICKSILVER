#include "usb_configurator.h"

#include <stdarg.h>
#include <stdio.h>
#include <string.h>

#include "drv_time.h"
#include "drv_usb.h"
#include "flash.h"
#include "profile.h"
#include "project.h"

#if defined(F405)
#define QUIC_HEADER_LEN 4

extern profile_t profile;
extern float rx[4];
extern float rxcopy[4];
extern float vbattfilt;
extern float vbatt_comp;

extern uint8_t encode_buffer[1024];
extern uint8_t decode_buffer[1024];

typedef enum {
  QUIC_CMD_INVALID,
  QUIC_CMD_GET,
  QUIC_CMD_SET,
  QUIC_CMD_LOG,
} quic_command;

typedef enum {
  QUIC_VAL_INVALID,
  QUIC_VAL_PROFILE,
  QUIC_VAL_RX,
  QUIC_VAL_VBAT,
} quic_values;

void send_quic(quic_command cmd, uint8_t *data, uint16_t len) {
  const uint16_t size = len + QUIC_HEADER_LEN;

  uint8_t frame[size];
  frame[0] = USB_MAGIC_QUIC;
  frame[1] = cmd;
  frame[2] = (len >> 8) & 0xFF;
  frame[3] = len & 0xFF;

  for (uint32_t i = 0; i < len; i++) {
    frame[i + QUIC_HEADER_LEN] = data[i];
  }

  usb_serial_write(frame, size);
}

cbor_result_t send_quic_log(const char *str) {
  const uint32_t size = strlen(str) + 128;
  uint8_t buffer[size];

  cbor_value_t enc;
  cbor_encoder_init(&enc, buffer, size);

  cbor_result_t res = cbor_encode_str(&enc, str);
  if (res < CBOR_OK) {
    return res;
  }
  send_quic(QUIC_CMD_LOG, buffer, cbor_encoder_len(&enc));
  return res;
}

cbor_result_t send_quic_logf(const char *fmt, ...) {
  const uint32_t size = strlen(fmt) + 128;
  char str[size];

  memset(str, 0, size);

  va_list args;
  va_start(args, fmt);
  vsnprintf(str, size, fmt, args);
  va_end(args);
  return send_quic_log(str);
}

void get_quic(quic_values value) {
  cbor_result_t res = CBOR_OK;
  cbor_value_t enc;
  cbor_encoder_init(&enc, encode_buffer, 1024);

  switch (value) {
  case QUIC_VAL_PROFILE: {
    res = cbor_encode_profile_t(&enc, profile);
    if (res < CBOR_OK) {
      send_quic_logf("CBOR ERROR %d", res);
      return;
    }
    send_quic(QUIC_CMD_GET, encode_buffer, cbor_encoder_len(&enc));
    break;
  }
  case QUIC_VAL_RX:
    cbor_encode_map(&enc, 2);

    cbor_encode_str(&enc, "raw");
    cbor_encode_array(&enc, 4);
    cbor_encode_float(&enc, rx[0]);
    cbor_encode_float(&enc, rx[1]);
    cbor_encode_float(&enc, rx[2]);
    cbor_encode_float(&enc, rx[3]);

    cbor_encode_str(&enc, "copy");
    cbor_encode_array(&enc, 4);
    cbor_encode_float(&enc, rxcopy[0]);
    cbor_encode_float(&enc, rxcopy[1]);
    cbor_encode_float(&enc, rxcopy[2]);
    cbor_encode_float(&enc, rxcopy[3]);
    send_quic(QUIC_CMD_GET, encode_buffer, cbor_encoder_len(&enc));
    break;
  case QUIC_VAL_VBAT:
    cbor_encode_map(&enc, 2);
    cbor_encode_str(&enc, "filter");
    cbor_encode_float(&enc, vbattfilt);
    cbor_encode_str(&enc, "compare");
    cbor_encode_float(&enc, vbatt_comp);

    send_quic(QUIC_CMD_GET, encode_buffer, cbor_encoder_len(&enc));
    break;
  default:
    break;
  }
}

void set_quic(quic_values value, uint8_t *data, uint32_t len) {
  cbor_result_t res = CBOR_OK;

  cbor_value_t dec;
  cbor_decoder_init(&dec, data, len);

  cbor_value_t enc;
  cbor_encoder_init(&enc, encode_buffer, 1024);

  switch (value) {
  case QUIC_VAL_PROFILE: {
    res = cbor_decode_profile_t(&dec, &profile);
    if (res < CBOR_OK) {
      send_quic_logf("CBOR ERROR %d", res);
      return;
    }

    flash_save();
    flash_load();

    res = cbor_encode_profile_t(&enc, profile);
    if (res < CBOR_OK) {
      send_quic_logf("CBOR ERROR %d", res);
      return;
    }
    send_quic(QUIC_CMD_SET, encode_buffer, cbor_encoder_len(&enc));
    break;
  }
  default:
    break;
  }
}

void usb_process_quic() {
  const uint8_t cmd = usb_serial_read_byte();
  if (cmd == QUIC_CMD_INVALID) {
    usb_serial_printf("ERROR invalid cmd (%d)\r\n", cmd);
    return;
  }

  const uint16_t size = (uint16_t)usb_serial_read_byte() << 8 | usb_serial_read_byte();
  if (size == 0) {
    usb_serial_printf("ERROR invalid size (%d)\r\n", size);
    return;
  }

  uint32_t len = usb_serial_read(decode_buffer, size);
  for (uint32_t timeout = 1000; len < size && timeout > 0; --timeout) {
    len += usb_serial_read(decode_buffer + len, size - len);
    delay(10);
  }
  if (len != size) {
    usb_serial_printf("ERROR invalid size (%d vs %d)\r\n", len, size);
    return;
  }

  switch (cmd) {
  case QUIC_CMD_GET:
    get_quic(decode_buffer[0]);
    break;
  case QUIC_CMD_SET:
    set_quic(decode_buffer[0], decode_buffer + 1, size - 1);
    break;
  }

  extern unsigned int lastlooptime;
  lastlooptime = gettime();
}
#endif