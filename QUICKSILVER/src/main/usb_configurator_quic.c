#include "usb_configurator.h"

#include "drv_time.h"
#include "drv_usb.h"
#include "profile.h"
#include "project.h"

#if defined(F405)
#define QUIC_HEADER_LEN 4

extern profile_t profile;
extern float rx[4];
extern float rxcopy[4];

extern uint8_t encode_buffer[1024];
extern uint8_t decode_buffer[1024];

typedef enum {
  QUIC_CMD_INVALID,
  QUIC_CMD_GET,
  QUIC_CMD_SET,
} quic_command;

typedef enum {
  QUIC_VAL_INVALID,
  QUIC_VAL_PROFILE,
  QUIC_VAL_RX,
} quic_values;

void send_quic(quic_command cmd, uint8_t *data, uint16_t len) {
  const uint16_t size = len + QUIC_HEADER_LEN;

  uint8_t frame[size];
  frame[0] = USB_MAGIC_QUIC;
  frame[1] = cmd;
  frame[2] = (len >> 8) & 0xFF;
  frame[3] = len & 0xFF;

  for (uint8_t i = 0; i < len; i++) {
    frame[i + QUIC_HEADER_LEN] = data[i];
  }

  usb_serial_write(frame, size);
}

void get_quic(quic_values value) {
  nanocbor_encoder_t enc;
  nanocbor_encoder_init(&enc, encode_buffer, 1024);

  switch (value) {
  case QUIC_VAL_PROFILE:
    nanocbor_fmt_profile_t(&enc, profile);
    send_quic(QUIC_CMD_GET, encode_buffer, nanocbor_encoded_len(&enc));
    break;
  case QUIC_VAL_RX:
    nanocbor_fmt_map(&enc, 2);
    nanocbor_put_tstr(&enc, "raw");
    nanocbor_fmt_array(&enc, 4);
    nanocbor_fmt_float(&enc, rx[0]);
    nanocbor_fmt_float(&enc, rx[1]);
    nanocbor_fmt_float(&enc, rx[2]);
    nanocbor_fmt_float(&enc, rx[3]);
    nanocbor_put_tstr(&enc, "copy");
    nanocbor_fmt_array(&enc, 4);
    nanocbor_fmt_float(&enc, rxcopy[0]);
    nanocbor_fmt_float(&enc, rxcopy[1]);
    nanocbor_fmt_float(&enc, rxcopy[2]);
    nanocbor_fmt_float(&enc, rxcopy[3]);
    send_quic(QUIC_CMD_GET, encode_buffer, nanocbor_encoded_len(&enc));
    break;
  default:
    break;
  }
}

void set_quic(quic_values value, uint8_t *data, uint32_t len) {
  nanocbor_value_t val;
  nanocbor_decoder_init(&val, data, len);

  nanocbor_encoder_t enc;
  nanocbor_encoder_init(&enc, encode_buffer, 1024);

  switch (value) {
  case QUIC_VAL_PROFILE: {
    nanocbor_get_profile_t(&val, &profile);
    nanocbor_fmt_profile_t(&enc, profile);
    send_quic(QUIC_CMD_SET, encode_buffer, nanocbor_encoded_len(&enc));
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