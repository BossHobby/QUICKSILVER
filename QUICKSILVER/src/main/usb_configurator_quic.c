#include "usb_configurator.h"

#include "drv_usb.h"
#include "profile.h"
#include "project.h"

#if defined(F405)
#define QUIC_HEADER_LEN 3

extern profile_t profile;
extern float rx[4];
extern float rxcopy[4];

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

void send_quic(quic_command cmd, uint8_t *data, uint8_t len) {
  const uint8_t size = len + QUIC_HEADER_LEN;

  uint8_t frame[size];
  frame[0] = USB_MAGIC_QUIC;
  frame[1] = cmd;
  frame[2] = len;

  for (uint8_t i = 0; i < len; i++) {
    frame[i + QUIC_HEADER_LEN] = data[i];
  }

  usb_serial_write(frame, size);
}

void get_quic(quic_values value) {
  uint8_t buf[1024];

  nanocbor_encoder_t enc;
  nanocbor_encoder_init(&enc, buf, 1024);

  switch (value) {
  case QUIC_VAL_PROFILE:
    nanocbor_fmt_profile_t(&enc, profile);
    send_quic(QUIC_CMD_GET, buf, nanocbor_encoded_len(&enc));
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
    send_quic(QUIC_CMD_GET, buf, nanocbor_encoded_len(&enc));
    break;
  default:
    break;
  }
}

void set_quic(quic_values value, uint8_t *data, uint32_t len) {
  uint8_t buf[1024];

  nanocbor_value_t val;
  nanocbor_decoder_init(&val, data, len);

  nanocbor_encoder_t enc;
  nanocbor_encoder_init(&enc, buf, 1024);

  switch (value) {
  case QUIC_VAL_PROFILE: {
    profile_t p;
    nanocbor_get_profile_t(&val, &p);
    nanocbor_fmt_profile_t(&enc, p);
    send_quic(QUIC_CMD_SET, buf, nanocbor_encoded_len(&enc));
    break;
  }
  default:
    break;
  }
}

void usb_process_quic() {
  const uint8_t cmd = usb_serial_read_byte();
  const uint8_t size = usb_serial_read_byte();

  uint8_t data[size];
  uint32_t len = usb_serial_read(data, size);
  for (uint32_t timeout = 1000; len < size && timeout; --timeout) {
    len += usb_serial_read(data + len, size - len);
    delay(10);
    __WFI();
  }
  if (len < size) {
    usb_serial_printf("ERROR invalid size (%d vs %d)\r\n", len, size);
    return;
  }

  switch (cmd) {
  case QUIC_CMD_GET:
    get_quic(data[0]);
    break;
  case QUIC_CMD_SET:
    set_quic(data[0], data + 1, size - 1);
    break;
  }
}
#endif