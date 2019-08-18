#include "usb_configurator.h"

#include "nanocbor/nanocbor.h"
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

void nanocbor_fmt_vector(nanocbor_encoder_t *enc, vector_t *vec) {
  nanocbor_fmt_array(enc, 3);
  nanocbor_fmt_float(enc, vec->axis[0]);
  nanocbor_fmt_float(enc, vec->axis[1]);
  nanocbor_fmt_float(enc, vec->axis[2]);
}

void get_quic(quic_values value) {
  uint8_t buf[1024];

  nanocbor_encoder_t enc;
  nanocbor_encoder_init(&enc, buf, 1024);

  switch (value) {
  case QUIC_VAL_PROFILE:
    nanocbor_fmt_map_indefinite(&enc);

    nanocbor_put_tstr(&enc, "rate_mode");
    nanocbor_fmt_uint(&enc, profile.rate_mode);

    nanocbor_put_tstr(&enc, "silverware_rate");
    {
      nanocbor_fmt_map(&enc, 3);
      nanocbor_put_tstr(&enc, "max_rate");
      nanocbor_fmt_vector(&enc, &profile.silverware_rate.max_rate);
      nanocbor_put_tstr(&enc, "angle_expo");
      nanocbor_fmt_vector(&enc, &profile.silverware_rate.angle_expo);
      nanocbor_put_tstr(&enc, "acro_expo");
      nanocbor_fmt_vector(&enc, &profile.silverware_rate.acro_expo);
    }

    nanocbor_put_tstr(&enc, "betaflight_rate");
    {
      nanocbor_fmt_map(&enc, 3);
      nanocbor_put_tstr(&enc, "rc_rate");
      nanocbor_fmt_vector(&enc, &profile.betaflight_rate.rc_rate);
      nanocbor_put_tstr(&enc, "super_rate");
      nanocbor_fmt_vector(&enc, &profile.betaflight_rate.super_rate);
      nanocbor_put_tstr(&enc, "expo");
      nanocbor_fmt_vector(&enc, &profile.betaflight_rate.expo);
    }

    nanocbor_put_tstr(&enc, "low_rate_mulitplier");
    nanocbor_fmt_float(&enc, profile.low_rate_mulitplier);

    nanocbor_fmt_end_indefinite(&enc);
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
  }
}

uint8_t usb_process_quic(uint8_t *data, uint32_t len) {
  // already checked, just for reference
  if (data[0] != USB_MAGIC_QUIC) {
    return;
  }

  const uint8_t cmd = data[1];
  const uint8_t size = data[2];

  switch (cmd) {
  case QUIC_CMD_GET:
    get_quic(data[3]);
    break;
  }
}
#endif