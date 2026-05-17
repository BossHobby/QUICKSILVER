#pragma once

#include <cbor.h>
#include <stdint.h>

#include "io/blackbox.h"

#define QUIC_MAGIC '#'
#define QUIC_HEADER_LEN 4

// QUIC protocol changelog (protocol compatibility):
// 0.2.10: blackbox: add GPS/home coordinates and navigation altitude (0.1 m), move Debug to field 16.
//         state: add RTH heading acquisition and heading-failed phases; reorder RTH phase values.
//         profile: add GPS constellation selection.
// 0.2.9 (30cde6a5e): profile: move bind and vtx settings into profile.
// 0.2.8 (fab52980): rx: add CRSF bind command.
// 0.2.7 (399b96c5): blackbox: harden logging and transfer handling.
// 0.2.4 (116782b9): add support for esc serial passthrough.
// 0.2.3 (073633ad): vtx: increase power level count, increase label length.
// 0.2.2 (f2a6eeee): adjust serial 4 way pass-through order to match with profile.
// 0.2.1 (33646963): quic: bump protocol.
// 0.2.0 (227c6435): quic field cleanup and compatibility-related tweaks.
// 0.1.3 (33e3666d): increment quic protocol version.
// 0.1.2 (9eb3a5a2): increment quic protocol version.
// 0.1.1 (e1e99633): extract quic protocol.
// 0.1.0: introduce QUIC protocol serialization.
#define QUIC_PROTOCOL_VERSION MAKE_SEMVER(0, 2, 10)

typedef enum {
  QUIC_CMD_INVALID,
  QUIC_CMD_GET,
  QUIC_CMD_SET,
  QUIC_CMD_LOG,
  QUIC_CMD_CAL_IMU,
  QUIC_CMD_BLACKBOX,
  QUIC_CMD_MOTOR,
  QUIC_CMD_CAL_STICKS,
  QUIC_CMD_SERIAL,
  QUIC_CMD_OSD,
  QUIC_CMD_RX,
} __attribute__((__packed__)) quic_command;

typedef enum {
  QUIC_BLACKBOX_RESET,
  QUIC_BLACKBOX_LIST,
  QUIC_BLACKBOX_GET
} __attribute__((__packed__)) quic_blackbox_command;

typedef enum {
  QUIC_MOTOR_TEST_STATUS,
  QUIC_MOTOR_TEST_ENABLE,
  QUIC_MOTOR_TEST_DISABLE,
  QUIC_MOTOR_TEST_SET_VALUE,
  QUIC_MOTOR_ESC4WAY_IF,
  QUIC_MOTOR_SERIAL,
  QUIC_MOTOR_SET_DIRECTION,
} __attribute__((__packed__)) quic_motor_command;

typedef enum {
  QUIC_SERIAL_ENABLE,
} __attribute__((__packed__)) quic_serial_command;

typedef enum {
  QUIC_OSD_READ_CHAR,
  QUIC_OSD_WRITE_CHAR,
} __attribute__((__packed__)) quic_osd_command;

typedef enum {
  QUIC_RX_BIND,
} __attribute__((__packed__)) quic_rx_command;

typedef enum {
  QUIC_FLAG_NONE,
  QUIC_FLAG_ERROR,
  QUIC_FLAG_STREAMING,
  QUIC_FLAG_EXIT,
} __attribute__((__packed__)) quic_flag;

typedef enum {
  QUIC_VAL_INVALID,
  QUIC_VAL_INFO,
  QUIC_VAL_PROFILE,
  QUIC_VAL_DEFAULT_PROFILE,
  QUIC_VAL_STATE,
  QUIC_VAL_PID_RATE_PRESETS,
  QUIC_VAL_VTX_SETTINGS, // deprecated
  QUIC_VAL_OSD_FONT, // deprecated
  QUIC_VAL_BLHEL_SETTINGS, // deprecated
  QUIC_VAL_BIND_INFO, // deprecated
  QUIC_VAL_PERF_COUNTERS,
  QUIC_VAL_BLACKBOX_PRESETS,
  QUIC_VAL_TARGET,
  QUIC_VAL_GPS_STATUS,
} __attribute__((__packed__)) quic_values;

typedef void (*quic_send_fn_t)(uint8_t *data, uint32_t len, void *priv);

typedef struct {
  void *priv_data;
  quic_send_fn_t send;
} quic_t;

cbor_result_t quic_send_str(quic_t *quic, quic_command cmd, quic_flag flag, const char *str);

bool quic_process(quic_t *quic, uint8_t *data, uint32_t size);
