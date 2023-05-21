#pragma once

#include <cbor.h>
#include <stdint.h>

#include "io/blackbox.h"

#define QUIC_MAGIC '#'
#define QUIC_HEADER_LEN 4

#define QUIC_PROTOCOL_VERSION MAKE_SEMVER(0, 2, 0)

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
} quic_command;

typedef enum {
  QUIC_BLACKBOX_RESET,
  QUIC_BLACKBOX_LIST,
  QUIC_BLACKBOX_GET
} quic_blackbox_command;

typedef enum {
  QUIC_MOTOR_TEST_STATUS,
  QUIC_MOTOR_TEST_ENABLE,
  QUIC_MOTOR_TEST_DISABLE,
  QUIC_MOTOR_TEST_SET_VALUE,
  QUIC_MOTOR_ESC4WAY_IF
} quic_motor_command;

typedef enum {
  QUIC_SERIAL_ENABLE,
} quic_serial_command;

typedef enum {
  QUIC_OSD_READ_CHAR,
  QUIC_OSD_WRITE_CHAR,
} quic_osd_command;

typedef enum {
  QUIC_FLAG_NONE,
  QUIC_FLAG_ERROR,
  QUIC_FLAG_STREAMING,
  QUIC_FLAG_EXIT,
} quic_flag;

typedef enum {
  QUIC_VAL_INVALID,
  QUIC_VAL_INFO,
  QUIC_VAL_PROFILE,
  QUIC_VAL_DEFAULT_PROFILE,
  QUIC_VAL_STATE,
  QUIC_VAL_PID_RATE_PRESETS,
  QUIC_VAL_VTX_SETTINGS,
  QUIC_VAL_OSD_FONT, // deprecated
  QUIC_VAL_BLHEL_SETTINGS,
  QUIC_VAL_BIND_INFO,
  QUIC_VAL_PERF_COUNTERS,
  QUIC_VAL_BLACKBOX_PRESETS,
  QUIC_VAL_TARGET,
} quic_values;

typedef void (*quic_send_fn_t)(uint8_t *data, uint32_t len, void *priv);

typedef struct {
  void *priv_data;
  quic_send_fn_t send;
} quic_t;

cbor_result_t quic_send_str(quic_t *quic, quic_command cmd, quic_flag flag, const char *str);

bool quic_process(quic_t *quic, uint8_t *data, uint32_t size);