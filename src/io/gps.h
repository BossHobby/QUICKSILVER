#pragma once

#include <cbor.h>
#include <stdbool.h>
#include <stdint.h>

#define GPS_MIN_SATS_FOR_LOCK 4

typedef enum {
  GPS_FIX_NONE = 0,
  GPS_FIX_DR_ONLY = 1,
  GPS_FIX_2D = 2,
  GPS_FIX_3D = 3,
  GPS_FIX_GNSS_DR = 4,
  GPS_FIX_TIME_ONLY = 5,
} gps_fix_type_t;

typedef enum {
  GPS_DETECT_BAUD = 0,
  GPS_CHANGE_BAUD = 1,
  GPS_CHECK_PERFORMANCE = 2,
  GPS_CONFIG_CONSTELLATIONS = 3,
  GPS_CONFIG_GROUND_ASSIST = 4,
  GPS_CONFIG_RATE = 5,
  GPS_CONFIG_NAV5 = 6,
  GPS_CONFIG_SBAS = 7,
  GPS_WAITING_FOR_LOCK = 8,
  GPS_RUNNING = 9,
  GPS_NOT_DETECTED = 10,
} gps_config_state_t;

typedef enum {
  GPS_POWER_FULL = 0,
  GPS_POWER_BALANCED = 1,
  GPS_POWER_LOW = 2,
  GPS_POWER_UNKNOWN = 0xFF,
} gps_power_mode_t;

typedef struct {
  uint8_t enabled;
  uint8_t healthy;
  uint8_t sats_in_view;
  uint8_t sats_used;
} gps_constellation_t;

typedef struct {
  uint8_t state;
  uint32_t version;
  uint8_t fix_type;
  uint8_t fix_quality;
  uint8_t sats_in_view;
  uint8_t sats_used;
  gps_constellation_t gps;
  gps_constellation_t glonass;
  gps_constellation_t galileo;
  gps_constellation_t beidou;
  uint16_t pdop;
  uint32_t h_acc;
  uint32_t v_acc;
  uint8_t power_mode;
  uint8_t update_rate;
  uint8_t avg_cno;
} gps_status_t;

#define GPS_CONSTELLATION_MEMBERS   \
  START_STRUCT(gps_constellation_t) \
  MEMBER(enabled, uint8_t)          \
  MEMBER(healthy, uint8_t)          \
  MEMBER(sats_in_view, uint8_t)     \
  MEMBER(sats_used, uint8_t)        \
  END_STRUCT()

#define GPS_STATUS_MEMBERS             \
  START_STRUCT(gps_status_t)           \
  MEMBER(state, uint8_t)               \
  MEMBER(version, uint32_t)            \
  MEMBER(fix_type, uint8_t)            \
  MEMBER(fix_quality, uint8_t)         \
  MEMBER(sats_in_view, uint8_t)        \
  MEMBER(sats_used, uint8_t)           \
  MEMBER(gps, gps_constellation_t)     \
  MEMBER(glonass, gps_constellation_t) \
  MEMBER(galileo, gps_constellation_t) \
  MEMBER(beidou, gps_constellation_t)  \
  MEMBER(pdop, uint16_t)               \
  MEMBER(h_acc, uint32_t)              \
  MEMBER(v_acc, uint32_t)              \
  MEMBER(power_mode, uint8_t)          \
  MEMBER(update_rate, uint8_t)         \
  MEMBER(avg_cno, uint8_t)             \
  END_STRUCT()

extern gps_status_t gps_status;

void gps_init();
void gps_task();

cbor_result_t cbor_encode_gps_constellation_t(cbor_value_t *enc, const gps_constellation_t *o);
cbor_result_t cbor_encode_gps_status_t(cbor_value_t *enc, const gps_status_t *o);
