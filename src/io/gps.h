#pragma once

#include <cbor.h>
#include <stdbool.h>
#include <stdint.h>

#include <FreeRTOS.h>

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
  GPS_CONFIG_UPDATE_RATE = 4,
  GPS_CONFIG_NAV_MODEL = 5,
  GPS_CONFIG_SBAS = 6,
  GPS_CONFIG_MESSAGES = 7,
  GPS_WAITING_FOR_LOCK = 8,
  GPS_CONFIG_FLIGHT_MODEL = 9,
  GPS_RUNNING = 10,
  GPS_CONFIG_DISABLE_NAV_SAT = 11,
  GPS_RUNNING_NAV_SAT_OFF = 12,
  GPS_NOT_DETECTED = 13,
} gps_config_state_t;

// Receiver interference reports (MON-RF or SEC-SIG jammingState/spoofingState).
typedef enum {
  GPS_JAMMING_UNKNOWN = 0, // not reported, detection disabled or stale
  GPS_JAMMING_OK = 1,
  GPS_JAMMING_WARNING = 2,  // interference visible, fix still OK
  GPS_JAMMING_CRITICAL = 3, // interference visible, no fix
} gps_jamming_t;

typedef enum {
  GPS_SPOOFING_UNKNOWN = 0, // not reported, detection disabled or stale
  GPS_SPOOFING_OK = 1,
  GPS_SPOOFING_INDICATED = 2, // indicated by a single source
  GPS_SPOOFING_MULTIPLE = 3,  // indicated by multiple sources
} gps_spoofing_t;

// Receiver solution and interference status in one byte, written only by the
// GPS driver in IO.
typedef struct {
  uint8_t lock : 1;     // valid 3D fix with enough satellites; cleared after 500 ms without NAV-PVT
  uint8_t jamming : 2;  // gps_jamming_t from the receiver (M9/M10); unknown when stale
  uint8_t spoofing : 2; // gps_spoofing_t from M10 SEC-SIG; unknown when stale
} gps_health_t;

// Encoded as a {lock, jamming, spoofing} map; bitfields have no address for
// the member encoders.
cbor_result_t cbor_encode_gps_health_t(cbor_value_t *enc, const gps_health_t *health);

// MON-RF antenna supervisor state.
typedef enum {
  GPS_ANTENNA_INIT = 0,
  GPS_ANTENNA_UNKNOWN = 1,
  GPS_ANTENNA_OK = 2,
  GPS_ANTENNA_SHORT = 3,
  GPS_ANTENNA_OPEN = 4,
} gps_antenna_status_t;

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
  int32_t height;
  int32_t vel_n;
  int32_t vel_e;
  int32_t vel_d;
  uint32_t h_acc;
  uint32_t v_acc;
  uint32_t speed_acc;
  uint32_t head_acc;
  uint8_t power_mode;
  uint8_t update_rate; // Hz, measured from delivered solutions; 0 until two arrive
  uint8_t avg_cno;
  // MON-RF diagnostics (M9/M10), refreshed at 1 Hz.
  uint8_t antenna_status; // gps_antenna_status_t
  uint16_t noise_per_ms;  // RF noise level
  uint16_t agc_count;     // AGC monitor, 0-8191 of full gain
  uint8_t cw_jamming;     // continuous-wave interference, 0 (none) to 255 (strong)
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
  MEMBER(height, int32_t)              \
  MEMBER(vel_n, int32_t)               \
  MEMBER(vel_e, int32_t)               \
  MEMBER(vel_d, int32_t)               \
  MEMBER(h_acc, uint32_t)              \
  MEMBER(v_acc, uint32_t)              \
  MEMBER(speed_acc, uint32_t)          \
  MEMBER(head_acc, uint32_t)           \
  MEMBER(power_mode, uint8_t)          \
  MEMBER(update_rate, uint8_t)         \
  MEMBER(avg_cno, uint8_t)             \
  MEMBER(antenna_status, uint8_t)      \
  MEMBER(noise_per_ms, uint16_t)       \
  MEMBER(agc_count, uint16_t)          \
  MEMBER(cw_jamming, uint8_t)          \
  END_STRUCT()

extern gps_status_t gps_status;

void gps_init();
// Event-driven on serial RX data; reports a periodic backstop in ticks that
// covers config retries and missed notifications, portMAX_DELAY when no GPS
// port is configured.
TickType_t gps_task();

cbor_result_t cbor_encode_gps_constellation_t(cbor_value_t *enc, const gps_constellation_t *o);
cbor_result_t cbor_encode_gps_status_t(cbor_value_t *enc, const gps_status_t *o);
