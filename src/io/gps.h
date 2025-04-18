#pragma once

#include <cbor.h>
#include <stdbool.h>
#include <stdint.h>

#define GPS_MIN_SATS_FOR_LOCK 4

// GPS fix types
typedef enum {
  GPS_FIX_NONE = 0,
  GPS_FIX_DR_ONLY = 1,   // Dead reckoning only
  GPS_FIX_2D = 2,        // 2D fix
  GPS_FIX_3D = 3,        // 3D fix
  GPS_FIX_GNSS_DR = 4,   // GNSS + dead reckoning
  GPS_FIX_TIME_ONLY = 5, // Time only fix
} gps_fix_type_t;

// GPS configuration states
typedef enum {
  GPS_DETECT_BAUD = 0,
  GPS_CHANGE_BAUD = 1,
  GPS_CHECK_PERFORMANCE = 2,     // Check/set performance mode for M10 before rate config
  GPS_CONFIG_CONSTELLATIONS = 3, // Configure GNSS constellations for M9/M10
  GPS_CONFIG_GROUND_ASSIST = 4,  // Configure ground assistance features
  GPS_CONFIG_RATE = 5,
  GPS_CONFIG_NAV5 = 6,
  GPS_CONFIG_SBAS = 7,
  GPS_WAITING_FOR_LOCK = 8, // Configuration complete, waiting for initial lock
  GPS_RUNNING = 9,          // Has lock, running normally with reduced messages
  GPS_NOT_DETECTED = 10,
} gps_config_state_t;

// GPS power modes
typedef enum {
  GPS_POWER_FULL = 0,     // Full power (performance mode)
  GPS_POWER_BALANCED = 1, // Balanced mode
  GPS_POWER_LOW = 2,      // Low power mode
  GPS_POWER_UNKNOWN = 0xFF
} gps_power_mode_t;

// Individual constellation status
typedef struct {
  uint8_t enabled;
  uint8_t healthy;
  uint8_t sats_in_view;
  uint8_t sats_used;
} gps_constellation_t;

// Comprehensive GPS status
typedef struct {
  // Configuration state
  gps_config_state_t state;

  // Basic info
  gps_fix_type_t fix_type;
  uint8_t fix_quality; // 0-100% quality indicator

  // Satellites
  uint8_t sats_in_view;
  uint8_t sats_used;

  // Individual constellation status
  gps_constellation_t gps;
  gps_constellation_t glonass;
  gps_constellation_t galileo;
  gps_constellation_t beidou;

  // DOP (Dilution of Precision) values
  uint16_t pdop; // Position DOP [x0.01]

  // Accuracy estimates
  uint32_t h_acc; // Horizontal accuracy [mm]
  uint32_t v_acc; // Vertical accuracy [mm]

  // Performance
  gps_power_mode_t power_mode;
  uint8_t update_rate; // Current update rate [Hz]

  // Signal quality
  uint8_t avg_cno; // Average carrier-to-noise ratio [dBHz]
} gps_status_t;

// CBOR member definitions
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
bool gps_update();

// CBOR function declarations
cbor_result_t cbor_encode_gps_constellation_t(cbor_value_t *enc, const gps_constellation_t *o);
cbor_result_t cbor_encode_gps_status_t(cbor_value_t *enc, const gps_status_t *o);