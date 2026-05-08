#pragma once

#include <stdbool.h>
#include <stdint.h>

#include "config/config.h"
#include "config/feature.h"
#include "rx/stick_wizard.h"

#define RX_CHANNEL_MAX 16

typedef enum {
  RX_PROTOCOL_INVALID,
  RX_PROTOCOL_UNIFIED_SERIAL,
  RX_PROTOCOL_SBUS,
  RX_PROTOCOL_CRSF,
  RX_PROTOCOL_IBUS,
  RX_PROTOCOL_FPORT,
  RX_PROTOCOL_DSM,
  RX_PROTOCOL_NRF24_BAYANG_TELEMETRY,
  RX_PROTOCOL_BAYANG_PROTOCOL_BLE_BEACON,
  RX_PROTOCOL_BAYANG_PROTOCOL_TELEMETRY_AUTOBIND,
  RX_PROTOCOL_FRSKY_D8,
  RX_PROTOCOL_FRSKY_D16_FCC,
  RX_PROTOCOL_FRSKY_D16_LBT,
  RX_PROTOCOL_REDPINE,
  RX_PROTOCOL_EXPRESS_LRS,
  RX_PROTOCOL_FLYSKY_AFHDS,
  RX_PROTOCOL_FLYSKY_AFHDS2A,
  RX_PROTOCOL_MAX,
} __attribute__((__packed__)) rx_protocol_t;

typedef enum {
  RX_LQI_SOURCE_PACKET_RATE,
  RX_LQI_SOURCE_CHANNEL,
  RX_LQI_SOURCE_DIRECT,
} __attribute__((__packed__)) rx_lqi_source_t;

typedef enum {
#ifdef VEHICLE_ROVER
  RX_ROLE_THROTTLE,
  RX_ROLE_STEERING,
#else
  RX_ROLE_ROLL,
  RX_ROLE_PITCH,
  RX_ROLE_YAW,
  RX_ROLE_THROTTLE,
#endif
  RX_ROLE_MAX,
} __attribute__((__packed__)) rx_role_t;

typedef struct {
  uint8_t channel;
  float min;
  float center;
  float max;
} rx_role_map_t;

typedef enum {
  RX_CHANNEL_1,
  RX_CHANNEL_2,
  RX_CHANNEL_3,
  RX_CHANNEL_4,
  RX_CHANNEL_5,
  RX_CHANNEL_6,
  RX_CHANNEL_7,
  RX_CHANNEL_8,
  RX_CHANNEL_9,
  RX_CHANNEL_10,
  RX_CHANNEL_11,
  RX_CHANNEL_12,
  RX_CHANNEL_13,
  RX_CHANNEL_14,
  RX_CHANNEL_15,
  RX_CHANNEL_16,
  RX_CHANNEL_OFF,
  RX_CHANNEL_ON,

  RX_CHANNEL_COUNT
} __attribute__((__packed__)) rx_channel_t;

typedef enum {
  AUX_ARMING,
#ifndef VEHICLE_ROVER
  AUX_IDLE_UP,
  AUX_LEVELMODE,
  AUX_RACEMODE,
  AUX_HORIZON,
  AUX_STICK_BOOST_PROFILE,
  UNUSED_AUX_HIGH_RATES,
#endif
  AUX_BUZZER_ENABLE,
#ifndef VEHICLE_ROVER
  AUX_TURTLE,
  AUX_MOTOR_TEST,
#endif
  AUX_RSSI,
  AUX_FPV_SWITCH,
  AUX_BLACKBOX,
  AUX_PREARM,
  AUX_OSD_PROFILE,
#ifdef VEHICLE_ROVER
  AUX_RATE_ASSIST,
  AUX_RATE_THROTTLE,
#endif

  AUX_FUNCTION_MAX
} __attribute__((__packed__)) aux_function_t;

typedef struct {
  rx_channel_t channel;
  uint16_t range_min;
  uint16_t range_max;
} aux_function_map_t;

#define AUX_VALUE_MAX 65535
#define AUX_VALUE_MID 32768

#define rx_aux_on(function) ((state.aux_active & (1U << (function))) != 0)
#define rx_aux_value(channel) (state.rx_channels[(channel)])

void rx_init();
void rx_update();
void rx_stop();

void rx_lqi_lost_packet();
void rx_lqi_got_packet();

void rx_lqi_update();
void rx_lqi_update_from_fps(float expected_fps);
void rx_lqi_update_direct(float rssi);

void rx_spektrum_bind();
