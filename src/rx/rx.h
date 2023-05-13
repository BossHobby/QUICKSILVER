#pragma once

#include <stdbool.h>
#include <stdint.h>

#include "rx/stick_wizard.h"

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
} rx_protocol_t;

typedef enum {
  RX_LQI_SOURCE_PACKET_RATE,
  RX_LQI_SOURCE_CHANNEL,
  RX_LQI_SOURCE_DIRECT,
} rx_lqi_source_t;

typedef enum {
  RX_MAPPING_AETR,
  RX_MAPPING_TAER,
} rx_channel_mapping_t;

typedef enum {
  AUX_CHANNEL_0,
  AUX_CHANNEL_1,
  AUX_CHANNEL_2,
  AUX_CHANNEL_3,
  AUX_CHANNEL_4,
  AUX_CHANNEL_5,
  AUX_CHANNEL_6,
  AUX_CHANNEL_7,
  AUX_CHANNEL_8,
  AUX_CHANNEL_9,
  AUX_CHANNEL_10,
  AUX_CHANNEL_11,
  AUX_CHANNEL_OFF,
  AUX_CHANNEL_ON,
  AUX_CHANNEL_GESTURE,

  AUX_CHANNEL_MAX
} aux_channel_t;

typedef enum {
  AUX_ARMING,
  AUX_IDLE_UP,
  AUX_LEVELMODE,
  AUX_RACEMODE,
  AUX_HORIZON,
  AUX_STICK_BOOST_PROFILE,
  AUX_RATE_PROFILE,
  AUX_BUZZER_ENABLE,
  AUX_TURTLE,
  AUX_MOTOR_TEST,
  AUX_RSSI,
  AUX_FPV_SWITCH,
  AUX_BLACKBOX,
  AUX_PREARM,

  AUX_FUNCTION_MAX
} aux_function_t;

uint8_t rx_aux_on(aux_function_t function);

void rx_init();
void rx_update();

float rx_smoothing_hz();
void rx_map_channels(const float channels[4]);

void rx_lqi_lost_packet();
void rx_lqi_got_packet();

void rx_lqi_update();
void rx_lqi_update_from_fps(float expected_fps);
void rx_lqi_update_direct(float rssi);

void rx_spektrum_bind();
