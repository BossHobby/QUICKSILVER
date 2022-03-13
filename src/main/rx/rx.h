#pragma once

#include <stdbool.h>
#include <stdint.h>

#include "project.h"

#ifdef RX_UNIFIED_SERIAL
#define RX_PROTOCOL RX_PROTOCOL_UNIFIED_SERIAL
#endif
#ifdef RX_NRF24_BAYANG_TELEMETRY
#define RX_PROTOCOL RX_PROTOCOL_NRF24_BAYANG_TELEMETRY
#endif
#ifdef RX_BAYANG_PROTOCOL_BLE_BEACON
#define RX_PROTOCOL RX_PROTOCOL_BAYANG_PROTOCOL_BLE_BEACON
#endif
#ifdef RX_BAYANG_PROTOCOL_TELEMETRY_AUTOBIND
#define RX_PROTOCOL RX_PROTOCOL_BAYANG_PROTOCOL_TELEMETRY_AUTOBIND
#endif
#ifdef RX_FRSKY_D8
#define RX_PROTOCOL RX_PROTOCOL_FRSKY_D8
#endif
#if defined(RX_FRSKY_D16_LBT) || defined(RX_FRSKY_D16_FCC)
#define RX_PROTOCOL RX_PROTOCOL_FRSKY_D16
#endif
#ifdef RX_REDPINE
#define RX_PROTOCOL RX_PROTOCOL_REDPINE
#endif
#ifdef RX_EXPRESS_LRS
#define RX_PROTOCOL RX_PROTOCOL_EXPRESS_LRS
#endif

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
  RX_PROTOCOL_FRSKY_D16,
  RX_PROTOCOL_REDPINE,
  RX_PROTOCOL_EXPRESS_LRS,
  RX_PROTOCOL_MAX,
} rx_protocol_t;

typedef enum {
  RX_LQI_SOURCE_PACKET_RATE,
  RX_LQI_SOURCE_CHANNEL,
  RX_LQI_SOURCE_DIRECT,
} rx_lqi_source_t;

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

  AUX_FUNCTION_MAX
} aux_function_t;

uint8_t rx_aux_on(aux_function_t function);

typedef enum {
  INACTIVE,
  CAPTURE_STICKS,
  WAIT_FOR_CONFIRM,
  CALIBRATION_CONFIRMED,
  TIMEOUT,
  CALIBRATION_SUCCESS,
  CALIBRATION_FAILED
} stick_calibration_wizard_t;

void rx_init();
void rx_serial_init();

void rx_update();

float rx_smoothing_hz(rx_protocol_t proto);

void rx_apply_stick_calibration_scale();
void request_stick_calibration_wizard();

void rx_lqi_lost_packet();
void rx_lqi_got_packet();

void rx_lqi_update();
void rx_lqi_update_from_fps(float expected_fps);
void rx_lqi_update_direct(float rssi);

#if defined(RX_DSMX) || defined(RX_DSM2) || defined(RX_UNIFIED_SERIAL)
void rx_spektrum_bind();
#endif
