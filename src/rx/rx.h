#pragma once

#include <stdbool.h>
#include <stdint.h>

#include <FreeRTOS.h>

#include "config/config.h"
#include "config/feature.h"
#include "rx/stick_wizard.h"

#define RX_CHANNEL_MAX 16

// Values are persisted and shared with the Configurator: they stay fixed even
// when a receiver family is compiled out.
typedef enum {
  RX_PROTOCOL_INVALID = 0,
  // Cycles the serial protocols below until one locks, then selects it.
  RX_PROTOCOL_SERIAL_AUTO = 1,
  RX_PROTOCOL_SBUS = 2,
  RX_PROTOCOL_CRSF = 3,
  RX_PROTOCOL_IBUS = 4,
  RX_PROTOCOL_FPORT = 5,
  RX_PROTOCOL_DSM = 6,
  RX_PROTOCOL_REDPINE_SERIAL = 7,
  RX_PROTOCOL_MAVLINK = 8,
#ifdef USE_RX_SPI_FRSKY
  RX_PROTOCOL_FRSKY_D8 = 9,
  RX_PROTOCOL_FRSKY_D16_FCC = 10,
  RX_PROTOCOL_FRSKY_D16_LBT = 11,
  RX_PROTOCOL_REDPINE = 12,
#endif
#ifdef USE_RX_SPI_EXPRESS_LRS
  RX_PROTOCOL_EXPRESS_LRS = 13,
#endif
#ifdef USE_RX_SPI_FLYSKY
  RX_PROTOCOL_FLYSKY_AFHDS = 14,
  RX_PROTOCOL_FLYSKY_AFHDS2A = 15,
#endif
  RX_PROTOCOL_MAX = 16,
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
#ifdef VEHICLE_WING
  AUX_ACROMODE,
#else
  AUX_RACEMODE,
  AUX_HORIZON,
  AUX_STICK_BOOST_PROFILE,
  UNUSED_AUX_HIGH_RATES,
#endif
#endif
  AUX_BUZZER_ENABLE,
#if !defined(VEHICLE_ROVER) && !defined(VEHICLE_WING)
  AUX_TURTLE,
  AUX_MOTOR_TEST,
#endif
  AUX_RSSI,
  AUX_VTX_PIT_MODE,
  AUX_BLACKBOX,
  AUX_PREARM,
  AUX_OSD_PROFILE,
  AUX_RETURN_TO_HOME,
#ifdef VEHICLE_ROVER
  AUX_RATE_ASSIST,
  AUX_RATE_THROTTLE,
#endif
#ifdef VEHICLE_WING
  AUX_AUTOTRIM,
  AUX_AUTOLAUNCH,
  AUX_LOITER,
  AUX_CRUISE,
#endif
#ifndef VEHICLE_ROVER
  AUX_MISSION,
#endif

  AUX_PINIO_1,
  AUX_PINIO_2,
  AUX_PINIO_3,
  AUX_PINIO_4,
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

// Decoder-owned staging, including retained channels in partial frames.
// Consumers use state.rx_channels, published by Flight from the mailbox.
extern uint16_t rx_channels[RX_CHANNEL_MAX];

void rx_init();
// IO thread: polls serial/SPI and publishes receiver state. Reports the next
// service deadline in ticks: fast poll for SPI protocols (time-compared hop
// state machines), the autodetect timer while it cycles, portMAX_DELAY for
// serial protocols whose data arrives via serial_rx_notify_from_isr.
TickType_t rx_update();
// Flight task: consumes received channels and conditions commands before control.
void rx_process();
void rx_stop();

void rx_lqi_lost_packet();
void rx_lqi_got_packet();

void rx_lqi_update(float expected_fps);
void rx_lqi_update_direct(float rssi);

void rx_spektrum_bind();
void rx_reset_bind();
