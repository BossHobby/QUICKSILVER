#pragma once

#include "drv_serial.h"
#include "profile.h"
#include "rx.h"

#define RX_BUFF_SIZE 128

#define LQ_EXPO 0.9f

typedef struct {
  rx_serial_protocol_t protocol;
} rx_unified_bind_data_t;

#ifdef RX_UNIFIED_SERIAL
typedef enum {
  FRAME_INVALID,
  FRAME_IDLE,
  FRAME_RX,
  FRAME_RX_DONE,
  FRAME_TX,
  FRAME_DONE
} frame_status_t;

typedef enum {
  RX_STATUS_NONE = 0,
  RX_STATUS_DETECTING = 100,
  // RX_STATUS_DETECTING + RX_SERIAL_PROTOCOL_X = detecting proto X
  RX_STATUS_DETECTED = 200,
  // RX_STATUS_DETECTED + RX_SERIAL_PROTOCOL_X = detected proto X
} rx_status_t;

void rx_serial_find_protocol();

void rx_serial_process_dsmx();
void rx_serial_process_sbus();
void rx_serial_process_ibus();
void rx_serial_process_fport();
void rx_serial_process_crsf();
void rx_serial_process_redpine();

void rx_serial_send_fport_telemetry();

void rx_lqi_lost_packet();
void rx_lqi_got_packet();

void rx_lqi_update_fps(uint16_t fixed_fps);
void rx_lqi_update_rssi_from_lqi(float expected_fps);
void rx_lqi_update_rssi_direct(float rssi);

#endif