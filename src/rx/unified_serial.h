#pragma once

#include "core/profile.h"
#include "driver/serial.h"
#include "rx/rx.h"

#define RX_BUFF_SIZE 128

#define LQ_EXPO 0.9f

typedef struct {
  rx_serial_protocol_t protocol;
} rx_unified_bind_data_t;

typedef enum {
  FRAME_INVALID,
  FRAME_IDLE,
  FRAME_RX,
  FRAME_RX_DONE,
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

float rx_serial_crsf_expected_fps();
float rx_serial_dsm_expected_fps();
uint16_t rx_serial_crsf_smoothing_cutoff();
uint16_t rx_serial_dsm_smoothing_cutoff();

void rx_serial_init();
bool rx_serial_check();

bool rx_serial_process_dsm();
bool rx_serial_process_sbus();
bool rx_serial_process_ibus();
bool rx_serial_process_fport();
bool rx_serial_process_crsf();
bool rx_serial_process_redpine();

void rx_serial_send_telemetry(uint32_t size);

void rx_serial_send_fport_telemetry();
void rx_serial_send_crsf_telemetry();