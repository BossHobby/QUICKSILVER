#pragma once

#include "rx.h"

#define RX_BUFF_SIZE 68

#define LQ_EXPO 0.9f

typedef enum {
  FRAME_INVALID,
  FRAME_IDLE,
  FRAME_RX,
  FRAME_RX_DONE,
  FRAME_TX,
  FRAME_DONE
} frame_status_t;

void rx_serial_find_protocol();

void rx_serial_process_dsmx();
void rx_serial_process_sbus();
void rx_serial_process_ibus();
void rx_serial_process_fport();
void rx_serial_process_crsf();
void rx_serial_process_redpine();

void rx_serial_send_fport_telemetry();