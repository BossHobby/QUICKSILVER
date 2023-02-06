#pragma once

#include <stdbool.h>
#include <stdint.h>

#define VTX_BUFFER_SIZE 256

typedef enum {
  VTX_ERROR,
  VTX_IDLE,
  VTX_WAIT,
  VTX_SUCCESS
} vtx_update_result_t;

bool serial_vtx_is_ready();
bool serial_vtx_wait_for_ready();

bool serial_vtx_send_data(uint8_t *data, uint32_t size);
uint8_t serial_vtx_read_byte(uint8_t *data);