#pragma once

#include <stdbool.h>
#include <stdint.h>

extern uint32_t vtx_last_valid_read;
extern uint32_t vtx_last_request;

extern uint8_t vtx_payload[32];
extern uint8_t vtx_payload_offset;

bool serial_vtx_is_ready();
bool serial_vtx_wait_for_ready();

bool serial_vtx_send_data(uint8_t *data, uint32_t size);
uint8_t serial_vtx_read_byte(uint8_t *data);