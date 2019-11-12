#pragma once

#include <stdint.h>

void serial_smart_audio_send_payload(uint8_t cmd, uint8_t *payload, uint32_t size);