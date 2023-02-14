#pragma once

#include <stdint.h>

#include "driver/osd.h"

#define HDZERO_ROWS 18
#define HDZERO_COLS 50

void hdzero_init();
bool hdzero_is_ready();
void hdzero_intro();

uint8_t hdzero_clear_async();
osd_system_t hdzero_check_system();

bool hdzero_can_fit(uint8_t size);
bool hdzero_push_string(uint8_t attr, uint8_t x, uint8_t y, const uint8_t *data, uint8_t size);
bool hdzero_flush();