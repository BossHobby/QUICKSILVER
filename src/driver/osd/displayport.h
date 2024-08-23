#pragma once

#include <stdint.h>

#include "driver/osd/osd.h"

#define DISPLAYPORT_ROWS 18
#define DISPLAYPORT_COLS 50

void displayport_init();
bool displayport_is_ready();
void displayport_intro();

uint8_t displayport_clear_async();
osd_system_t displayport_check_system();

bool displayport_can_fit(uint8_t size);
bool displayport_push_string(uint8_t attr, uint8_t x, uint8_t y, const uint8_t *data, uint8_t size);
bool displayport_flush();