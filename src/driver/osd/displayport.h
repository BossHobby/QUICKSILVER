#pragma once

#include <stdint.h>

#include "driver/osd/osd.h"

void displayport_init();
bool displayport_is_ready();
void displayport_intro();

bool displayport_clear_async();
osd_system_t displayport_check_system();

uint32_t displayport_can_fit();
bool displayport_push_string(uint8_t attr, uint8_t x, uint8_t y, const uint8_t *data, uint8_t size);
bool displayport_flush();