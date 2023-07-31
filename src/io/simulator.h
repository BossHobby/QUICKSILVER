#pragma once

#include "driver/osd.h"

void simulator_init();
void simulator_update();

bool simulator_rx_check();

void simulator_osd_init();
bool simulator_osd_is_ready();
void simulator_osd_intro();

uint8_t simulator_osd_clear_async();
osd_system_t simulator_osd_check_system();

bool simulator_osd_can_fit(uint8_t size);
bool simulator_osd_push_string(uint8_t attr, uint8_t x, uint8_t y, const uint8_t *data, uint8_t size);
bool simulator_osd_flush();