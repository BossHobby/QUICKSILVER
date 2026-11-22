#pragma once

#include "driver/osd/osd.h"

// Fixed by the shared memory layout the simulator viewer reads.
static constexpr uint8_t SIMULATOR_OSD_COLS = 50;
static constexpr uint8_t SIMULATOR_OSD_ROWS = 18;

void simulator_init();
void simulator_update();

bool simulator_rx_check();

void simulator_osd_init();
bool simulator_osd_is_ready();
void simulator_osd_intro();

bool simulator_osd_clear_async();

uint32_t simulator_osd_can_fit();
bool simulator_osd_push_string(uint8_t attr, uint8_t x, uint8_t y, const uint8_t *data, uint8_t size);
bool simulator_osd_flush();