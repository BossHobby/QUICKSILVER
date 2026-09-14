#pragma once

#include <FreeRTOS.h>

void vbat_init();
// Runs the measurement pass when due and reports the next service deadline in
// ticks; portMAX_DELAY is never returned (fixed cadence).
TickType_t vbat_calc();
