#pragma once

#include <FreeRTOS.h>

void buzzer_init();
// Pulse edges are timestamp-driven; reports the next fixed service deadline.
TickType_t buzzer_update();