#pragma once

#include <FreeRTOS.h>

void pinio_init();
// IO owns GPIO writes; Flight publishes the requested AUX bits.
TickType_t pinio_update();
