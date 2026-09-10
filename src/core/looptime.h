#pragma once

#include <stdint.h>

void looptime_init();
void looptime_reset();
// Finish the previous loop and return the next loop's timing boundary.
// Housekeeping after that boundary is included in the next task budget.
uint32_t looptime_update();
