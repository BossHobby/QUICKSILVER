#include "driver/reset.h"

#include "core/project.h"

uint8_t system_reset_failloop = 0;

void system_reset() {
#ifndef SIMULATOR
  NVIC_SystemReset();
#endif
}
