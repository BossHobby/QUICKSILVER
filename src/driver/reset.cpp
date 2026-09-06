#include "driver/reset.h"

#include "core/project.h"

void system_reset() {
#ifndef SIMULATOR
  NVIC_SystemReset();
#endif
}