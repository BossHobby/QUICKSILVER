#include "driver/reset.h"

#include "core/project.h"

void system_reset() {
  NVIC_SystemReset();
}