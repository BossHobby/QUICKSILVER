#include "driver/rcc.h"

// Mock RCC implementation for simulator
void rcc_enable(rcc_reg_t reg) {
  // In a simulator, there's no actual hardware to enable
  // Just return success
}