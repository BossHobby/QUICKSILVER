#pragma once

#include <stdint.h>

// Smallest ARMv7-M MPU region; its base must be aligned to its size. ARMv8-M
// regions have the same 32-byte granularity.
static constexpr uint32_t MPU_GUARD_SIZE = 32;
// Regions 1-7 exist on every core (region 0 maps H7 DMA RAM, or F7 ITCM code);
// one guards the main stack.
static constexpr uint32_t MPU_STACK_GUARD_MAX = 6;

// Programs the MPU once at boot: no-access (ARMv8-M: read-only) regions at the bottom of the main
// (interrupt) stack and per given guard, each MPU_GUARD_SIZE bytes and aligned
// to it, and with USE_FAST_CODE ITCM read-only once its code is copied there.
// Context switches never reprogram the MPU.
void mpu_init(const uint8_t *const *guards, uint32_t count);
