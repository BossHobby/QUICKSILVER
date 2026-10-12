#pragma once

#include <stdint.h>

// Smallest ARMv7-M MPU region; its base must be aligned to its size.
static constexpr uint32_t MPU_GUARD_SIZE = 32;
// Regions 1-7 exist on every core (region 0 maps H7 DMA RAM); one guards the
// main stack.
static constexpr uint32_t MPU_STACK_GUARD_MAX = 6;

// Programs no-access regions once at boot: one at the bottom of the main
// (interrupt) stack and one per given guard, each MPU_GUARD_SIZE bytes and
// aligned to it. Context switches never reprogram the MPU.
void mpu_stack_guard_init(const uint8_t *const *guards, uint32_t count);
