#pragma once

#include "config/config.h"
#include "config/feature.h"

#include "driver/mcu/system.h"

#include "core/target.h"

#ifdef USE_FAST_RAM
#define FAST_RAM __attribute__((section(".fast_ram"), aligned(4)))
#else
#define FAST_RAM
#endif

#ifdef USE_SLOW_FLASH
// Configuration, maintenance and startup code that never runs per Flight loop.
// It may execute from flash with wait states, leaving fast flash for hot code.
// Inlining would move it back into its caller's section.
#define SLOW_FLASH __attribute__((section(".slow_flash"), noinline))
#else
#define SLOW_FLASH
#endif

#ifdef USE_DMA_RAM
#define DMA_RAM __attribute__((section(".dma_ram"), aligned(32)))
#else
#define DMA_RAM
#endif
