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

// Code that never runs in normal flight: configuration, maintenance, bench
// tests, turtle and failsafe handling. Kept out of line and optimized for size
// on every MCU, so it does not dilute the Flight loop's code; callers treat
// the branch as unlikely. Where only part of flash runs at full speed, it is
// placed in the slow part.
#ifdef USE_SLOW_FLASH
#define COLD_CODE __attribute__((section(".slow_flash"), cold, noinline))
#else
#define COLD_CODE __attribute__((cold, noinline))
#endif

#ifdef USE_DMA_RAM
#define DMA_RAM __attribute__((section(".dma_ram"), aligned(32)))
#elif defined(STM32F7)
// The linker places these first in .bss, inside the uncached DTCM. The .bss
// prefix keeps the section NOBITS, so it is zeroed with .bss and costs no flash.
#define DMA_RAM __attribute__((section(".bss.dma_ram"), aligned(32)))
#else
#define DMA_RAM
#endif
