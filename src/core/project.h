#pragma once

#include "config/config.h"
#include "config/feature.h"

#include "core/target.h"

#ifdef STM32
#include "driver/mcu/stm32/system.h"
#endif

#ifdef AT32
#include "driver/mcu/at32/system.h"
#endif

#ifdef SIMULATOR
#include "driver/mcu/native/system.h"
#endif

#ifdef USE_FAST_RAM
#define FAST_RAM __attribute__((section(".fast_ram"), aligned(4)))
#else
#define FAST_RAM
#endif

#ifdef USE_DMA_RAM
#define DMA_RAM __attribute__((section(".dma_ram"), aligned(32)))
#else
#define DMA_RAM
#endif

#define static_assert(...) _Static_assert(__VA_ARGS__)