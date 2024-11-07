#pragma once

#ifdef STM32
#include "driver/mcu/stm32/system.h"
#endif

#ifdef AT32
#include "driver/mcu/at32/system.h"
#endif

#ifdef SIMULATOR
#include "driver/mcu/native/system.h"
#endif