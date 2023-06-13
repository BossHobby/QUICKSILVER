#pragma once

#ifdef AT32F4
#include <at32f435_437_clock.h>
#include <at32f435_437_conf.h>
#endif

#ifdef AT32F435
#define SYS_CLOCK_FREQ_HZ 288000000
#define PWM_CLOCK_FREQ_HZ 288000000
#define SPI_CLOCK_FREQ_HZ (SYS_CLOCK_FREQ_HZ / 2)

#define LOOPTIME_MAX 125

#define UID_BASE 0x1FFFF7E8
#endif