#pragma once

#ifdef AT32F4
#include <at32f435_437_conf.h>
#endif

#ifdef AT32F435
#define SYS_CLOCK_FREQ_HZ 288000000
#define PWM_CLOCK_FREQ_HZ 288000000
#define SPI_CLOCK_FREQ_HZ (SYS_CLOCK_FREQ_HZ / 2)

#define LOOPTIME LOOPTIME_8K
#endif

typedef gpio_type gpio_port_t;