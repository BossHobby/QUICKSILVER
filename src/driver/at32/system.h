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
typedef dma_type dma_port_t;
typedef dmamux_channel_type dma_stream_t;
typedef spi_type spi_port_t;
typedef tmr_type timer_dev_t;
typedef usart_type usart_dev_t;