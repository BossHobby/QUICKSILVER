#pragma once

#include "driver/dma.h"
#include "driver/gpio.h"
#include "driver/motor.h"

typedef struct {
  gpio_port_t *port;
  uint32_t pin;

  uint32_t dshot_port;
} dshot_pin_t;

typedef struct {
  gpio_port_t *gpio;

  uint32_t port_low;  // motor pins for BSRRL, for setting pins low
  uint32_t port_high; // motor pins for BSRRH, for setting pins high

  uint32_t timer_channel;
  dma_device_t dma_device;
} dshot_gpio_port_t;

#define DSHOT_CMD_BEEP1 1
#define DSHOT_CMD_BEEP2 2
#define DSHOT_CMD_BEEP3 3
#define DSHOT_CMD_BEEP4 4
#define DSHOT_CMD_BEEP5 5 // 5 currently uses the same tone as 4 in BLHeli_S.

#define DSHOT_CMD_ROTATE_NORMAL 20
#define DSHOT_CMD_ROTATE_REVERSE 21

#define DSHOT_DIR_CHANGE_IDLE_TIME_US 10000
#define DSHOT_DIR_CHANGE_CMD_TIME_US 1000

#define DSHOT_TIME profile.motor.dshot_time
#define DSHOT_SYMBOL_TIME (PWM_CLOCK_FREQ_HZ / (3 * DSHOT_TIME * 1000) - 1)

#define DSHOT_MAX_PORT_COUNT 3
#define DSHOT_DMA_SYMBOLS (16)
#define DSHOT_DMA_BUFFER_SIZE (3 * DSHOT_DMA_SYMBOLS)