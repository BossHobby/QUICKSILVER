#pragma once

#include "core/profile.h"
#include "core/project.h"
#include "driver/dma.h"
#include "driver/gpio.h"
#include "driver/motor.h"

typedef struct {
  uint32_t pin_mask;
  uint32_t dshot_port;

  uint32_t set_mask;
  uint32_t reset_mask;
} dshot_pin_t;

typedef struct {
  gpio_port_t *gpio;

  uint32_t set_mask;
  uint32_t reset_mask;

  resource_tag_t timer_tag;

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

#define GCR_TIME ((DSHOT_TIME * 5) / 4)
#define GCR_SYMBOL_TIME (PWM_CLOCK_FREQ_HZ / (3 * GCR_TIME * 1000 - 1))

// 30us delay, 5us buffer, 21 bits for worst case
#define GCR_SYMBOL_TIME_MAX (((DSHOT_TIME_MAX * 5) / 4) * 3)
#define GCR_DMA_BUFFER_SIZE (((30 + 5) * GCR_SYMBOL_TIME_MAX) / 1000 + 21 * 3)

extern volatile uint32_t dshot_phase;

extern uint8_t dshot_gpio_port_count;
extern dshot_gpio_port_t dshot_gpio_ports[DSHOT_MAX_PORT_COUNT];

extern volatile DMA_RAM uint16_t dshot_input_buffer[DSHOT_MAX_PORT_COUNT][GCR_DMA_BUFFER_SIZE];
extern volatile DMA_RAM uint32_t dshot_output_buffer[DSHOT_MAX_PORT_COUNT][DSHOT_DMA_BUFFER_SIZE];

const dshot_gpio_port_t *dshot_gpio_for_device(const dma_device_t dev);

void dshot_init_gpio_port(dshot_gpio_port_t *port);

void dshot_gpio_init_output(gpio_pins_t pin);
void dshot_dma_setup_output(uint32_t index);

void dshot_gpio_init_input(gpio_pins_t pin);
void dshot_dma_setup_input(uint32_t index);

uint32_t dshot_decode_eRPM_telemetry_value(uint16_t value);
uint32_t dshot_decode_gcr(uint16_t *dma_buffer, uint32_t pin_mask);
