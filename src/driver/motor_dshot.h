#pragma once

#include "core/profile.h"
#include "core/project.h"
#include "driver/dma.h"
#include "driver/gpio.h"
#include "driver/motor.h"

#define DSHOT_CMD_BEEP1 1
#define DSHOT_CMD_BEEP2 2
#define DSHOT_CMD_BEEP3 3
#define DSHOT_CMD_BEEP4 4
#define DSHOT_CMD_BEEP5 5 // 5 currently uses the same tone as 4 in BLHeli_S.

#define DSHOT_CMD_SPIN_DIRECTION_1 7
#define DSHOT_CMD_SPIN_DIRECTION_2 8
#define DSHOT_CMD_SAVE_SETTINGS 12

#define DSHOT_CMD_SPIN_DIRECTION_NORMAL 20
#define DSHOT_CMD_SPIN_DIRECTION_REVERSED 21

#define DSHOT_CMD_MOTOR_STOP 0

#define DSHOT_THROTTLE_MIN 48
#define DSHOT_THROTTLE_MAX 2047

#define DSHOT_3D_REVERSE_MIN 48
#define DSHOT_3D_REVERSE_MAX 1047
#define DSHOT_3D_FORWARD_MIN 1048
#define DSHOT_3D_FORWARD_MAX 2047

#define DSHOT_DIR_CHANGE_IDLE_TIME_US 10000
#define DSHOT_DIR_CHANGE_CMD_TIME_US 1000

#define DSHOT_TIME profile.motor.dshot_time

#define GCR_TIME ((DSHOT_TIME * 5) / 4)

// 30us delay, 5us buffer, 21 bits for worst case, sampled three times per bit
#define GCR_SYMBOL_TIME_MAX (((DSHOT_TIME_MAX * 5) / 4) * 3)
#define GCR_DMA_BUFFER_SIZE (((30 + 5) * GCR_SYMBOL_TIME_MAX) / 1000 + 21 * 3)

// Transport, one per MCU family; outputs are target output slots.
// Prepares the pins of the outputs in mask, from motor_dshot_init().
void dshot_init_outputs(uint32_t mask);
// With bidirectional DShot, first decodes the replies to the previous frame
// into state.dshot_rpm; then sends packets[i] to every output in mask.
void dshot_send_frame(const uint16_t *packets, uint32_t mask);
// True once the frame, and with bidirectional DShot the reply capture, ended.
bool dshot_frame_done();

// eRPM / 100 from a reply sampled three times per GCR bit, one sample per
// element masked by pin_mask; 0 when invalid or stopped.
uint32_t dshot_decode_gcr(uint16_t *dma_buffer, uint32_t pin_mask);

#ifdef USE_DSHOT_BITBANG
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

#define DSHOT_SYMBOL_TIME (PWM_CLOCK_FREQ_HZ / (3 * DSHOT_TIME * 1000) - 1)
#define GCR_SYMBOL_TIME (PWM_CLOCK_FREQ_HZ / (3 * GCR_TIME * 1000) - 1)

#define DSHOT_MAX_PORT_COUNT 3

#define DSHOT_DMA_SYMBOLS (16)
#define DSHOT_DMA_BUFFER_SIZE (3 * DSHOT_DMA_SYMBOLS)

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
#endif
