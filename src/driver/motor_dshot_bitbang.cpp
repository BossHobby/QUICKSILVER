#include "driver/motor_dshot.h"

#include "control/control.h"
#include "core/failloop.h"
#include "core/profile.h"
#include "core/project.h"
#include "driver/dma.h"
#include "driver/gpio.h"

#ifdef USE_DSHOT_BITBANG

// Bit-bangs every DShot pin of a GPIO port at once: a timer paces DMA writes
// of set/reset masks to the port, three per bit, and with bidirectional
// DShot samples the port the same way for the replies.

volatile uint32_t dshot_phase = 0;

uint8_t dshot_gpio_port_count = 0;
dshot_gpio_port_t dshot_gpio_ports[DSHOT_MAX_PORT_COUNT] = {
    {.dma_device = DMA_DEVICE_DSHOT_CH1},
    {.dma_device = DMA_DEVICE_DSHOT_CH2},
    {.dma_device = DMA_DEVICE_DSHOT_CH3},
};

volatile DMA_RAM uint16_t dshot_input_buffer[DSHOT_MAX_PORT_COUNT][GCR_DMA_BUFFER_SIZE];
volatile DMA_RAM uint32_t dshot_output_buffer[DSHOT_MAX_PORT_COUNT][DSHOT_DMA_BUFFER_SIZE];

static dshot_pin_t dshot_pins[MOTOR_PIN_MAX];

const dshot_gpio_port_t *dshot_gpio_for_device(const dma_device_t dev) {
  return &dshot_gpio_ports[dev - DMA_DEVICE_DSHOT_CH1];
}

void dshot_gpio_init_output(gpio_pins_t pin) {
  gpio_config_t config;
  config.mode = GPIO_OUTPUT;
  config.output = GPIO_PUSHPULL;
  config.drive = GPIO_DRIVE_HIGH;
  config.pull = GPIO_NO_PULL;
  gpio_pin_init(pin, config);
}

void dshot_gpio_init_input(gpio_pins_t pin) {
  gpio_config_t config;
  config.mode = GPIO_INPUT;
  config.output = GPIO_OPENDRAIN;
  config.drive = GPIO_DRIVE_HIGH;
  config.pull = GPIO_NO_PULL;
  gpio_pin_init(pin, config);
}

static void dshot_init_motor_pin(uint32_t index) {
  const gpio_pins_t pin = target.outputs[index].pin;
  if (pin == PIN_NONE) {
    return;
  }

  dshot_gpio_init_output(pin);
  if (profile.motor.dshot_telemetry)
    gpio_pin_set(pin);
  else
    gpio_pin_reset(pin);

  const gpio_pin_def_t *def = &gpio_pin_defs[pin];
  const uint32_t pin_mask = dshot_pins[index].pin_mask = 0x1 << def->pin_index;
  dshot_pins[index].dshot_port = 0;
  dshot_pins[index].set_mask = profile.motor.dshot_telemetry ? (pin_mask << 16) : pin_mask;
  dshot_pins[index].reset_mask = profile.motor.dshot_telemetry ? pin_mask : (pin_mask << 16);

  for (uint8_t i = 0; i < DSHOT_MAX_PORT_COUNT; i++) {
    if (dshot_gpio_ports[i].gpio == def->port || i == dshot_gpio_port_count) {
      // we already got a matching port in our array
      // or we reached the first empty spot
      dshot_gpio_ports[i].gpio = def->port;
      dshot_gpio_ports[i].set_mask |= dshot_pins[index].set_mask;
      dshot_gpio_ports[i].reset_mask |= dshot_pins[index].reset_mask;

      dshot_pins[index].dshot_port = i;

      if (i + 1 > dshot_gpio_port_count) {
        dshot_gpio_port_count = i + 1;
      }
      break;
    }
  }
}

// Decodes the replies to the previous frame, then returns the pins to output.
// Bidirectional DShot runs this every pass, but state.dshot_rpm only reaches
// the Configurator today, so it stays cold. Once eRPM drives flight control
// (an RPM filter), this and dshot_decode_gcr() become hot and worth optimizing.
COLD_CODE static void dshot_read_telemetry(uint32_t active) {
  for (uint32_t i = 0; i < MOTOR_PIN_MAX; i++) {
    if (!(active & (1U << i)))
      continue;
    const uint32_t port = dshot_pins[i].dshot_port;
    state.dshot_rpm[i] = dshot_decode_gcr((uint16_t *)dshot_input_buffer[port], dshot_pins[i].pin_mask);
    dshot_gpio_init_output(target.outputs[i].pin);
  }
}

void dshot_send_frame(const uint16_t *packets, uint32_t active) {
  if (profile.motor.dshot_telemetry) {
    dshot_read_telemetry(active);
  }

  for (uint32_t j = 0; j < dshot_gpio_port_count; j++) {
    for (uint32_t i = 0; i < DSHOT_DMA_SYMBOLS; i++) {
      dshot_output_buffer[j][i * 3 + 1] = 0; // clear middle bit
    }
  }
  for (uint32_t motor = 0; motor < MOTOR_PIN_MAX; motor++) {
    if (!(active & (1U << motor)))
      continue;
    const dshot_pin_t &pin = dshot_pins[motor];
    uint32_t packet = packets[motor];
    // Partial unroll: most of the speed at a quarter of the full unroll's size.
#pragma GCC unroll 4
    for (uint32_t i = 0; i < DSHOT_DMA_SYMBOLS; i++) {
      // for 1 hold the line high for two timeunits
      // first timeunit is already applied
      dshot_output_buffer[pin.dshot_port][i * 3 + 1] |= packet & 0x8000 ? pin.set_mask : pin.reset_mask;
      packet <<= 1;
    }
  }

  dma_prepare_tx_memory((void *)dshot_output_buffer, sizeof(dshot_output_buffer));
  dma_prepare_rx_memory((void *)dshot_input_buffer, sizeof(dshot_input_buffer));

#ifdef STM32F4
  while (!dma_can_use_dma2(DMA_DEVICE_INVALID))
    __NOP();
#endif

  dshot_phase = dshot_gpio_port_count * (profile.motor.dshot_telemetry ? 2 : 1);
  for (uint32_t j = 0; j < dshot_gpio_port_count; j++) {
    dshot_dma_setup_output(j);
  }
}

bool dshot_frame_done() {
  return dshot_phase == 0;
}

void dshot_init_outputs(uint32_t mask) {
  dshot_gpio_port_count = 0;
  for (uint32_t i = 0; i < MOTOR_PIN_MAX; i++) {
    if (mask & (1U << i))
      dshot_init_motor_pin(i);
  }

  for (uint32_t j = 0; j < dshot_gpio_port_count; j++) {
    dshot_gpio_port_t *port = &dshot_gpio_ports[j];

    port->timer_tag = target.dma[port->dma_device].tag;
    if (port->timer_tag == 0) {
      failloop(FAILLOOP_DMA);
    }

    dshot_init_gpio_port(port);

    for (uint8_t i = 0; i < DSHOT_DMA_SYMBOLS; i++) {
      dshot_output_buffer[j][i * 3 + 0] = port->set_mask;   // start bit
      dshot_output_buffer[j][i * 3 + 1] = 0;                // actual bit, set below
      dshot_output_buffer[j][i * 3 + 2] = port->reset_mask; // return line to low
    }
  }
}

#endif
