#include "driver/motor_dshot.h"

#include "control/control.h"
#include "core/profile.h"
#include "core/project.h"
#include "driver/dma.h"
#include "driver/gpio.h"
#include "driver/spi.h"
#include "driver/time.h"
#include "util/util.h"

#ifdef USE_MOTOR_DSHOT

typedef enum {
  DIR_CHANGE_START,
  DIR_CHANGE_DELAY,
  DIR_CHANGE_CMD,
  DIR_CHANGE_STOP,
} dir_change_state_t;

volatile uint32_t dshot_phase = 0;

uint8_t dshot_gpio_port_count = 0;
dshot_gpio_port_t dshot_gpio_ports[DSHOT_MAX_PORT_COUNT] = {
    {.dma_device = DMA_DEVICE_DSHOT_CH1},
    {.dma_device = DMA_DEVICE_DSHOT_CH2},
    {.dma_device = DMA_DEVICE_DSHOT_CH3},
};

volatile DMA_RAM uint16_t dshot_input_buffer[DSHOT_MAX_PORT_COUNT][GCR_DMA_BUFFER_SIZE];
volatile DMA_RAM uint32_t dshot_output_buffer[DSHOT_MAX_PORT_COUNT][DSHOT_DMA_BUFFER_SIZE];

static uint16_t dshot_packet[MOTOR_PIN_MAX];
static dshot_pin_t dshot_pins[MOTOR_PIN_MAX];
// Outputs whose pins motor_dshot_init() configured.
static uint32_t dshot_motor_mask = 0;

static motor_direction_t motor_dir = MOTOR_FORWARD;
static bool dir_change_done = true;

static bool dshot_motor_slot_active(uint32_t slot) {
  if (slot >= MOTOR_PIN_MAX) {
    return false;
  }
  return target.outputs[slot].pin != PIN_NONE && (target.outputs[slot].caps & OUTPUT_CAP_DSHOT) != 0 &&
         profile_output_slot_uses_protocol(slot, OUTPUT_PROTOCOL_DSHOT);
}

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

// Configured outputs the profile still assigns to DShot. A USB profile change
// can move an output to another protocol without re-initializing the motors.
static uint32_t dshot_active_mask() {
  return dshot_motor_mask & profile_output_protocol_slots(OUTPUT_PROTOCOL_DSHOT);
}

static void dshot_make_packet(uint8_t number, uint16_t value, bool telemetry) {
  const uint16_t packet = (value << 1) | (telemetry ? 1 : 0);

  uint16_t csum = 0;
  uint16_t csum_data = packet;
  for (uint8_t i = 0; i < 3; ++i) {
    csum ^= csum_data; // xor data by nibbles
    csum_data >>= 4;
  }

  if (profile.motor.dshot_telemetry) {
    csum = ~csum;
  }
  csum &= 0xf;

  dshot_packet[number] = (packet << 4) | csum;
}

static void dshot_make_packet_all(uint16_t value, bool telemetry) {
  const uint32_t active = dshot_active_mask();
  for (uint32_t i = 0; i < MOTOR_PIN_MAX; i++) {
    if (active & (1U << i))
      dshot_make_packet(i, value, telemetry);
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

// Make the DMA frame from dshot_packet, then fire. dshot_packet is left
// intact; every caller builds the packets for a frame before starting it.
void dshot_dma_start() {
  const uint32_t active = dshot_active_mask();
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
    uint32_t packet = dshot_packet[motor];
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

void motor_dshot_wait_for_ready() {
  while (dshot_phase != 0)
    __NOP();
}

static void dshot_send_packet(uint8_t index, uint16_t value, bool telemetry) {
  motor_dshot_wait_for_ready();

  const uint32_t active = dshot_active_mask();
  for (uint32_t i = 0; i < MOTOR_PIN_MAX; i++) {
    if (active & (1U << i))
      dshot_make_packet(i, i == index ? value : DSHOT_CMD_MOTOR_STOP, telemetry && i == index);
  }

  dshot_dma_start();
}

COLD_CODE static void dshot_handle_dir_change() {
  static uint8_t counter = 0;
  static uint32_t dir_change_time = 0;
  static dir_change_state_t state = DIR_CHANGE_START;

  switch (state) {
  case DIR_CHANGE_START: {
    if (counter < 100) {
      dshot_make_packet_all(0, false);
      dshot_dma_start();
      counter++;
    } else {
      state = DIR_CHANGE_DELAY;
      counter = 0;
    }
    dir_change_time = time_micros();
    break;
  }

  case DIR_CHANGE_DELAY:
    if ((time_micros() - dir_change_time) < DSHOT_DIR_CHANGE_IDLE_TIME_US) {
      break;
    }
    state = DIR_CHANGE_CMD;
    dir_change_time = time_micros();
    break;

  case DIR_CHANGE_CMD: {
    const uint16_t value = motor_dir == MOTOR_REVERSE ? DSHOT_CMD_SPIN_DIRECTION_REVERSED : DSHOT_CMD_SPIN_DIRECTION_NORMAL;
    dshot_make_packet_all(value, true);
    dshot_dma_start();
    if ((time_micros() - dir_change_time) >= DSHOT_DIR_CHANGE_CMD_TIME_US) {
      state = DIR_CHANGE_STOP;
    }
    break;
  }

  case DIR_CHANGE_STOP:
    dir_change_done = true;
    state = DIR_CHANGE_START;
    break;
  }
}

static uint16_t dshot_encode_motor_value(float value) {
#ifdef VEHICLE_ROVER
  if (profile.rover.reversible) {
    if (value == MOTOR_OFF) {
      return DSHOT_CMD_MOTOR_STOP;
    }
    if (value > 0.0f) {
      const float pwm = constrain(value, 0.0f, 1.0f);
      return mapf(pwm, 0.0f, 1.0f, DSHOT_3D_FORWARD_MIN, DSHOT_3D_FORWARD_MAX);
    }
    if (value < 0.0f) {
      const float pwm = constrain(value, -1.0f, 0.0f);
      return mapf(pwm, -1.0f, 0.0f, DSHOT_3D_REVERSE_MIN, DSHOT_3D_REVERSE_MAX);
    }
    return DSHOT_CMD_MOTOR_STOP;
  }
#endif

  if (value >= 0.0f) {
    const float pwm = constrain(value, 0.0f, 1.0f);
    return mapf(pwm, 0.0f, 1.0f, DSHOT_THROTTLE_MIN, DSHOT_THROTTLE_MAX);
  }

  return DSHOT_CMD_MOTOR_STOP;
}

void motor_dshot_init() {
  dshot_gpio_port_count = 0;
  dshot_motor_mask = 0;
  motor_dir = MOTOR_FORWARD;
  for (uint32_t i = 0; i < MOTOR_PIN_MAX; i++) {
    if (!dshot_motor_slot_active(i)) {
      continue;
    }
    dshot_init_motor_pin(i);
    dshot_motor_mask |= 1U << i;
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

  motor_dir = MOTOR_FORWARD;
}

void motor_dshot_write(float *values) {
  if (!dir_change_done) {
    return dshot_handle_dir_change();
  }

  const uint32_t active = dshot_active_mask();
  for (uint32_t i = 0; i < MOTOR_PIN_MAX; i++) {
    if (active & (1U << i))
      dshot_make_packet(i, dshot_encode_motor_value(values[i]), false);
  }

  dshot_dma_start();
}

void motor_dshot_set_direction(motor_direction_t dir) {
  if (dir_change_done) {
    motor_dir = dir;
    dir_change_done = false;
  }
}

bool motor_dshot_direction_change_done() {
  return dir_change_done;
}

bool motor_dshot_configure_direction(uint8_t index, motor_direction_t dir) {
  if (!dir_change_done || index >= MOTOR_PIN_MAX || !(dshot_active_mask() & (1U << index))) {
    return false;
  }

  for (uint8_t i = 0; i < 100; i++) {
    dshot_send_packet(index, DSHOT_CMD_MOTOR_STOP, false);
  }

  time_delay_us(DSHOT_DIR_CHANGE_IDLE_TIME_US);

  // 7/8 configure the base direction; 20/21 temporarily reverse it for turtle mode.
  const uint16_t command = dir == MOTOR_REVERSE ? DSHOT_CMD_SPIN_DIRECTION_2 : DSHOT_CMD_SPIN_DIRECTION_1;
  for (uint8_t i = 0; i < 10; i++) {
    dshot_send_packet(index, command, true);
    time_delay_us(DSHOT_DIR_CHANGE_CMD_TIME_US);
  }

  for (uint8_t i = 0; i < 10; i++) {
    dshot_send_packet(index, DSHOT_CMD_SAVE_SETTINGS, true);
    time_delay_us(DSHOT_DIR_CHANGE_CMD_TIME_US);
  }

  // Allow the ESC to finish its nonvolatile write before sending more packets.
  time_delay_ms(35);

  dshot_send_packet(index, DSHOT_CMD_MOTOR_STOP, false);
  motor_dshot_wait_for_ready();

  return true;
}

void motor_dshot_beep() {
  static uint32_t last_time = 0;
  const uint32_t time = time_millis() - last_time;

  static uint8_t beep_command = DSHOT_CMD_BEEP1;
  dshot_make_packet_all(beep_command, true);
  dshot_dma_start();

  if (time >= 500) {
    beep_command++;
    if (beep_command > DSHOT_CMD_BEEP5) {
      beep_command = DSHOT_CMD_BEEP1;
    }

    last_time = time_millis();
  }
}
#endif
