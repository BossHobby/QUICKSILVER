#include "driver/motor_dshot.h"

#include "core/profile.h"
#include "core/project.h"
#include "driver/dma.h"
#include "driver/gpio.h"
#include "driver/spi.h"
#include "driver/time.h"
#include "flight/control.h"
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

static uint16_t dshot_packet[MOTOR_PIN_MAX]; // 16bits dshot data for 4 motors
static dshot_pin_t dshot_pins[MOTOR_PIN_MAX];

static motor_direction_t motor_dir = MOTOR_FORWARD;
static bool dir_change_done = true;

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
  dshot_gpio_init_output(target.motor_pins[index]);
  if (profile.motor.dshot_telemetry)
    gpio_pin_set(target.motor_pins[index]);
  else
    gpio_pin_reset(target.motor_pins[index]);

  const gpio_pin_def_t *def = &gpio_pin_defs[target.motor_pins[index]];
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
  for (uint32_t i = 0; i < MOTOR_PIN_MAX; i++) {
    dshot_make_packet(profile.motor.motor_pins[i], value, telemetry);
  }
}

// make dshot dma packet, then fire
void dshot_dma_start() {
  if (profile.motor.dshot_telemetry) {
    for (uint32_t i = 0; i < MOTOR_PIN_MAX; i++) {
      const uint32_t port = dshot_pins[i].dshot_port;
      state.dshot_rpm[i] = dshot_decode_eRPM_telemetry_value(dshot_decode_gcr((uint16_t *)dshot_input_buffer[port], dshot_pins[i].pin_mask));
      dshot_gpio_init_output(target.motor_pins[i]);
    }
  }

  for (uint8_t i = 0; i < 16; i++) {
    for (uint32_t j = 0; j < dshot_gpio_port_count; j++) {
      dshot_output_buffer[j][i * 3 + 1] = 0; // clear middle bit
    }
    for (uint8_t motor = 0; motor < MOTOR_PIN_MAX; motor++) {
      // for 1 hold the line high for two timeunits
      // first timeunit is already applied
      const bool bit = dshot_packet[motor] & 0x8000;
      const uint32_t port = dshot_pins[motor].dshot_port;
      dshot_output_buffer[port][i * 3 + 1] |= bit ? dshot_pins[motor].set_mask : dshot_pins[motor].reset_mask;

      dshot_packet[motor] <<= 1;
    }
  }

  dma_prepare_tx_memory((void *)dshot_output_buffer, sizeof(dshot_output_buffer));
  dma_prepare_rx_memory((void *)dshot_input_buffer, sizeof(dshot_input_buffer));

#ifdef STM32F4
  while (spi_dma_is_ready(SPI_PORT1) == 0)
    __NOP();
#endif

  dshot_phase = dshot_gpio_port_count * (profile.motor.dshot_telemetry ? 2 : 1);
  for (uint32_t j = 0; j < dshot_gpio_port_count; j++) {
    dshot_dma_setup_output(j);
  }
}

static void dshot_handle_dir_change() {
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
    if ((time_micros() - dir_change_time) < DSHOT_DIR_CHANGE_CMD_TIME_US) {
      break;
    }

    const uint16_t value = motor_dir == MOTOR_REVERSE ? DSHOT_CMD_ROTATE_REVERSE : DSHOT_CMD_ROTATE_NORMAL;
    if (counter < 10) {
      dshot_make_packet_all(value, true);
      dshot_dma_start();
      counter++;
    } else {
      state = DIR_CHANGE_STOP;
      counter = 0;
    }
    dir_change_time = time_micros();
    break;
  }

  case DIR_CHANGE_STOP:
    dir_change_done = true;
    state = DIR_CHANGE_START;
    break;
  }
}

void motor_dshot_init() {
  dshot_gpio_port_count = 0;
  motor_dir = MOTOR_FORWARD;
  for (uint32_t i = 0; i < MOTOR_PIN_MAX; i++) {
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

  motor_dir = MOTOR_FORWARD;
}

void motor_dshot_write(float *values) {
  if (!dir_change_done) {
    return dshot_handle_dir_change();
  }

  for (uint32_t i = 0; i < MOTOR_PIN_MAX; i++) {
    uint16_t value = 0;
    if (values[i] >= 0.0f) {
      const float pwm = constrain(values[i], 0.0f, 1.0f);
      value = mapf(pwm, 0.0f, 1.0f, 48, 2047);
    } else {
      value = 0;
    }
    dshot_make_packet(profile.motor.motor_pins[i], value, false);
  }

  dshot_dma_start();
}

void motor_dshot_wait_for_ready() {
  while (dshot_phase != 0)
    __NOP();
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