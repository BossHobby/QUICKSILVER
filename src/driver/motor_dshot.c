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

volatile uint32_t dshot_dma_phase = 0; // 0: idle, 1 - (gpio_port_count + 1): handle port n
uint16_t dshot_packet[MOTOR_PIN_MAX];  // 16bits dshot data for 4 motors
dshot_pin_t dshot_pins[MOTOR_PIN_MAX];

uint8_t dshot_gpio_port_count = 0;
dshot_gpio_port_t dshot_gpio_ports[DSHOT_MAX_PORT_COUNT] = {
    {.dma_device = DMA_DEVICE_DSHOT_CH1},
    {.dma_device = DMA_DEVICE_DSHOT_CH2},
    {.dma_device = DMA_DEVICE_DSHOT_CH3},
};

motor_direction_t motor_dir = MOTOR_FORWARD;
static bool dir_change_done = true;

volatile DMA_RAM uint32_t port_dma_buffer[DSHOT_MAX_PORT_COUNT][DSHOT_DMA_BUFFER_SIZE];

extern void dshot_dma_setup_port(uint32_t index);
extern void dshot_init_gpio_port(dshot_gpio_port_t *port);

const dshot_gpio_port_t *dshot_gpio_for_device(const dma_device_t dev) {
  return &dshot_gpio_ports[dev - DMA_DEVICE_DSHOT_CH1];
}

void dshot_init_motor_pin(uint32_t index) {
  gpio_config_t gpio_init;
  gpio_init.mode = GPIO_OUTPUT;
  gpio_init.output = GPIO_PUSHPULL;
  gpio_init.drive = GPIO_DRIVE_HIGH;
  gpio_init.pull = GPIO_NO_PULL;
  gpio_pin_init(target.motor_pins[index], gpio_init);
  gpio_pin_reset(target.motor_pins[index]);

  dshot_pins[index].port = gpio_pin_defs[target.motor_pins[index]].port;
  dshot_pins[index].pin = gpio_pin_defs[target.motor_pins[index]].pin;
  dshot_pins[index].dshot_port = 0;

  for (uint8_t i = 0; i < DSHOT_MAX_PORT_COUNT; i++) {
    if (dshot_gpio_ports[i].gpio == dshot_pins[index].port || i == dshot_gpio_port_count) {
      // we already got a matching port in our array
      // or we reached the first empty spot
      dshot_gpio_ports[i].gpio = dshot_pins[index].port;
      dshot_gpio_ports[i].port_high |= dshot_pins[index].pin;
      dshot_gpio_ports[i].port_low |= (dshot_pins[index].pin << 16);

      dshot_pins[index].dshot_port = i;

      if (i + 1 > dshot_gpio_port_count) {
        dshot_gpio_port_count = i + 1;
      }

      break;
    }
  }
}

void dshot_make_packet(uint8_t number, uint16_t value, bool telemetry) {
  const uint16_t packet = (value << 1) | (telemetry ? 1 : 0);

  uint16_t csum = 0;
  uint16_t csum_data = packet;
  for (uint8_t i = 0; i < 3; ++i) {
    csum ^= csum_data; // xor data by nibbles
    csum_data >>= 4;
  }

  dshot_packet[number] = (packet << 4) | (csum & 0xf);
}

void dshot_make_packet_all(uint16_t value, bool telemetry) {
  for (uint32_t i = 0; i < MOTOR_PIN_MAX; i++) {
    dshot_make_packet(profile.motor.motor_pins[i], value, telemetry);
  }
}

// make dshot dma packet, then fire
void dshot_dma_start() {
  for (uint8_t i = 0; i < 16; i++) {
    for (uint32_t j = 0; j < dshot_gpio_port_count; j++) {
      port_dma_buffer[j][i * 3 + 1] = 0; // clear middle bit
    }
    for (uint8_t motor = 0; motor < MOTOR_PIN_MAX; motor++) {
      const uint32_t port = dshot_pins[motor].dshot_port;
      const uint32_t motor_high = (dshot_pins[motor].pin);
      const uint32_t motor_low = (dshot_pins[motor].pin << 16);

      const bool bit = dshot_packet[motor] & 0x8000;

      // for 1 hold the line high for two timeunits
      // first timeunit is already applied
      port_dma_buffer[port][i * 3 + 1] |= bit ? motor_high : motor_low;

      dshot_packet[motor] <<= 1;
    }
  }

  dma_prepare_tx_memory((void *)port_dma_buffer, sizeof(port_dma_buffer));

#ifdef STM32F4
  while (spi_dma_is_ready(SPI_PORT1) == 0)
    __NOP();
#endif

  dshot_dma_phase = dshot_gpio_port_count;
  for (uint32_t j = 0; j < dshot_gpio_port_count; j++) {
    dshot_dma_setup_port(j);
  }
}

void motor_dshot_init() {
  dshot_gpio_port_count = 0;

  for (uint32_t i = 0; i < MOTOR_PIN_MAX; i++) {
    dshot_init_motor_pin(i);
  }

  for (uint32_t j = 0; j < dshot_gpio_port_count; j++) {
    dshot_init_gpio_port(&dshot_gpio_ports[j]);

    for (uint8_t i = 0; i < 16; i++) {
      port_dma_buffer[j][i * 3 + 0] = dshot_gpio_ports[j].port_high; // start bit
      port_dma_buffer[j][i * 3 + 1] = 0;                             // actual bit, set below
      port_dma_buffer[j][i * 3 + 2] = dshot_gpio_ports[j].port_low;  // return line to low
    }
  }

  motor_dir = MOTOR_FORWARD;
}

void motor_dshot_write(float *values) {
  if (dir_change_done) {
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
  } else {
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