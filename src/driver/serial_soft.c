#include "driver/serial_soft.h"

#include "core/project.h"
#include "driver/gpio.h"
#include "driver/interrupt.h"
#include "driver/timer.h"

#ifdef USE_SOFT_SERIAL

#define BAUD_DIVIDER 4

typedef enum {
  START_BIT = 0,
  DATA_BITS = START_BIT + BAUD_DIVIDER,
  STOP_BITS = START_BIT + BAUD_DIVIDER + BAUD_DIVIDER * 8,
  IDLE = 255,
} soft_serial_tx_state_t;

volatile soft_serial_t soft_serial_ports[SERIAL_SOFT_COUNT];

#define PORT soft_serial_ports[port - SERIAL_SOFT_START]
#define TIMER timer_defs[TIMER_TAG_TIM(PORT.timer)]

extern void soft_serial_rx_isr(serial_ports_t);
extern void soft_serial_tx_isr(serial_ports_t);

extern void soft_serial_timer_start(serial_ports_t port);
extern void soft_serial_timer_stop(serial_ports_t port);

static void soft_serial_init_rx(serial_ports_t port) {
  if (PORT.rx == PIN_NONE) {
    return;
  }

  gpio_config_t gpio_init;
  gpio_init.mode = GPIO_INPUT;
  gpio_init.output = GPIO_OPENDRAIN;
  gpio_init.pull = GPIO_UP_PULL;
  gpio_init.drive = GPIO_DRIVE_HIGH;
  gpio_pin_init(PORT.rx, gpio_init);

  if (PORT.half_duplex) {
    PORT.tx_active = false;
  }
  PORT.rx_active = true;
}

static void soft_serial_init_tx(serial_ports_t port) {
  if (PORT.tx == PIN_NONE) {
    return;
  }

  gpio_config_t gpio_init;
  gpio_init.mode = GPIO_OUTPUT;
  gpio_init.output = GPIO_PUSHPULL;
  gpio_init.pull = GPIO_NO_PULL;
  gpio_init.drive = GPIO_DRIVE_HIGH;
  gpio_pin_init(PORT.tx, gpio_init);

  gpio_pin_set(PORT.tx);

  if (PORT.half_duplex) {
    PORT.rx_active = false;
  }
  PORT.tx_active = true;
}

static void soft_serial_set_input(serial_ports_t port) {
  if (PORT.half_duplex && !PORT.rx_active) {
    soft_serial_init_rx(port);
  }
}

static void soft_serial_set_output(serial_ports_t port) {
  if (PORT.half_duplex && !PORT.tx_active) {
    soft_serial_init_tx(port);
  }
}

uint8_t soft_serial_init(serial_port_config_t config) {
  const serial_ports_t port = config.port;
  if (PORT.timer) {
    soft_serial_timer_stop(port);
  }

  const target_serial_port_t *dev = &target.serial_soft_ports[port - SERIAL_SOFT_START];

  PORT.baud = config.baudrate;
  PORT.stop_bits = config.stop_bits == SERIAL_STOP_BITS_2 ? 2 : 1;

  PORT.tx_state = IDLE;
  PORT.rx_state = START_BIT;

  PORT.half_duplex = config.half_duplex || dev->tx == dev->rx;

  if (PORT.half_duplex) {
    PORT.rx = PORT.tx = dev->tx;
  } else {
    PORT.rx = dev->rx;
    PORT.tx = dev->tx;
  }

  if (!PORT.timer) {
    PORT.timer = timer_alloc(TIMER_USE_SOFT_SERIAL);
  }
  timer_up_init(TIMER_TAG_TIM(PORT.timer), 1, PWM_CLOCK_FREQ_HZ / (config.baudrate * BAUD_DIVIDER));
  interrupt_enable(TIMER.irq, TIMER_PRIORITY);

  soft_serial_init_tx(port);
  soft_serial_init_rx(port);
  soft_serial_enable_read(port);

  return 1;
}

void soft_serial_enable_write(serial_ports_t port) {
  PORT.tx_state = IDLE;

  soft_serial_timer_stop(port);
  soft_serial_set_output(port);
  soft_serial_timer_start(port);
}

void soft_serial_enable_read(serial_ports_t port) {
  PORT.rx_state = START_BIT;

  soft_serial_set_input(port);
  soft_serial_timer_start(port);
}

uint8_t soft_serial_read_byte(serial_ports_t port) {
  return PORT.rx_byte;
}

void soft_serial_write_byte(serial_ports_t port, uint8_t byte) {
  PORT.tx_byte = byte;
  PORT.tx_state = START_BIT;
}

void soft_serial_tx_update(serial_ports_t port) {
  if (!PORT.tx_active) {
    return;
  }

  if (PORT.tx_state == IDLE) {
    soft_serial_tx_isr(port);
    return;
  }

  if (PORT.tx_state == START_BIT) {
    gpio_pin_reset(PORT.tx);
  }

  if (PORT.tx_state >= DATA_BITS && PORT.tx_state < STOP_BITS) {
    const uint8_t bit_index = (PORT.tx_state - DATA_BITS) / BAUD_DIVIDER;

    if ((PORT.tx_byte >> bit_index) & 0x01)
      gpio_pin_set(PORT.tx);
    else
      gpio_pin_reset(PORT.tx);
  }

  if (PORT.tx_state >= STOP_BITS && PORT.tx_state < (STOP_BITS + PORT.stop_bits * BAUD_DIVIDER)) {
    gpio_pin_set(PORT.tx);
  }

  if (PORT.tx_state >= (STOP_BITS + PORT.stop_bits * BAUD_DIVIDER)) {
    PORT.tx_state = IDLE;
    return;
  }

  PORT.tx_state++;
}

void soft_serial_rx_update(serial_ports_t port) {
  if (!PORT.rx_active) {
    return;
  }

  static uint32_t timeout = 0;
  if (timeout == 100000) {
    PORT.rx_state = START_BIT;
    timeout = 0;
    return;
  }

  if (PORT.rx_state == START_BIT) {
    if (gpio_pin_read(PORT.rx)) {
      PORT.rx_state++;
    } else {
      timeout++;
    }
    return;
  }

  if (PORT.rx_state > START_BIT && PORT.rx_state <= DATA_BITS) {
    if (!gpio_pin_read(PORT.rx)) {
      PORT.rx_byte = 0;
      PORT.rx_state++;
      timeout = 0;
    } else {
      timeout++;
    }
    return;
  }

  if (PORT.rx_state > DATA_BITS && PORT.rx_state <= STOP_BITS && (PORT.rx_state % BAUD_DIVIDER) == 2) {
    const uint8_t bit_index = (PORT.rx_state - DATA_BITS) / BAUD_DIVIDER;

    if (gpio_pin_read(PORT.rx)) {
      PORT.rx_byte |= (0x01 << bit_index);
    }
  }

  // should be PORT.stop_bits * BAUD_DIVIDER
  if (PORT.rx_state > (STOP_BITS + (BAUD_DIVIDER / 2)) && PORT.rx_state < (STOP_BITS + 1 * BAUD_DIVIDER)) {
    if (!gpio_pin_read(PORT.rx)) {
      PORT.rx_state = START_BIT;
      timeout = 0;
      return;
    }
  }

  PORT.rx_state++;

  // should be PORT.stop_bits * BAUD_DIVIDER
  if (PORT.rx_state == (STOP_BITS + 1 * BAUD_DIVIDER)) {
    soft_serial_rx_isr(port);
    PORT.rx_state = START_BIT;
    timeout = 0;
  }
}

#endif