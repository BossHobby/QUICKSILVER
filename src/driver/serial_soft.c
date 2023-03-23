#include "driver/serial_soft.h"

#include "core/project.h"
#include "driver/gpio.h"
#include "driver/interrupt.h"
#include "driver/timer.h"

#define BAUD_DIVIDER 4

typedef enum {
  START_BIT = 0,
  DATA_BITS = START_BIT + BAUD_DIVIDER,
  STOP_BITS = START_BIT + BAUD_DIVIDER + BAUD_DIVIDER * 8,
} soft_serial_tx_state_t;

static volatile soft_serial_t soft_serial_ports[SERIAL_SOFT_COUNT];

#define DEV target.serial_soft_ports[port - SERIAL_SOFT_START]
#define PORT soft_serial_ports[port - SERIAL_SOFT_START]
#define TIMER timer_defs[TIMER_TAG_TIM(PORT.timer)]

extern void soft_serial_rx_isr();
extern void soft_serial_tx_isr();

static void soft_serial_timer_start(serial_ports_t port) {
  LL_TIM_SetCounter(TIMER.instance, 0);

  LL_TIM_ClearFlag_UPDATE(TIMER.instance);
  LL_TIM_EnableIT_UPDATE(TIMER.instance);

  LL_TIM_EnableCounter(TIMER.instance);
}

static void soft_serial_timer_stop(serial_ports_t port) {
  LL_TIM_DisableIT_UPDATE(TIMER.instance);
  LL_TIM_DisableCounter(TIMER.instance);
}

static int soft_serial_is_1wire(serial_ports_t port) {
  return DEV.tx == DEV.rx;
}

static void soft_serial_init_rx(serial_ports_t port) {
  if (DEV.rx == PIN_NONE) {
    return;
  }

  LL_GPIO_InitTypeDef gpio_init;
  gpio_init.Mode = LL_GPIO_MODE_INPUT;
  gpio_init.OutputType = LL_GPIO_OUTPUT_OPENDRAIN;
  gpio_init.Pull = LL_GPIO_PULL_UP;
  gpio_init.Speed = LL_GPIO_SPEED_FREQ_HIGH;
  gpio_pin_init(&gpio_init, DEV.rx);

  gpio_pin_set(DEV.rx);

  if (soft_serial_is_1wire(port)) {
    PORT.tx_active = false;
  }
  PORT.rx_active = true;
}

static void soft_serial_init_tx(serial_ports_t port) {
  if (DEV.tx == PIN_NONE) {
    return;
  }

  LL_GPIO_InitTypeDef gpio_init;
  gpio_init.Mode = LL_GPIO_MODE_OUTPUT;
  gpio_init.OutputType = LL_GPIO_OUTPUT_PUSHPULL;
  gpio_init.Pull = LL_GPIO_PULL_NO;
  gpio_init.Speed = LL_GPIO_SPEED_FREQ_HIGH;
  gpio_pin_init(&gpio_init, DEV.tx);

  gpio_pin_reset(DEV.tx);

  if (soft_serial_is_1wire(port)) {
    PORT.rx_active = false;
  }
  PORT.tx_active = true;
}

static void soft_serial_set_input(serial_ports_t port) {
  if (soft_serial_is_1wire(port) && !PORT.rx_active) {
    soft_serial_init_rx(port);
  }
}

static void soft_serial_set_output(serial_ports_t port) {
  if (soft_serial_is_1wire(port) && !PORT.tx_active) {
    soft_serial_init_tx(port);
  }
}

uint8_t soft_serial_init(serial_ports_t port, uint32_t baudrate, uint8_t stop_bits) {
  if (PORT.timer) {
    soft_serial_timer_stop(port);
  }

  PORT.baud = baudrate;
  PORT.stop_bits = stop_bits;

  PORT.tx_state = START_BIT;
  PORT.rx_state = START_BIT;

  soft_serial_init_tx(port);
  soft_serial_init_rx(port);

  if (!PORT.timer) {
    PORT.timer = timer_alloc(TIMER_USE_SOFT_SERIAL);
  }
  timer_up_init(TIMER_TAG_TIM(PORT.timer), 1, PWM_CLOCK_FREQ_HZ / (baudrate * BAUD_DIVIDER));
  interrupt_enable(TIMER.irq, TIMER_PRIORITY);

  return 1;
}

void soft_serial_enable_write(serial_ports_t port) {
  PORT.tx_state = START_BIT;

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
}

void soft_serial_tx_update(serial_ports_t port) {
  if (!PORT.tx_active) {
    return;
  }

  if (PORT.tx_state == START_BIT) {
    soft_serial_tx_isr();
    gpio_pin_reset(DEV.tx);
  }

  if (PORT.tx_state >= DATA_BITS && PORT.tx_state < STOP_BITS) {
    const uint8_t bit_index = (PORT.tx_state - DATA_BITS) / BAUD_DIVIDER;

    if ((PORT.tx_byte >> bit_index) & 0x01)
      gpio_pin_set(DEV.tx);
    else
      gpio_pin_reset(DEV.tx);
  }

  if (PORT.tx_state >= STOP_BITS && PORT.tx_state < (STOP_BITS + PORT.stop_bits * BAUD_DIVIDER)) {
    gpio_pin_set(DEV.tx);
  }

  if (PORT.tx_state == (STOP_BITS + PORT.stop_bits * BAUD_DIVIDER)) {
    PORT.tx_state = START_BIT;
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
    soft_serial_timer_stop(port);
    return;
  }

  if (PORT.rx_state == START_BIT) {
    if (gpio_pin_read(DEV.rx)) {
      PORT.rx_state++;
    } else {
      timeout++;
    }
    return;
  }

  if (PORT.rx_state > START_BIT && PORT.rx_state <= DATA_BITS) {
    if (!gpio_pin_read(DEV.rx)) {
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

    if (gpio_pin_read(DEV.rx)) {
      PORT.rx_byte |= (0x01 << bit_index);
    }
  }

  // should be PORT.stop_bits * BAUD_DIVIDER
  if (PORT.rx_state > (STOP_BITS + (BAUD_DIVIDER / 2)) && PORT.rx_state < (STOP_BITS + 1 * BAUD_DIVIDER)) {
    if (!gpio_pin_read(DEV.rx)) {
      PORT.rx_state = START_BIT;
      timeout = 0;
      return;
    }
  }

  PORT.rx_state++;

  // should be PORT.stop_bits * BAUD_DIVIDER
  if (PORT.rx_state == (STOP_BITS + 1 * BAUD_DIVIDER)) {
    soft_serial_rx_isr();
    PORT.rx_state = START_BIT;
    timeout = 0;
  }
}

void soft_serial_timer_irq_handler() {
  for (uint8_t port = SERIAL_SOFT_START; port < SERIAL_SOFT_MAX; port++) {
    if (PORT.timer == RESOURCE_INVALID) {
      continue;
    }

    if (!LL_TIM_IsActiveFlag_UPDATE(TIMER.instance)) {
      continue;
    }

    LL_TIM_ClearFlag_UPDATE(TIMER.instance);
    soft_serial_tx_update(port);
    soft_serial_rx_update(port);
  }
}