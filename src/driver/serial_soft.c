#include "driver/serial_soft.h"

#include "driver/gpio.h"
#include "driver/interrupt.h"
#include "driver/timer.h"
#include "project.h"

#define TIMER_INSTANCE TIM4
#define TIMER_IRQN TIM4_IRQn
#define BAUD_DIVIDER 4

typedef enum {
  START_BIT = 0,
  DATA_BITS = START_BIT + BAUD_DIVIDER,
  STOP_BITS = START_BIT + BAUD_DIVIDER + BAUD_DIVIDER * 8,
} soft_serial_tx_state_t;

#define USART_PORT(chan, rx, tx)
#define SOFT_SERIAL_PORT(_index, rx, tx) \
  {                                      \
      .index = _index,                   \
      .rx_pin = rx,                      \
      .tx_pin = tx,                      \
  },

static volatile soft_serial_t soft_serial_ports[SOFT_SERIAL_PORTS_MAX - USART_PORTS_MAX] = {USART_PORTS};

#undef USART_PORT
#undef SOFT_SERIAL_PORT

#define DEV soft_serial_ports[port - USART_PORTS_MAX]

extern void soft_serial_rx_isr();
extern void soft_serial_tx_isr();

static void soft_serial_timer_start() {
  LL_TIM_SetCounter(TIMER_INSTANCE, 0);

  LL_TIM_ClearFlag_UPDATE(TIMER_INSTANCE);
  LL_TIM_EnableIT_UPDATE(TIMER_INSTANCE);

  LL_TIM_EnableCounter(TIMER_INSTANCE);
}

static void soft_serial_timer_stop() {
  LL_TIM_DisableIT_UPDATE(TIMER_INSTANCE);
  LL_TIM_DisableCounter(TIMER_INSTANCE);
}

static int soft_serial_is_1wire(usart_ports_t port) {
  return DEV.tx_pin == DEV.rx_pin;
}

static void soft_serial_init_rx(usart_ports_t port) {
  if (DEV.rx_pin == PIN_NONE) {
    return;
  }

  LL_GPIO_InitTypeDef gpio_init;
  gpio_init.Mode = LL_GPIO_MODE_INPUT;
  gpio_init.OutputType = LL_GPIO_OUTPUT_OPENDRAIN;
  gpio_init.Pull = LL_GPIO_PULL_UP;
  gpio_init.Speed = LL_GPIO_SPEED_FREQ_HIGH;
  gpio_pin_init(&gpio_init, DEV.rx_pin);

  gpio_pin_set(DEV.rx_pin);

  if (soft_serial_is_1wire(port)) {
    DEV.tx_active = false;
  }
  DEV.rx_active = true;
}

static void soft_serial_init_tx(usart_ports_t port) {
  if (DEV.tx_pin == PIN_NONE) {
    return;
  }

  LL_GPIO_InitTypeDef gpio_init;
  gpio_init.Mode = LL_GPIO_MODE_OUTPUT;
  gpio_init.OutputType = LL_GPIO_OUTPUT_PUSHPULL;
  gpio_init.Pull = LL_GPIO_PULL_NO;
  gpio_init.Speed = LL_GPIO_SPEED_FREQ_HIGH;
  gpio_pin_init(&gpio_init, DEV.tx_pin);

  gpio_pin_reset(DEV.tx_pin);

  if (soft_serial_is_1wire(port)) {
    DEV.rx_active = false;
  }
  DEV.tx_active = true;
}

static void soft_serial_set_input(usart_ports_t port) {
  if (soft_serial_is_1wire(port) && !DEV.rx_active) {
    soft_serial_init_rx(port);
  }
}

static void soft_serial_set_output(usart_ports_t port) {
  if (soft_serial_is_1wire(port) && !DEV.tx_active) {
    soft_serial_init_tx(port);
  }
}

uint8_t soft_serial_init(usart_ports_t port, uint32_t baudrate, uint8_t stop_bits) {
  soft_serial_timer_stop();

  DEV.baud = baudrate;
  DEV.stop_bits = stop_bits;

  DEV.tx_state = START_BIT;
  DEV.rx_state = START_BIT;

  soft_serial_init_tx(port);
  soft_serial_init_rx(port);

  timer_init(TIMER_INSTANCE, 1, PWM_CLOCK_FREQ_HZ / (baudrate * BAUD_DIVIDER));
  interrupt_enable(TIMER_IRQN, TIMER_PRIORITY);

  return 1;
}

void soft_serial_enable_write(usart_ports_t port) {
  DEV.tx_state = START_BIT;

  soft_serial_timer_stop();
  soft_serial_set_output(port);
  soft_serial_timer_start();
}

void soft_serial_enable_read(usart_ports_t port) {
  DEV.rx_state = START_BIT;

  soft_serial_set_input(port);
  soft_serial_timer_start();
}

uint8_t soft_serial_read_byte(usart_ports_t port) {
  return DEV.rx_byte;
}

void soft_serial_write_byte(usart_ports_t port, uint8_t byte) {
  DEV.tx_byte = byte;
}

void soft_serial_tx_update(usart_ports_t port) {
  if (!DEV.tx_active) {
    return;
  }

  if (DEV.tx_state == START_BIT) {
    soft_serial_tx_isr();
    gpio_pin_reset(DEV.tx_pin);
  }

  if (DEV.tx_state >= DATA_BITS && DEV.tx_state < STOP_BITS) {
    const uint8_t bit_index = (DEV.tx_state - DATA_BITS) / BAUD_DIVIDER;

    if ((DEV.tx_byte >> bit_index) & 0x01)
      gpio_pin_set(DEV.tx_pin);
    else
      gpio_pin_reset(DEV.tx_pin);
  }

  if (DEV.tx_state >= STOP_BITS && DEV.tx_state < (STOP_BITS + DEV.stop_bits * BAUD_DIVIDER)) {
    gpio_pin_set(DEV.tx_pin);
  }

  if (DEV.tx_state == (STOP_BITS + DEV.stop_bits * BAUD_DIVIDER)) {
    DEV.tx_state = START_BIT;
    return;
  }

  DEV.tx_state++;
}

void soft_serial_rx_update(usart_ports_t port) {
  if (!DEV.rx_active) {
    return;
  }

  static uint32_t timeout = 0;
  if (timeout == 100000) {
    DEV.rx_state = START_BIT;
    timeout = 0;
    soft_serial_timer_stop();
    return;
  }

  if (DEV.rx_state == START_BIT) {
    if (gpio_pin_read(DEV.rx_pin)) {
      DEV.rx_state++;
    } else {
      timeout++;
    }
    return;
  }

  if (DEV.rx_state > START_BIT && DEV.rx_state <= DATA_BITS) {
    if (!gpio_pin_read(DEV.rx_pin)) {
      DEV.rx_byte = 0;
      DEV.rx_state++;
      timeout = 0;
    } else {
      timeout++;
    }
    return;
  }

  if (DEV.rx_state > DATA_BITS && DEV.rx_state <= STOP_BITS && (DEV.rx_state % BAUD_DIVIDER) == 2) {
    const uint8_t bit_index = (DEV.rx_state - DATA_BITS) / BAUD_DIVIDER;

    if (gpio_pin_read(DEV.rx_pin)) {
      DEV.rx_byte |= (0x01 << bit_index);
    }
  }

  // should be DEV.stop_bits * BAUD_DIVIDER
  if (DEV.rx_state > (STOP_BITS + (BAUD_DIVIDER / 2)) && DEV.rx_state < (STOP_BITS + 1 * BAUD_DIVIDER)) {
    if (!gpio_pin_read(DEV.rx_pin)) {
      DEV.rx_state = START_BIT;
      timeout = 0;
      return;
    }
  }

  DEV.rx_state++;

  // should be DEV.stop_bits * BAUD_DIVIDER
  if (DEV.rx_state == (STOP_BITS + 1 * BAUD_DIVIDER)) {
    soft_serial_rx_isr();
    DEV.rx_state = START_BIT;
    timeout = 0;
  }
}

void TIM4_IRQHandler() {
  if (LL_TIM_IsActiveFlag_UPDATE(TIMER_INSTANCE)) {
    for (uint8_t port = USART_PORTS_MAX; port < SOFT_SERIAL_PORTS_MAX; port++) {
      soft_serial_tx_update(port);
      soft_serial_rx_update(port);
    }
    LL_TIM_ClearFlag_UPDATE(TIMER_INSTANCE);
  }
}