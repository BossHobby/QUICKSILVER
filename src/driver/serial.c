#include "driver/serial.h"

#include "driver/interrupt.h"
#include "driver/serial_soft.h"

#ifdef USE_SERIAL

serial_port_t *serial_ports[SERIAL_PORT_MAX];

extern const usart_port_def_t usart_port_defs[SERIAL_PORT_MAX];
extern void serial_hard_init(serial_port_t *serial, serial_port_config_t config, bool swap);

bool serial_is_soft(serial_ports_t port) {
  if (port < SERIAL_PORT_MAX) {
    return false;
  }
  return true;
}

const target_serial_port_t *serial_get_dev(const serial_ports_t port) {
  if (serial_is_soft(port)) {
    return &target.serial_soft_ports[port - SERIAL_SOFT_START];
  }
  return &target.serial_ports[port];
}

static const gpio_af_t *serial_hard_find_af(gpio_pins_t pin, serial_ports_t port) {
  for (uint32_t i = 0; i < GPIO_AF_MAX; i++) {
    const gpio_af_t *func = &gpio_pin_afs[i];
    if (func->pin == pin && RESOURCE_TAG_TYPE(func->tag) == RESOURCE_SERIAL && SERIAL_TAG_PORT(func->tag) == port) {
      return func;
    }
  }
  return NULL;
}

static bool serial_hard_pin_init(serial_port_t *serial, serial_port_config_t config) {
  const serial_ports_t port = config.port;
  const target_serial_port_t *dev = &target.serial_ports[port];

  bool swap = false;

  gpio_config_t gpio_init;
  gpio_init.mode = GPIO_ALTERNATE;
  gpio_init.drive = GPIO_DRIVE_HIGH;
  if (config.half_duplex) {
    if (config.half_duplex_pp) {
      gpio_init.output = GPIO_PUSHPULL;
      gpio_init.pull = GPIO_NO_PULL;
    } else {
      gpio_init.output = GPIO_OPENDRAIN;
      gpio_init.pull = GPIO_UP_PULL;
    }

    const gpio_af_t *tx_af = serial_hard_find_af(dev->tx, port);
    if (tx_af == NULL) {
      return false;
    }

    if (SERIAL_TAG_PIN(tx_af->tag) == RES_SERIAL_RX) {
      swap = true;
    }

    gpio_pin_init_af(dev->tx, gpio_init, tx_af->af);
  } else {
    gpio_init.output = GPIO_PUSHPULL;
    gpio_init.pull = GPIO_NO_PULL;

    const gpio_af_t *rx_af = serial_hard_find_af(dev->rx, port);
    const gpio_af_t *tx_af = serial_hard_find_af(dev->tx, port);
    if (tx_af == NULL || rx_af == NULL) {
      return false;
    }

    if (SERIAL_TAG_PIN(rx_af->tag) == RES_SERIAL_TX &&
        SERIAL_TAG_PIN(tx_af->tag) == RES_SERIAL_RX) {
      swap = true;
    }

    gpio_pin_init_af(dev->rx, gpio_init, rx_af->af);
    gpio_pin_init_af(dev->tx, gpio_init, tx_af->af);
  }

  return swap;
}

void serial_init(serial_port_t *serial, serial_port_config_t config) {
  const serial_ports_t port = config.port;
  if (port == SERIAL_PORT_INVALID || serial == NULL) {
    return;
  }

  const target_serial_port_t *dev = serial_get_dev(port);
  if (!target_serial_port_valid(dev)) {
    return;
  }
  serial->config = config;
  serial->tx_done = true;

  for (uint32_t i = 0; i < SERIAL_PORT_MAX; i++) {
    if (serial_ports[i] == serial) {
      serial_ports[i] = NULL;
    }
  }

  serial_ports[port] = serial;

  ring_buffer_clear(serial->rx_buffer);
  ring_buffer_clear(serial->tx_buffer);

  if (serial_is_soft(config.port)) {
    soft_serial_init(config);
  } else {
    const bool swap = serial_hard_pin_init(serial, config);
    serial_hard_init(serial, config, swap);
  }
}

void serial_enable_rcc(serial_ports_t port) {
  const rcc_reg_t reg = usart_port_defs[port].rcc;
  rcc_enable(reg);
}

void serial_enable_isr(serial_ports_t port) {
  const IRQn_Type irq = usart_port_defs[port].irq;
  interrupt_enable(irq, UART_PRIORITY);
}

void serial_disable_isr(serial_ports_t port) {
  const IRQn_Type irq = usart_port_defs[port].irq;
  interrupt_disable(irq);
}

uint32_t serial_bytes_available(serial_port_t *serial) {
  return ring_buffer_available(serial->rx_buffer);
}

uint32_t serial_bytes_free(serial_port_t *serial) {
  return ring_buffer_free(serial->tx_buffer);
}

uint32_t serial_read_bytes(serial_port_t *serial, uint8_t *data, const uint32_t size) {
  return ring_buffer_read_multi(serial->rx_buffer, data, size);
}

void soft_serial_tx_isr(serial_ports_t port) {
  serial_port_t *serial = serial_ports[port];

  uint8_t data = 0;
  if (ring_buffer_read(serial->tx_buffer, &data)) {
    soft_serial_write_byte(port, data);
  } else {
    soft_serial_enable_read(port);
    serial->tx_done = true;
  }
}

void soft_serial_rx_isr(serial_ports_t port) {
  serial_port_t *serial = serial_ports[port];

  const uint8_t data = soft_serial_read_byte(port);
  ring_buffer_write(serial->rx_buffer, data);
}

#endif
