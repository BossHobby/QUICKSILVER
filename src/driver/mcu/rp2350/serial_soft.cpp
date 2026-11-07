#include "driver/serial_soft.h"

#include <hardware/pio.h>

#include "driver/interrupt.h"

#ifdef USE_SOFT_SERIAL

#include "serial_soft.pio.h"

// Soft serial ports run on PIO state machines, one per direction, claimed on
// first use from whichever block has room and reaches the pin. Interrupts
// move bytes between the FIFOs and the port's ring buffers like a UART ISR.

static constexpr uint32_t SOFT_SERIAL_CYCLES_PER_BIT = 8;
// One idle frame (ten bits) at two cycles per loop of the RX program.
static constexpr uint32_t SOFT_SERIAL_IDLE_LOOPS = 10 * SOFT_SERIAL_CYCLES_PER_BIT / 2;

typedef enum {
  SOFT_SERIAL_TX,
  SOFT_SERIAL_RX,
  SOFT_SERIAL_PROGRAMS,
} soft_serial_program_t;

static const pio_program_t *const soft_serial_programs[SOFT_SERIAL_PROGRAMS] = {
    &serial_soft_tx_program,
    &serial_soft_rx_program,
};

typedef struct {
  PIO pio; // nullptr until claimed
  uint8_t sm;
  uint8_t offset;
} soft_serial_sm_t;

typedef struct {
  soft_serial_sm_t sms[SOFT_SERIAL_PROGRAMS];
  bool half_duplex;
} soft_serial_port_t;

static soft_serial_port_t soft_serial_ports[SERIAL_SOFT_COUNT];
// Each program is loaded once per block.
static int8_t soft_serial_offsets[NUM_PIOS][SOFT_SERIAL_PROGRAMS] = {
    {-1, -1},
    {-1, -1},
    {-1, -1},
};

#define PORT soft_serial_ports[port - SERIAL_SOFT_START]

static const IRQn_Type soft_serial_irqs[NUM_PIOS] = {
    PIO0_IRQ_0_IRQn,
    PIO1_IRQ_0_IRQn,
    PIO2_IRQ_0_IRQn,
};

static bool soft_serial_pio_unused(PIO pio) {
  for (uint32_t sm = 0; sm < NUM_PIO_STATE_MACHINES; sm++) {
    if (pio_sm_is_claimed(pio, sm))
      return false;
  }
  return true;
}

// PIO0 serves DShot first, so look at the other blocks before it.
static bool soft_serial_claim(soft_serial_sm_t *sm, soft_serial_program_t program, uint32_t pin) {
  if (sm->pio)
    return true;

  static const PIO candidates[] = {pio1, pio2, pio0};
  for (PIO pio : candidates) {
    // A block reaches 32 consecutive pins, from GPIO0 or GPIO16; only an
    // unused block may move its window.
    const uint32_t base = pio_get_gpio_base(pio);
    if (pin < base || pin >= base + 32) {
      if (!soft_serial_pio_unused(pio) || pio_set_gpio_base(pio, pin >= 32 ? 16 : 0) != PICO_OK)
        continue;
    }

    const int32_t index = pio_claim_unused_sm(pio, false);
    if (index < 0)
      continue;

    int8_t *offset = &soft_serial_offsets[pio_get_index(pio)][program];
    if (*offset < 0) {
      if (!pio_can_add_program(pio, soft_serial_programs[program])) {
        pio_sm_unclaim(pio, index);
        continue;
      }
      *offset = pio_add_program(pio, soft_serial_programs[program]);
    }

    sm->pio = pio;
    sm->sm = index;
    sm->offset = *offset;
    return true;
  }
  return false;
}

static void soft_serial_load_y(const soft_serial_sm_t *sm, uint32_t value) {
  pio_sm_put(sm->pio, sm->sm, value);
  pio_sm_exec(sm->pio, sm->sm, pio_encode_pull(false, true));
  pio_sm_exec(sm->pio, sm->sm, pio_encode_mov(pio_y, pio_osr));
}

static void soft_serial_init_tx(serial_ports_t port, const soft_serial_sm_t *sm, uint32_t pin, const serial_port_config_t *config) {
  pio_gpio_init(sm->pio, pin);
  gpio_set_pulls(pin, true, false);
  gpio_set_outover(pin, config->invert ? GPIO_OVERRIDE_INVERT : GPIO_OVERRIDE_NORMAL);

  pio_sm_config cfg = serial_soft_tx_program_get_default_config(sm->offset);
  sm_config_set_set_pins(&cfg, pin, 1);
  sm_config_set_out_pins(&cfg, pin, 1);
  sm_config_set_out_shift(&cfg, true, false, 32);
  sm_config_set_mov_status(&cfg, STATUS_TX_LESSTHAN, 1);
  sm_config_set_clkdiv(&cfg, (float)SYS_CLOCK_FREQ_HZ / (SOFT_SERIAL_CYCLES_PER_BIT * config->baudrate));
  pio_sm_init(sm->pio, sm->sm, sm->offset, &cfg);

  soft_serial_load_y(sm, PORT.half_duplex ? 1 : 0);
  pio_sm_exec(sm->pio, sm->sm, pio_encode_set(pio_pins, 1));
  pio_sm_exec(sm->pio, sm->sm, pio_encode_set(pio_pindirs, PORT.half_duplex ? 0 : 1));

  pio_set_irqn_source_enabled(sm->pio, 0, static_cast<pio_interrupt_source_t>(pis_interrupt0 + sm->sm), true);
  pio_sm_set_enabled(sm->pio, sm->sm, true);
}

static void soft_serial_init_rx(serial_ports_t port, const soft_serial_sm_t *sm, uint32_t pin, const serial_port_config_t *config) {
  if (!PORT.half_duplex)
    pio_gpio_init(sm->pio, pin);
  gpio_set_pulls(pin, true, false);
  gpio_set_inover(pin, config->invert ? GPIO_OVERRIDE_INVERT : GPIO_OVERRIDE_NORMAL);

  pio_sm_config cfg = serial_soft_rx_program_get_default_config(sm->offset);
  sm_config_set_in_pins(&cfg, pin);
  sm_config_set_jmp_pin(&cfg, pin);
  sm_config_set_in_shift(&cfg, true, true, 8);
  sm_config_set_clkdiv(&cfg, (float)SYS_CLOCK_FREQ_HZ / (SOFT_SERIAL_CYCLES_PER_BIT * config->baudrate));
  pio_sm_init(sm->pio, sm->sm, sm->offset, &cfg);

  soft_serial_load_y(sm, SOFT_SERIAL_IDLE_LOOPS);

  pio_set_irqn_source_enabled(sm->pio, 0, static_cast<pio_interrupt_source_t>(pis_sm0_rx_fifo_not_empty + sm->sm), true);
  pio_set_irqn_source_enabled(sm->pio, 0, static_cast<pio_interrupt_source_t>(pis_interrupt0 + sm->sm), true);
  pio_sm_set_enabled(sm->pio, sm->sm, true);
}

uint8_t soft_serial_init(serial_port_config_t config) {
  const serial_ports_t port = config.port;
  const target_serial_port_t *dev = &target.serial_soft_ports[port - SERIAL_SOFT_START];

  PORT.half_duplex = config.half_duplex || dev->tx == dev->rx;
  const gpio_pins_t tx = dev->tx;
  const gpio_pins_t rx = PORT.half_duplex ? dev->tx : dev->rx;

  for (const soft_serial_sm_t &sm : PORT.sms) {
    if (sm.pio)
      pio_sm_set_enabled(sm.pio, sm.sm, false);
  }

  if (tx != PIN_NONE && (config.direction & SERIAL_DIR_TX)) {
    soft_serial_sm_t *sm = &PORT.sms[SOFT_SERIAL_TX];
    const uint32_t pin = gpio_pin_defs[tx].pin_index;
    if (!soft_serial_claim(sm, SOFT_SERIAL_TX, pin))
      return 0;
    soft_serial_init_tx(port, sm, pin, &config);
    interrupt_enable(soft_serial_irqs[pio_get_index(sm->pio)], UART_PRIORITY);
  }
  if (rx != PIN_NONE && (config.direction & SERIAL_DIR_RX)) {
    soft_serial_sm_t *sm = &PORT.sms[SOFT_SERIAL_RX];
    const uint32_t pin = gpio_pin_defs[rx].pin_index;
    if (!soft_serial_claim(sm, SOFT_SERIAL_RX, pin))
      return 0;
    soft_serial_init_rx(port, sm, pin, &config);
    interrupt_enable(soft_serial_irqs[pio_get_index(sm->pio)], UART_PRIORITY);
  }

  serial_ports[serial_port_slot(port)]->tx_done = true;
  return 1;
}

// Starts draining the TX ring; a half-duplex port stops listening meanwhile.
void soft_serial_enable_write(serial_ports_t port) {
  const soft_serial_sm_t *tx = &PORT.sms[SOFT_SERIAL_TX];
  const soft_serial_sm_t *rx = &PORT.sms[SOFT_SERIAL_RX];
  if (!tx->pio)
    return;
  if (PORT.half_duplex && rx->pio)
    pio_sm_set_enabled(rx->pio, rx->sm, false);
  pio_set_irqn_source_enabled(tx->pio, 0, static_cast<pio_interrupt_source_t>(pis_sm0_tx_fifo_not_full + tx->sm), true);
}

// Listens again on a half-duplex port once the transmission ended.
void soft_serial_enable_read(serial_ports_t port) {
  const soft_serial_sm_t *rx = &PORT.sms[SOFT_SERIAL_RX];
  if (!PORT.half_duplex || !rx->pio)
    return;
  pio_sm_clear_fifos(rx->pio, rx->sm);
  pio_sm_restart(rx->pio, rx->sm);
  pio_sm_exec(rx->pio, rx->sm, pio_encode_jmp(rx->offset));
  pio_sm_set_enabled(rx->pio, rx->sm, true);
}

static void soft_serial_handle_rx(serial_ports_t port, serial_port_t *serial) {
  const soft_serial_sm_t *sm = &PORT.sms[SOFT_SERIAL_RX];
  while (!pio_sm_is_rx_fifo_empty(sm->pio, sm->sm))
    ring_buffer_write(serial->rx_buffer, pio_sm_get(sm->pio, sm->sm) >> 24);

  // The line went idle after a burst.
  if (pio_interrupt_get(sm->pio, sm->sm)) {
    pio_interrupt_clear(sm->pio, sm->sm);
    serial_rx_notify_from_isr(serial);
  }
}

static void soft_serial_handle_tx(serial_ports_t port, serial_port_t *serial) {
  const soft_serial_sm_t *sm = &PORT.sms[SOFT_SERIAL_TX];

  uint8_t data = 0;
  while (!pio_sm_is_tx_fifo_full(sm->pio, sm->sm) && ring_buffer_read(serial->tx_buffer, &data))
    pio_sm_put(sm->pio, sm->sm, data);
  if (ring_buffer_available(serial->tx_buffer) == 0)
    pio_set_irqn_source_enabled(sm->pio, 0, static_cast<pio_interrupt_source_t>(pis_sm0_tx_fifo_not_full + sm->sm), false);

  // The FIFO ran dry after a stop bit.
  if (pio_interrupt_get(sm->pio, sm->sm)) {
    pio_interrupt_clear(sm->pio, sm->sm);
    if (ring_buffer_available(serial->tx_buffer) == 0 && pio_sm_is_tx_fifo_empty(sm->pio, sm->sm)) {
      serial->tx_done = true;
      soft_serial_enable_read(port);
      serial_tx_notify_from_isr(serial);
    }
  }
}

static void soft_serial_irq_handler(PIO pio) {
  for (uint32_t i = SERIAL_SOFT_START + 1; i < SERIAL_SOFT_MAX; i++) {
    const auto port = static_cast<serial_ports_t>(i);
    serial_port_t *serial = serial_ports[serial_port_slot(port)];
    if (!serial)
      continue;
    if (PORT.sms[SOFT_SERIAL_RX].pio == pio)
      soft_serial_handle_rx(port, serial);
    if (PORT.sms[SOFT_SERIAL_TX].pio == pio)
      soft_serial_handle_tx(port, serial);
  }
}

extern "C" {
void PIO0_IRQ_0_Handler() { soft_serial_irq_handler(pio0); }
void PIO1_IRQ_0_Handler() { soft_serial_irq_handler(pio1); }
void PIO2_IRQ_0_Handler() { soft_serial_irq_handler(pio2); }
}

#endif
