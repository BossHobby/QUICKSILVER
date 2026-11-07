#include "driver/serial.h"

#include <hardware/uart.h>

#include "driver/interrupt.h"
#include "driver/serial_soft.h"

#ifdef USE_SERIAL


extern const usart_port_def_t usart_port_defs[SERIAL_PORT_MAX] = {
    {},
    {
        .channel_index = 0,
        .channel = uart0_hw,
        .irq = UART0_IRQ_IRQn,
        .rcc = RESETS_RESET_UART0_BITS,
    },
    {
        .channel_index = 1,
        .channel = uart1_hw,
        .irq = UART1_IRQ_IRQn,
        .rcc = RESETS_RESET_UART1_BITS,
    },
};

static uart_inst_t *serial_uart(serial_ports_t port) {
  return (uart_inst_t *)usart_port_defs[port].channel;
}

// Moves queued bytes into the TX FIFO. Once the ring is empty the TX
// interrupt is not needed and tx_done is set; otherwise the FIFO is full, so
// it drains through the trigger level and raises the interrupt again.
static void serial_fill_tx(serial_port_t *serial, uart_hw_t *hw) {
  uint8_t data = 0;
  while (!(hw->fr & UART_UARTFR_TXFF_BITS) && ring_buffer_read(serial->tx_buffer, &data))
    hw->dr = data;
  if (ring_buffer_available(serial->tx_buffer) == 0) {
    hw_clear_bits(&hw->imsc, UART_UARTIMSC_TXIM_BITS);
    serial->tx_done = true;
  } else {
    hw_set_bits(&hw->imsc, UART_UARTIMSC_TXIM_BITS);
  }
}

// Interrupt-driven; there is no RX DMA to synchronise.
void serial_hard_sync_rx(serial_port_t *serial) {}

// The UART cannot swap its pins or share one for half-duplex; those need a
// PIO UART. Inversion is applied in the pad.
void serial_hard_init(serial_port_t *serial, serial_port_config_t config, bool swap) {
  const serial_ports_t port = config.port;
  const target_serial_port_t *dev = &target.serial_ports[port];
  uart_hw_t *hw = usart_port_defs[port].channel;
  uart_inst_t *uart = serial_uart(port);

  serial_disable_isr(port);

  // uart_init() cycles the block through reset and enables both directions.
  uart_init(uart, config.baudrate);
  const uint32_t stop_bits = config.stop_bits >= SERIAL_STOP_BITS_1_5 ? 2 : 1;
  uart_set_format(uart, 8, stop_bits, UART_PARITY_NONE);
  if (!(config.direction & SERIAL_DIR_RX))
    hw_clear_bits(&hw->cr, UART_UARTCR_RXE_BITS);
  if (!(config.direction & SERIAL_DIR_TX))
    hw_clear_bits(&hw->cr, UART_UARTCR_TXE_BITS);

  const gpio_override inv = config.invert ? GPIO_OVERRIDE_INVERT : GPIO_OVERRIDE_NORMAL;
  if (dev->rx != PIN_NONE)
    gpio_set_inover(gpio_pin_defs[dev->rx].pin_index, inv);
  if (dev->tx != PIN_NONE)
    gpio_set_outover(gpio_pin_defs[dev->tx].pin_index, inv);

  // Interrupt at 1/8 full in both directions (4 bytes).
  hw_write_masked(&hw->ifls,
                  (0 << UART_UARTIFLS_RXIFLSEL_LSB) | (0 << UART_UARTIFLS_TXIFLSEL_LSB),
                  UART_UARTIFLS_RXIFLSEL_BITS | UART_UARTIFLS_TXIFLSEL_BITS);
  hw->icr = UART_UARTICR_BITS;
  if (config.direction & SERIAL_DIR_RX)
    hw->imsc = UART_UARTIMSC_RXIM_BITS | UART_UARTIMSC_RTIM_BITS;
  else
    hw->imsc = 0;

  serial->tx_done = true;
  serial_enable_isr(port);
}

bool serial_hard_set_baudrate(serial_port_t *serial, uint32_t baudrate) {
  if (!serial || baudrate == 0 || serial->config.port <= SERIAL_PORT_INVALID || serial->config.port >= SERIAL_PORT_MAX) {
    return false;
  }
  uart_set_baudrate(serial_uart(serial->config.port), baudrate);
  return true;
}

bool serial_write_bytes(serial_port_t *serial, const uint8_t *data, const uint32_t size) {
  if (!serial || !data || !serial->tx_buffer || serial->config.port <= SERIAL_PORT_INVALID) {
    return false;
  }

  if (size == 0) {
    return true;
  }

  const serial_ports_t port = serial->config.port;
  const bool is_soft = serial_is_soft(port);

  uint32_t written = 0;
  while (written < size) {
    written += ring_buffer_write_multi(serial->tx_buffer, data + written, size - written);
    // The PIO TX-not-full interrupt is level-sensitive, so enabling it starts
    // the ISR draining. The UART raises TXIM only when the FIFO drains through
    // its trigger level, so an idle UART is primed here instead.
    ATOMIC_BLOCK(UART_PRIORITY) {
      serial->tx_done = false;
      if (is_soft) {
#ifdef USE_SOFT_SERIAL
        soft_serial_enable_write(port);
#endif
      } else {
        uart_hw_t *hw = usart_port_defs[port].channel;
        if (!(hw->imsc & UART_UARTIMSC_TXIM_BITS))
          serial_fill_tx(serial, hw);
      }
    }
  }

  return true;
}

static void serial_read_byte_isr(serial_port_t *serial, uart_hw_t *hw) {
  const uint32_t dr = hw->dr;
  if (dr & (UART_UARTDR_OE_BITS | UART_UARTDR_BE_BITS | UART_UARTDR_PE_BITS | UART_UARTDR_FE_BITS))
    serial->rx_error_count = serial->rx_error_count + 1;
  ring_buffer_write(serial->rx_buffer, dr & UART_UARTDR_DATA_BITS);
}

static void handle_serial_isr(serial_port_t *serial, uart_hw_t *hw) {
  const uint32_t status = hw->mis;

  if (status & UART_UARTMIS_RTMIS_BITS) {
    // The line went quiet with bytes left in the FIFO: the burst is done.
    while (!(hw->fr & UART_UARTFR_RXFE_BITS))
      serial_read_byte_isr(serial, hw);
    hw->icr = UART_UARTICR_RTIC_BITS;
    serial_rx_notify_from_isr(serial);
  } else if (status & UART_UARTMIS_RXMIS_BITS) {
    // Drain only below the trigger level: the bytes left behind raise the
    // receive timeout once the burst ends, which wakes the consumer once.
    while (hw->ris & UART_UARTRIS_RXRIS_BITS)
      serial_read_byte_isr(serial, hw);
  }

  if (status & UART_UARTMIS_TXMIS_BITS) {
    serial_fill_tx(serial, hw);
    if (serial->tx_done)
      serial_tx_notify_from_isr(serial);
  }
}

static void handle_usart_isr(serial_ports_t index) {
  uart_hw_t *hw = usart_port_defs[index].channel;
  if (serial_ports[index]) {
    handle_serial_isr(serial_ports[index], hw);
    return;
  }

  // stray serial port. disable
  hw->imsc = 0;
  hw->icr = UART_UARTICR_BITS;
  hw_clear_bits(&hw->cr, UART_UARTCR_UARTEN_BITS);
}

extern "C" {
void UART0_IRQ_Handler() { handle_usart_isr(SERIAL_PORT1); }
void UART1_IRQ_Handler() { handle_usart_isr(SERIAL_PORT2); }
}

#endif
