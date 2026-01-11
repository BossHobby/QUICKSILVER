#include "driver/serial.h"

#include <string.h>

#include "core/project.h"
#include "util/ring_buffer.h"

#ifdef USE_SERIAL

extern const usart_port_def_t usart_port_defs[SERIAL_PORT_MAX] = {
    {},
    {
        .channel_index = 1,
        .channel = (usart_dev_t *)1, // Simulated USART1
        .rcc = 0,
    },
    {
        .channel_index = 2,
        .channel = (usart_dev_t *)2, // Simulated USART2
        .rcc = 0,
    },
    {
        .channel_index = 3,
        .channel = (usart_dev_t *)3, // Simulated USART3
        .rcc = 0,
    },
};

void serial_hard_init(serial_port_t *serial, serial_port_config_t config, bool swap) {
}

bool serial_write_bytes(serial_port_t *port, const uint8_t *data, const uint32_t count) {
  if (!port || !data || count == 0) {
    return false;
  }

  // In a real simulator, this would write to a virtual serial port
  // For now, just store data in the TX buffer
  uint32_t written = ring_buffer_write_multi(port->tx_buffer, data, count);
  return written == count;
}

#endif // USE_SERIAL