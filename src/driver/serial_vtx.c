#include "driver/serial_vtx.h"

#include "driver/serial.h"
#include "driver/serial_soft.h"
#include "driver/time.h"
#include "io/usb_configurator.h"
#include "util/ring_buffer.h"

static uint8_t tx_data[512];
static ring_buffer_t tx_buffer = {
    .buffer = tx_data,
    .head = 0,
    .tail = 0,
    .size = 512,
};

static uint8_t rx_data[512];
static ring_buffer_t rx_buffer = {
    .buffer = rx_data,
    .head = 0,
    .tail = 0,
    .size = 512,
};

serial_port_t serial_vtx = {
    .rx_buffer = &rx_buffer,
    .tx_buffer = &tx_buffer,

    .tx_done = true,
};

uint32_t vtx_last_valid_read = 0;
uint32_t vtx_last_request = 0;

bool serial_vtx_is_ready() {
  return serial_vtx.tx_done;
}

bool serial_vtx_wait_for_ready() {
  const uint32_t start = time_millis();
  while (!serial_vtx_is_ready()) {
    if ((time_millis() - start) > 100) {
      return false;
    }
    __NOP();
  }
  return true;
}

bool serial_vtx_send_data(uint8_t *data, uint32_t size) {
  if (!serial_vtx_is_ready()) {
    return false;
  }

  serial_write_bytes(&serial_vtx, data, size);

  vtx_last_request = time_millis();
  vtx_last_valid_read = time_millis();

  return true;
}

uint8_t serial_vtx_read_byte(uint8_t *data) {
  if (serial_read_bytes(&serial_vtx, data, 1) == 1) {
    vtx_last_valid_read = time_millis();
    return 1;
  }
  return 0;
}
