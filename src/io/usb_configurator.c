#include "io/usb_configurator.h"

#include <stdarg.h>
#include <stdio.h>
#include <string.h>

#include "core/debug.h"
#include "core/looptime.h"
#include "core/profile.h"
#include "core/project.h"
#include "core/reset.h"
#include "driver/serial.h"
#include "driver/usb.h"
#include "flight/control.h"
#include "io/msp.h"
#include "io/quic.h"
#include "util/crc.h"
#include "util/ring_buffer.h"
#include "util/util.h"

void usb_msp_send(msp_magic_t magic, uint8_t direction, uint16_t cmd, const uint8_t *data, uint16_t len) {

  if (magic == MSP2_MAGIC) {
    const uint8_t size = len + MSP2_HEADER_LEN + 1;

    uint8_t frame[size];
    frame[0] = '$';
    frame[1] = MSP2_MAGIC;
    frame[2] = '>';
    frame[3] = 0; // flag
    frame[4] = (cmd >> 0) & 0xFF;
    frame[5] = (cmd >> 8) & 0xFF;
    frame[6] = (len >> 0) & 0xFF;
    frame[7] = (len >> 8) & 0xFF;

    memcpy(frame + MSP2_HEADER_LEN, data, len);
    frame[len + MSP2_HEADER_LEN] = crc8_dvb_s2_data(0, frame + 3, len + 5);

    usb_serial_write(frame, size);
  } else {
    const uint8_t size = len + MSP_HEADER_LEN + 1;

    uint8_t frame[size];
    frame[0] = '$';
    frame[1] = MSP1_MAGIC;
    frame[2] = '>';
    frame[3] = len;
    frame[4] = cmd;

    memcpy(frame + MSP_HEADER_LEN, data, len);

    uint8_t chksum = len;
    for (uint8_t i = 4; i < (size - 1); i++) {
      chksum ^= frame[i];
    }
    frame[len + MSP_HEADER_LEN] = chksum;

    usb_serial_write(frame, size);
  }
}

void usb_quic_send(uint8_t *data, uint32_t len, void *priv) {
  usb_serial_write(data, len);
}

static quic_t quic = {
    .send = usb_quic_send,
};

void usb_quic_logf(const char *fmt, ...) {
  const uint32_t size = strlen(fmt) + 128;
  char str[size];

  memset(str, 0, size);

  va_list args;
  va_start(args, fmt);
  vsnprintf(str, size, fmt, args);
  va_end(args);

  quic_send_str(&quic, QUIC_CMD_LOG, QUIC_FLAG_NONE, str);
}

void usb_serial_passthrough(usart_ports_t port, uint32_t baudrate, uint8_t stop_bits, bool half_duplex) {
  uint8_t tx_data[512];
  ring_buffer_t tx_buffer = {
      .buffer = tx_data,
      .head = 0,
      .tail = 0,
      .size = 512,
  };

  uint8_t rx_data[512];
  ring_buffer_t rx_buffer = {
      .buffer = rx_data,
      .head = 0,
      .tail = 0,
      .size = 512,
  };

  serial_port_t serial = {
      .rx_buffer = &rx_buffer,
      .tx_buffer = &tx_buffer,
  };

  serial_disable_isr(port);

  serial_enable_rcc(port);
  serial_init(&serial, port, baudrate, stop_bits, half_duplex);

  uint8_t data[512];
  while (1) {
    while (1) {
      const uint32_t size = usb_serial_read(data, 512);
      if (size == 0) {
        break;
      }
      serial_write_bytes(&serial, data, size);
    }
    while (1) {
      const uint32_t size = serial_read_bytes(&serial, data, 512);
      if (size == 0) {
        break;
      }
      usb_serial_write(data, size);
    }
  }
}

// double promition in the following is intended
#pragma GCC diagnostic ignored "-Wdouble-promotion"
// This function will be where all usb send/receive coms live
void usb_configurator() {
  uint32_t buffer_size = 1;
  static uint8_t buffer[USB_BUFFER_SIZE];

  if (usb_serial_read(buffer, 1) != 1) {
    return;
  }

  switch (buffer[0]) {
  case USB_MAGIC_REBOOT:
    //  The following bits will reboot to DFU upon receiving 'R' (which is sent by BF configurator)
    system_reset_to_bootloader();
    break;

  case USB_MAGIC_SOFT_REBOOT:
    system_reset();
    break;

  case USB_MAGIC_MSP: {
    msp_t msp = {
        .buffer = buffer,
        .buffer_size = USB_BUFFER_SIZE,
        .buffer_offset = 1,
        .send = usb_msp_send,
        .device = MSP_DEVICE_FC,
    };

    while (true) {
      msp_status_t status = msp_process_serial(&msp, usb_serial_read_byte());
      if (status != MSP_EOF) {
        break;
      }
    }
    break;
  }
  case USB_MAGIC_QUIC: {
    while (true) {
      if (!quic_process(&quic, buffer, buffer_size)) {
        buffer[buffer_size++] = usb_serial_read_byte();
      } else {
        break;
      }
    }
    break;
  }
  }

  // this will block and handle all usb traffic while active
  looptime_reset();
}
#pragma GCC diagnostic pop