#include "usb_configurator.h"

#include "debug.h"
#include "drv_usb.h"
#include "flight/control.h"
#include "io/msp.h"
#include "profile.h"
#include "project.h"
#include "reset.h"
#include "util/util.h"

extern profile_t profile;
uint8_t encode_buffer[USB_BUFFER_SIZE];
uint8_t decode_buffer[USB_BUFFER_SIZE];

void usb_msp_send(uint8_t direction, uint8_t code, uint8_t *data, uint8_t len);

static msp_t msp = {
    .buffer = decode_buffer,
    .buffer_size = USB_BUFFER_SIZE,
    .read_offset = 0,
    .write_offset = 0,
    .send = usb_msp_send,
};

void usb_msp_send(uint8_t direction, uint8_t code, uint8_t *data, uint8_t len) {
  const uint8_t size = len + MSP_HEADER_LEN + 1;

  static uint8_t frame[64];

  frame[0] = '$';
  frame[1] = 'M';
  frame[2] = '>';
  frame[3] = len;
  frame[4] = code;

  for (uint8_t i = 0; i < len; i++) {
    frame[i + MSP_HEADER_LEN] = data[i];
  }

  uint8_t chksum = len;
  for (uint8_t i = 4; i < (size - 1); i++) {
    chksum ^= frame[i];
  }
  frame[len + MSP_HEADER_LEN] = chksum;

  usb_serial_write(frame, size);
}

// double promition in the following is intended
#pragma GCC diagnostic ignored "-Wdouble-promotion"
// This function will be where all usb send/receive coms live
void usb_configurator() {
  uint8_t magic = 0;
  if (usb_serial_read(&magic, 1) != 1) {
    return;
  }

  switch (magic) {
  case USB_MAGIC_REBOOT:
    //  The following bits will reboot to DFU upon receiving 'R' (which is sent by BF configurator)
    system_reset_to_bootloader();
    break;
  case USB_MAGIC_SOFT_REBOOT:
    system_reset();
    break;
  case USB_MAGIC_MSP: {
    msp_push_byte(&msp, magic);

    while (true) {
      msp_status_t status = msp_process_serial(&msp);
      if (status == MSP_EOF) {
        msp_push_byte(&msp, usb_serial_read_byte());
      } else {
        break;
      }
    }
    break;
  }
  case USB_MAGIC_QUIC:
    usb_process_quic();
    break;
  }

  // this will block and handle all usb traffic while active
  reset_looptime();
}
#pragma GCC diagnostic pop