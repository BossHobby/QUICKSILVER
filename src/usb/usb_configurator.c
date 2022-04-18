#include "usb_configurator.h"

#include "debug.h"
#include "drv_usb.h"
#include "flight/control.h"
#include "profile.h"
#include "project.h"
#include "reset.h"
#include "util/util.h"

extern profile_t profile;
uint8_t encode_buffer[USB_BUFFER_SIZE];
uint8_t decode_buffer[USB_BUFFER_SIZE];

// double promition in the following is intended
#pragma GCC diagnostic ignored "-Wdouble-promotion"
// This function will be where all usb send/receive coms live
void usb_configurator() {
  static uint8_t magic = 0;
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
  case USB_MAGIC_MSP:
    usb_process_msp();
    break;
  case USB_MAGIC_QUIC:
    usb_process_quic();
    break;
  }

  // this will block and handle all usb traffic while active
  reset_looptime();
}
#pragma GCC diagnostic pop