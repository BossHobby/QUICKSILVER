#include "usb_configurator.h"

#include "control.h"
#include "debug.h"
#include "drv_usb.h"
#include "profile.h"
#include "project.h"
#include "util.h"

#ifdef F4

#include "stm32f4xx.h"

#ifdef DEBUG
#ifdef RX_FRSKY
#include "rx.h"
#include "rx_frsky.h"
extern frsky_bind_data frsky_bind;
#endif
#endif

extern profile_t profile;
uint8_t encode_buffer[USB_BUFFER_SIZE];
uint8_t decode_buffer[USB_BUFFER_SIZE];

void systemResetToBootloader(void) {
  *((uint32_t *)0x2001FFFC) = 0xDEADBEEF; // 128KB SRAM STM32F4XX
  NVIC_SystemReset();
}

// double promition in the following is intended
#pragma GCC diagnostic ignored "-Wdouble-promotion"
//This function will be where all usb send/receive coms live
void usb_configurator(void) {
  static uint8_t magic = 0;
  if (usb_serial_read(&magic, 1) != 1) {
    return;
  }

  switch (magic) {
  case USB_MAGIC_REBOOT:
    //  The following bits will reboot to DFU upon receiving 'R' (which is sent by BF configurator)
    systemResetToBootloader();
    break;
  case USB_MAGIC_SOFT_REBOOT:
    NVIC_SystemReset();
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

#endif