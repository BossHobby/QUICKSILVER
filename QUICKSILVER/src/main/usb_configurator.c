#include "usb_configurator.h"

#include "debug.h"
#include "drv_time.h"
#include "drv_usb.h"
#include "profile.h"
#include "project.h"

#ifdef F405

#include "stm32f4xx.h"

#ifdef DEBUG
extern debug_type debug;
extern float rx[];

#ifdef RX_FRSKY
#include "rx.h"
extern frsky_bind_data frsky_bind;
#endif
#endif

extern profile_t profile;
uint8_t encode_buffer[1024];
uint8_t decode_buffer[1024];

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
    usb_serial_print("SYSTEM RESET\r\n");
    delay(50 * 1000);
    systemResetToBootloader();
    break;
#ifdef USE_SERIAL_4WAY_BLHELI_INTERFACE
  case USB_MAGIC_MSP:
    usb_process_msp();
    break;
#endif
  case USB_MAGIC_QUIC:
    usb_process_quic();
    break;
#ifdef DEBUG
  case 'D':
    usb_serial_printf("adcfilt: %f\r\n", debug.adcfilt);
    usb_serial_printf("looptime: %f\r\n", debug.timefilt);
    usb_serial_printf("cpu_load: %f\r\n", debug.cpu_load * 100);
    usb_serial_printf("max_cpu_load: %f\r\n", debug.max_cpu_load * 100);
    usb_serial_printf("RX: %f %f %f %f\r\n", rx[0], rx[1], rx[2], rx[3]);
#ifdef RX_FRSKY
    usb_serial_print("FRSKY_BIND\r\n");
    usb_serial_printf("idx: %d\r\n", frsky_bind.idx);
    usb_serial_printf("offset: %d\r\n", frsky_bind.offset);
    usb_serial_printf("tx_id: 0x%x%x\r\n", frsky_bind.tx_id[0], frsky_bind.tx_id[1]);
    usb_serial_printf("hop_data:");
    for (uint32_t i = 0; i < 50; i++) {
      usb_serial_printf(" %x, ", frsky_bind.hop_data[i]);
    }
    usb_serial_print("\r\n");
#endif
    break;
#endif
  case 'P':
    if (profile.rate.mode == RATE_MODE_BETAFLIGHT) {
      usb_serial_printf("betaflight ratemode\r\n");
      usb_serial_printf("rc_rate: %f %f %f\r\n",
                        profile.rate.betaflight.rc_rate.roll,
                        profile.rate.betaflight.rc_rate.pitch,
                        profile.rate.betaflight.rc_rate.yaw);
      usb_serial_printf("super_rate: %f %f %f\r\n",
                        profile.rate.betaflight.super_rate.roll,
                        profile.rate.betaflight.super_rate.pitch,
                        profile.rate.betaflight.super_rate.yaw);
    } else {
      usb_serial_printf("silverware ratemode\r\n");
      usb_serial_printf("max_rate: %f %f %f\r\n",
                        profile.rate.silverware.max_rate.roll,
                        profile.rate.silverware.max_rate.pitch,
                        profile.rate.silverware.max_rate.yaw);
    }

    usb_serial_printf("pidkp: %f %f %f\r\n",
                      profile_current_pid_rates()->kp.axis[0],
                      profile_current_pid_rates()->kp.axis[1],
                      profile_current_pid_rates()->kp.axis[2]);
    usb_serial_printf("pidki: %f %f %f\r\n",
                      profile_current_pid_rates()->ki.axis[0],
                      profile_current_pid_rates()->ki.axis[1],
                      profile_current_pid_rates()->ki.axis[2]);
    usb_serial_printf("pidkd: %f %f %f\r\n",
                      profile_current_pid_rates()->kd.axis[0],
                      profile_current_pid_rates()->kd.axis[1],
                      profile_current_pid_rates()->kd.axis[2]);
    break;
  }
}
#pragma GCC diagnostic pop

#endif