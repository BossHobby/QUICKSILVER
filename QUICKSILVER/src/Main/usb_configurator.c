#include "drv_usb.h"

#include "stm32f4xx.h"

void systemResetToBootloader(void) {
  *((uint32_t *)0x2001FFFC) = 0xDEADBEEF; // 128KB SRAM STM32F4XX
  NVIC_SystemReset();
}

//This function will be where all usb send/receive coms live
void usb_configurator(uint8_t *data, uint32_t len) {
  switch (data[0]) {
  case 'R':
    //  The following bits will reboot to DFU upon receiving 'R' (which is sent by BF configurator)
    systemResetToBootloader();
    break;
  default:
    usb_serial_write(data, len);
    break;
  }
}