#include "drv_usb.h"

#include "defines.h"
#include "drv_time.h"

#ifdef F4

#include <stdarg.h>
#include <stdio.h>

#include "stm32f4xx.h"
#include "string.h"
#include "usb_conf.h"
#include "usbd_cdc_core.h"
#include "usbd_cdc_vcp.h"
#include "usbd_desc.h"
#include "usbd_usr.h"

__ALIGN_BEGIN USB_OTG_CORE_HANDLE USB_OTG_dev __ALIGN_END;

void usb_init(void) {
  USBD_Init(&USB_OTG_dev,
            USB_OTG_FS_CORE_ID,
            &USR_desc,
            &USBD_CDC_cb,
            &USR_cb);

#ifdef USB_DETECT_PIN
  //RCC_AHB1PeriphClockCmd(RCC_AHB1Periph_GPIOA, ENABLE);
  GPIO_InitTypeDef GPIO_InitStructure;
  GPIO_InitStructure.GPIO_Pin = USB_DETECT_PIN;
  GPIO_InitStructure.GPIO_Mode = GPIO_Mode_IN;
  GPIO_InitStructure.GPIO_OType = GPIO_OType_OD;
  GPIO_InitStructure.GPIO_Speed = GPIO_Speed_50MHz;
  GPIO_InitStructure.GPIO_PuPd = GPIO_PuPd_NOPULL;
  GPIO_Init(USB_DETECT_PORT, &GPIO_InitStructure);
#endif
}

uint8_t usb_detect(void) {
#ifdef USB_DETECT_PIN
  const uint8_t usb_connect = GPIO_ReadInputDataBit(USB_DETECT_PORT, USB_DETECT_PIN);
  if (usb_connect != 1) {
    // no usb connetion, bail
    return 0;
  }
#endif

  if (bDeviceState != CONFIGURED) {
    // only read if we are configured
    return 0;
  }

  return 1;
}

uint32_t usb_serial_read(uint8_t *data, uint32_t len) {
  if (data == NULL || len == 0) {
    return 0;
  }
  if (bDeviceState != CONFIGURED || CDC_Receive_BytesAvailable() == 0) {
    return 0;
  }
  return CDC_Receive_DATA(data, len);
}

uint8_t usb_serial_read_byte(void) {
  uint8_t byte = 0;
  for (uint32_t timeout = 1000; usb_serial_read(&byte, 1) != 1 && timeout; --timeout) {
    delay(10);
    __WFI();
  }
  return byte;
}

void usb_serial_write(uint8_t *data, uint32_t len) {
  if (data == NULL || len == 0) {
    return;
  }
  if (bDeviceState != CONFIGURED) {
    return;
  }

  CDC_Send_DATA(data, len);
}

void usb_serial_print(char *str) {
  usb_serial_write((uint8_t *)str, strlen(str));
}

void usb_serial_printf(const char *fmt, ...) {
  if (bDeviceState != CONFIGURED) {
    return;
  }

  const size_t size = strlen(fmt) + 128;
  char str[size];

  memset(str, 0, size);

  va_list args;
  va_start(args, fmt);
  vsnprintf(str, size, fmt, args);
  va_end(args);

  usb_serial_print(str);
}

#endif
