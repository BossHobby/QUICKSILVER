#include "drv_usb.h"

#include "defines.h"

#ifdef F405

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

#define VCP_PACKET_SIZE 64

// uint8_t data_to_send[VCP_PACKET_SIZE];    //maxed out at 64 member array - adjust as needed for sending more than one array position at a time
uint8_t data_to_receive[VCP_PACKET_SIZE]; //maxed out at 64 member array - only set to one position for the time being since all we are listening for is 0x52 for flashing

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
  GPIO_InitStructure.GPIO_OType = GPIO_OType_PP;
  GPIO_InitStructure.GPIO_Speed = GPIO_Speed_50MHz;
  GPIO_InitStructure.GPIO_PuPd = GPIO_PuPd_DOWN;
  GPIO_Init(USB_DETECT_PORT, &GPIO_InitStructure);
#endif
}

void usb_detect(void) {
#ifdef USB_DETECT_PIN
  const uint8_t usb_connect = GPIO_ReadInputDataBit(USB_DETECT_PORT, USB_DETECT_PIN);
  if (usb_connect != 1) {
    // no usb connetion, bail
    return;
  }
#endif

  if (bDeviceState != CONFIGURED) {
    // only read if we are configured
    return;
  }

  const uint32_t len = CDC_Receive_DATA(data_to_receive, VCP_PACKET_SIZE);
  if (len == 0) {
    return; // no data, bail
  }
  usb_configurator(data_to_receive, len);
}

void usb_serial_write(uint8_t *data, uint32_t len) {
  if (bDeviceState != CONFIGURED) {
    // only write if we are configured
    return;
  }
  CDC_Send_DATA(data, len);
}

void usb_serial_print(char *str) {
  if (bDeviceState != CONFIGURED) {
    // only write if we are configured
    return;
  }
  CDC_Send_DATA((uint8_t *)str, strlen(str));
}

void usb_serial_printf(const char *fmt, ...) {
  if (bDeviceState != CONFIGURED) {
    // only write if we are configured
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
