
//TODO ..... Move a good chunk of this to a drv_usb file in drv folder leaving only the yet to be created serial USB/OSD protocol and usb_configurator() under src folder

#include "defines.h"

#ifdef F405

#include "stm32f4xx.h"
#include "string.h"
#include "usb_conf.h"
#include "usbd_cdc_core.h"
#include "usbd_cdc_vcp.h"
#include "usbd_desc.h"
#include "usbd_usr.h"

void systemResetToBootloader(void) {
  *((uint32_t *)0x2001FFFC) = 0xDEADBEEF; // 128KB SRAM STM32F4XX
  NVIC_SystemReset();
}

uint8_t data_to_send[1]; //maxed out at 64 member array - adjust as needed for sending more than one array position at a time
//	 Add carriage return and line feed
//	data_to_send[6] = 0x0A;
//	data_to_send[7] = 0x0D;
uint8_t data_to_receive[1]; //maxed out at 64 member array - only set to one position for the time being since all we are listening for is 0x52 for flashing

void usb_configurator(void) { //This function will be where all usb send/receive coms live
  //  The following bits will reboot to DFU upon receiving 'R' (which is sent by BF configurator)
  CDC_Receive_DATA(data_to_receive, 1);
  if (data_to_receive[0] == 0x52) // 0x52 is the same as 'R' which also works
  {
    systemResetToBootloader();
  }
  //for testing loopback of anything typed into a serial terminal - uncomment the lines below
  //else
  //{
  //CDC_Send_DATA(data_to_receive,strlen((char *)data_to_send));
  //}
}

#ifdef USB_DETECT_PIN
void usb_detect_init(void) {
  //RCC_AHB1PeriphClockCmd(RCC_AHB1Periph_GPIOA, ENABLE);
  GPIO_InitTypeDef GPIO_InitStructure;
  GPIO_InitStructure.GPIO_Pin = USB_DETECT_PIN;
  GPIO_InitStructure.GPIO_Mode = GPIO_Mode_IN;
  GPIO_InitStructure.GPIO_OType = GPIO_OType_PP;
  GPIO_InitStructure.GPIO_Speed = GPIO_Speed_50MHz;
  GPIO_InitStructure.GPIO_PuPd = GPIO_PuPd_DOWN;
  GPIO_Init(USB_DETECT_PORT, &GPIO_InitStructure);
}

void usb_detect(void) {
  const uint8_t usb_connect = GPIO_ReadInputDataBit(USB_DETECT_PORT, USB_DETECT_PIN);
  if (usb_connect == 1) {
    usb_configurator();
  }
}
#endif

#endif
