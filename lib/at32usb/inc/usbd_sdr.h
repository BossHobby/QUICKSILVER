/**
  **************************************************************************
  * @file     usb_sdr.h
  * @brief    usb header file
  **************************************************************************
  *                       Copyright notice & Disclaimer
  *
  * The software Board Support Package (BSP) that is made available to
  * download from Artery official website is the copyrighted work of Artery.
  * Artery authorizes customers to use, copy, and distribute the BSP
  * software and its related documentation for the purpose of design and
  * development in conjunction with Artery microcontrollers. Use of the
  * software is governed by this copyright notice and the following disclaimer.
  *
  * THIS SOFTWARE IS PROVIDED ON "AS IS" BASIS WITHOUT WARRANTIES,
  * GUARANTEES OR REPRESENTATIONS OF ANY KIND. ARTERY EXPRESSLY DISCLAIMS,
  * TO THE FULLEST EXTENT PERMITTED BY LAW, ALL EXPRESS, IMPLIED OR
  * STATUTORY OR OTHER WARRANTIES, GUARANTEES OR REPRESENTATIONS,
  * INCLUDING BUT NOT LIMITED TO, THE IMPLIED WARRANTIES OF MERCHANTABILITY,
  * FITNESS FOR A PARTICULAR PURPOSE, OR NON-INFRINGEMENT.
  *
  **************************************************************************
  */

/* define to prevent recursive inclusion -------------------------------------*/
#ifndef __USB_SDR_H
#define __USB_SDR_H

#ifdef __cplusplus
extern "C" {
#endif


/* includes ------------------------------------------------------------------*/
#include "usb_conf.h"
#include "usb_core.h"
/** @addtogroup AT32F435_437_middlewares_usbd_drivers
  * @{
  */

/** @addtogroup USBD_drivers_standard_request
  * @{
  */

/** @defgroup USBD_sdr_exported_functions
  * @{
  */


void usbd_setup_request_parse(usb_setup_type *setup, uint8_t *buf);
usb_sts_type usbd_device_request(usbd_core_type *udev);
usb_sts_type usbd_interface_request(usbd_core_type *udev);
usb_sts_type usbd_endpoint_request(usbd_core_type *udev);


/**
  * @}
  */

/**
  * @}
  */

/**
  * @}
  */
#ifdef __cplusplus
}
#endif

#endif
