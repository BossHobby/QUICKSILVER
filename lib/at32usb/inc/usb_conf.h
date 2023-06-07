/**
 **************************************************************************
 * @file     usb_conf.h
 * @brief    usb config header file
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
#ifndef __USB_CONF_H
#define __USB_CONF_H

#ifdef __cplusplus
extern "C" {
#endif

#include "at32f435_437.h"
#include "at32f435_437_usb.h"
#include "stdio.h"

/** @addtogroup AT32F437_periph_examples
 * @{
 */

/** @addtogroup 437_USB_device_vcp_loopback
 * @{
 */

/**
 * @brief enable usb device mode
 */
#define USE_OTG_DEVICE_MODE

/**
 * @brief enable usb host mode
 */
/* #define USE_OTG_HOST_MODE */

/**
 * @brief usb device mode config
 */
#ifdef USE_OTG_DEVICE_MODE
/**
 * @brief usb device mode fifo
 */
/* otg1 device fifo */
#define USBD_RX_SIZE 128
#define USBD_EP0_TX_SIZE 24
#define USBD_EP1_TX_SIZE 20
#define USBD_EP2_TX_SIZE 20
#define USBD_EP3_TX_SIZE 20
#define USBD_EP4_TX_SIZE 20
#define USBD_EP5_TX_SIZE 20
#define USBD_EP6_TX_SIZE 20
#define USBD_EP7_TX_SIZE 20

/* otg2 device fifo */
#define USBD2_RX_SIZE 128
#define USBD2_EP0_TX_SIZE 24
#define USBD2_EP1_TX_SIZE 20
#define USBD2_EP2_TX_SIZE 20
#define USBD2_EP3_TX_SIZE 20
#define USBD2_EP4_TX_SIZE 20
#define USBD2_EP5_TX_SIZE 20
#define USBD2_EP6_TX_SIZE 20
#define USBD2_EP7_TX_SIZE 20

/**
 * @brief usb endpoint max num define
 */
#ifndef USB_EPT_MAX_NUM
#define USB_EPT_MAX_NUM 8
#endif
#endif

/**
 * @brief usb host mode config
 */
#ifdef USE_OTG_HOST_MODE
#ifndef USB_HOST_CHANNEL_NUM
#define USB_HOST_CHANNEL_NUM 16
#endif

/**
 * @brief usb host mode fifo
 */
/* otg1 host fifo */
#define USBH_RX_FIFO_SIZE 128
#define USBH_NP_TX_FIFO_SIZE 96
#define USBH_P_TX_FIFO_SIZE 96

/* otg2 host fifo */
#define USBH2_RX_FIFO_SIZE 128
#define USBH2_NP_TX_FIFO_SIZE 96
#define USBH2_P_TX_FIFO_SIZE 96
#endif

/**
 * @brief usb sof output enable
 */
/* #define USB_SOF_OUTPUT_ENABLE */

/**
 * @brief usb vbus ignore, not use vbus pin
 */
#define USB_VBUS_IGNORE

/**
 * @brief usb low power wakeup handler enable
 */
/* #define USB_LOW_POWER_WAKUP */

void usb_delay_ms(uint32_t ms);
void usb_delay_us(uint32_t us);
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
