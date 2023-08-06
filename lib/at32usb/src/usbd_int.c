/**
  **************************************************************************
  * @file     usbd_int.c
  * @brief    usb interrupt request
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
#include "usbd_int.h"

/** @addtogroup AT32F435_437_middlewares_usbd_drivers
  * @{
  */

/** @defgroup USBD_drivers_interrupt
  * @brief usb device interrupt
  * @{
  */

/** @defgroup USBD_int_private_functions
  * @{
  */

/**
  * @brief  usb device interrput request handler.
  * @param  otgdev: to the structure of otg_core_type
  * @retval none
  */
void usbd_irq_handler(otg_core_type *otgdev)
{
  otg_global_type *usbx = otgdev->usb_reg;
  usbd_core_type *udev = &otgdev->dev;
  uint32_t intsts = usb_global_get_all_interrupt(usbx);

  /* check current device mode */
  if(usbx->gintsts_bit.curmode == 0)
  {
    /* mode mismatch interrupt */
    if(intsts & USB_OTG_MODEMIS_FLAG)
    {
      usb_global_clear_interrupt(usbx, USB_OTG_MODEMIS_FLAG);
    }

    /* in endpoint interrupt */
    if(intsts & USB_OTG_IEPT_FLAG)
    {
      usbd_inept_handler(udev);
    }

    /* out endpoint interrupt */
    if(intsts & USB_OTG_OEPT_FLAG)
    {
      usbd_outept_handler(udev);
    }

    /* usb reset interrupt */
    if(intsts & USB_OTG_USBRST_FLAG)
    {
      usbd_reset_handler(udev);
      usb_global_clear_interrupt(usbx, USB_OTG_USBRST_FLAG);
    }

    /* sof interrupt */
    if(intsts & USB_OTG_SOF_FLAG)
    {
      usbd_sof_handler(udev);
      usb_global_clear_interrupt(usbx, USB_OTG_SOF_FLAG);
    }

    /* enumeration done interrupt */
    if(intsts & USB_OTG_ENUMDONE_FLAG)
    {
      usbd_enumdone_handler(udev);
      usb_global_clear_interrupt(usbx, USB_OTG_ENUMDONE_FLAG);
    }

    /* rx non-empty interrupt, indicates that there is at least one
       data packet pending to be read in rx fifo */
    if(intsts & USB_OTG_RXFLVL_FLAG)
    {
      usbd_rxflvl_handler(udev);
    }

    /* incomplete isochronous in transfer interrupt */
    if(intsts & USB_OTG_INCOMISOIN_FLAG)
    {
      usbd_incomisioin_handler(udev);
      usb_global_clear_interrupt(usbx, USB_OTG_INCOMISOIN_FLAG);
    }
 #ifndef USB_VBUS_IGNORE
    /* disconnect detected interrupt  */
    if(intsts & USB_OTG_OTGINT_FLAG)
    {
      uint32_t tmp = udev->usb_reg->gotgint;
      if(udev->usb_reg->gotgint_bit.sesenddet)
        usbd_discon_handler(udev);
      udev->usb_reg->gotgint = tmp;
      usb_global_clear_interrupt(usbx, USB_OTG_OTGINT_FLAG);
    }
#endif
    /* incomplete isochronous out transfer interrupt */
    if(intsts & USB_OTG_INCOMPIP_INCOMPISOOUT_FLAG)
    {
      usbd_incomisoout_handler(udev);
      usb_global_clear_interrupt(usbx, USB_OTG_INCOMPIP_INCOMPISOOUT_FLAG);
    }

    /* resume/remote wakeup interrupt */
    if(intsts & USB_OTG_WKUP_FLAG)
    {
      usbd_wakeup_handler(udev);
      usb_global_clear_interrupt(usbx, USB_OTG_WKUP_FLAG);
    }

    /* usb suspend interrupt */
    if(intsts & USB_OTG_USBSUSP_FLAG)
    {
      usbd_suspend_handler(udev);
      usb_global_clear_interrupt(usbx, USB_OTG_USBSUSP_FLAG);
    }
  }
}

/**
  * @brief  usb write tx fifo.
  * @param  udev: to the structure of usbd_core_type
  * @param  ept_num: endpoint number
  * @retval none
  */
void usb_write_empty_txfifo(usbd_core_type *udev, uint32_t ept_num)
{
  otg_global_type *usbx = udev->usb_reg;
  usb_ept_info *ept_info = &udev->ept_in[ept_num];
  uint32_t length = ept_info->total_len - ept_info->trans_len;
  uint32_t wlen = 0;

  if(length > ept_info->maxpacket)
  {
    length = ept_info->maxpacket;
  }
  wlen = (length + 3) / 4;

  while((USB_INEPT(usbx, ept_num)->dtxfsts & USB_OTG_DTXFSTS_INEPTFSAV) > wlen &&
    (ept_info->trans_len < ept_info->total_len) && (ept_info->total_len != 0))
  {
    length = ept_info->total_len - ept_info->trans_len;
    if(length > ept_info->maxpacket)
    {
      length = ept_info->maxpacket;
    }
    wlen = (length + 3) / 4;
    usb_write_packet(usbx, ept_info->trans_buf, ept_num, length);

    ept_info->trans_buf += length;
    ept_info->trans_len += length;

  }
  if(length <= 0)
  {
    OTG_DEVICE(usbx)->diepempmsk &= ~(0x1 << ept_num);
  }
}


/**
  * @brief  usb in endpoint handler
  * @param  udev: to the structure of usbd_core_type
  * @retval none
  */
void usbd_inept_handler(usbd_core_type *udev)
{
  otg_global_type *usbx = udev->usb_reg;
  uint32_t ept_num = 0, ept_int;
  uint32_t intsts;

  /*get all endpoint interrut */
  intsts = usb_get_all_in_interrupt(usbx);
  while(intsts)
  {
    if(intsts & 0x1)
    {
      /* get endpoint interrupt flag */
      ept_int = usb_ept_in_interrupt(usbx, ept_num);

      /* transfer completed interrupt */
      if(ept_int & USB_OTG_DIEPINT_XFERC_FLAG)
      {
        OTG_DEVICE(usbx)->diepempmsk &= ~(1 << ept_num);
        usb_ept_in_clear(usbx, ept_num , USB_OTG_DIEPINT_XFERC_FLAG);
        usbd_core_in_handler(udev, ept_num);
      }

      /* timeout condition interrupt */
      if(ept_int & USB_OTG_DIEPINT_TIMEOUT_FLAG)
      {
        usb_ept_in_clear(usbx, ept_num , USB_OTG_DIEPINT_TIMEOUT_FLAG);
      }

      /* in token received when tx fifo is empty */
      if(ept_int & USB_OTG_DIEPINT_INTKNTXFEMP_FLAG)
      {
        usb_ept_in_clear(usbx, ept_num , USB_OTG_DIEPINT_INTKNTXFEMP_FLAG);
      }

      /* in endpoint nak effective */
      if(ept_int & USB_OTG_DIEPINT_INEPTNAK_FLAG)
      {
        usb_ept_in_clear(usbx, ept_num , USB_OTG_DIEPINT_INEPTNAK_FLAG);
      }

      /* endpoint disable interrupt */
      if(ept_int & USB_OTG_DIEPINT_EPTDISD_FLAG)
      {
        usb_ept_in_clear(usbx, ept_num , USB_OTG_DIEPINT_EPTDISD_FLAG);
      }

      /* transmit fifo empty interrupt */
      if(ept_int & USB_OTG_DIEPINT_TXFEMP_FLAG)
      {
        usb_write_empty_txfifo(udev, ept_num);
      }
    }
    ept_num ++;
    intsts >>= 1;
  }
}

/**
  * @brief  usb out endpoint handler
  * @param  udev: to the structure of usbd_core_type
  * @retval none
  */
void usbd_outept_handler(usbd_core_type *udev)
{
  otg_global_type *usbx = udev->usb_reg;
  uint32_t ept_num = 0, ept_int;
  uint32_t intsts;

  /* get all out endpoint interrupt */
  intsts = usb_get_all_out_interrupt(usbx);

  while(intsts)
  {
    if(intsts & 0x1)
    {
      /* get out endpoint interrupt */
      ept_int = usb_ept_out_interrupt(usbx, ept_num);

      /* transfer completed interrupt */
      if(ept_int & USB_OTG_DOEPINT_XFERC_FLAG)
      {
        usb_ept_out_clear(usbx, ept_num , USB_OTG_DOEPINT_XFERC_FLAG);
        usbd_core_out_handler(udev, ept_num);
      }

      /* setup phase done interrupt */
      if(ept_int & USB_OTG_DOEPINT_SETUP_FLAG)
      {
        usb_ept_out_clear(usbx, ept_num , USB_OTG_DOEPINT_SETUP_FLAG);
        usbd_core_setup_handler(udev, ept_num);
        if(udev->device_addr != 0)
        {
          OTG_DEVICE(udev->usb_reg)->dcfg_bit.devaddr = udev->device_addr;
          udev->device_addr = 0;
        }
      }

      /* endpoint disable interrupt */
      if(ept_int & USB_OTG_DOEPINT_OUTTEPD_FLAG)
      {
        usb_ept_out_clear(usbx, ept_num , USB_OTG_DOEPINT_OUTTEPD_FLAG);
      }
    }
    ept_num ++;
    intsts >>= 1;
  }
}

/**
  * @brief  usb enumeration done handler
  * @param  udev: to the structure of usbd_core_type
  * @retval none
  */
void usbd_enumdone_handler(usbd_core_type *udev)
{
  otg_global_type *usbx = udev->usb_reg;

  usb_ept0_setup(usbx);

  usbx->gusbcfg_bit.usbtrdtim = USB_TRDTIM_16;

  /* open endpoint 0 out */
  usbd_ept_open(udev, 0x00, EPT_CONTROL_TYPE, 0x40);

  /* open endpoint 0 in */
  usbd_ept_open(udev, 0x80, EPT_CONTROL_TYPE, 0x40);

  /* usb connect state set to default */
  udev->conn_state = USB_CONN_STATE_DEFAULT;

  /* clear callback */
  if(udev->class_handler->clear_handler != 0)
    udev->class_handler->clear_handler(udev);
}

/**
  * @brief  usb rx non-empty handler
  * @param  udev: to the structure of usbd_core_type
  * @retval none
  */
void usbd_rxflvl_handler(usbd_core_type *udev)
{
  otg_global_type *usbx = udev->usb_reg;
  uint32_t stsp;
  uint32_t count;
  uint32_t pktsts;
  usb_ept_info *ept_info;

  /* disable rxflvl interrupt */
  usb_global_interrupt_enable(usbx, USB_OTG_RXFLVL_INT, FALSE);

  /* get rx status */
  stsp = usbx->grxstsp;

  /*get the byte count of receive */
  count = (stsp & USB_OTG_GRXSTSP_BCNT) >> 4;

  /* get packet status */
  pktsts = (stsp &USB_OTG_GRXSTSP_PKTSTS) >> 17;

  /* get endpoint infomation struct */
  ept_info = &udev->ept_out[stsp & USB_OTG_GRXSTSP_EPTNUM];

  /* received out data packet */
  if(pktsts == USB_OUT_STS_DATA)
  {
    if(count != 0)
    {
      /* read packet to buffer */
      usb_read_packet(usbx, ept_info->trans_buf, (stsp & USB_OTG_GRXSTSP_EPTNUM), count);
      ept_info->trans_buf += count;
      ept_info->trans_len += count;

    }
  }
  /* setup data received */
  else if ( pktsts == USB_SETUP_STS_DATA)
  {
    /* read packet to buffer */
    usb_read_packet(usbx, udev->setup_buffer, (stsp & USB_OTG_GRXSTSP_EPTNUM), count);
    ept_info->trans_len += count;
  }

  /* enable rxflvl interrupt */
  usb_global_interrupt_enable(usbx, USB_OTG_RXFLVL_INT, TRUE);

}

/**
  * @brief  usb disconnect handler
  * @param  udev: to the structure of usbd_core_type
  * @retval none
  */
void usbd_discon_handler(usbd_core_type *udev)
{
  /* disconnect callback handler */
  if(udev->class_handler->event_handler != 0)
    udev->class_handler->event_handler(udev, USBD_DISCONNECT_EVNET);
}


/**
  * @brief  usb incomplete out handler
  * @param  udev: to the structure of usbd_core_type
  * @retval none
  */
void usbd_incomisoout_handler(usbd_core_type *udev)
{
    if(udev->class_handler->event_handler != 0)
      udev->class_handler->event_handler(udev, USBD_OUTISOINCOM_EVENT);
}

/**
  * @brief  usb incomplete in handler
  * @param  udev: to the structure of usbd_core_type
  * @retval none
  */
void usbd_incomisioin_handler(usbd_core_type *udev)
{
  if(udev->class_handler->event_handler != 0)
    udev->class_handler->event_handler(udev, USBD_INISOINCOM_EVENT);
}

/**
  * @brief  usb device reset interrupt request handler.
  * @param  udev: to the structure of usbd_core_type
  * @retval none
  */
void usbd_reset_handler(usbd_core_type *udev)
{
  otg_global_type *usbx = udev->usb_reg;
  otg_device_type *dev = OTG_DEVICE(usbx);
  uint32_t i_index = 0;

  /* disable remote wakeup singal */
  dev->dctl_bit.rwkupsig = FALSE;

  /* endpoint fifo alloc */
  usbd_fifo_alloc(udev);

  /* flush all tx fifo */
  usb_flush_tx_fifo(usbx, 0x10);

  /* clear in and out endpoint interrupt flag */
  for(i_index = 0; i_index < USB_EPT_MAX_NUM; i_index ++)
  {
    USB_INEPT(usbx, i_index)->diepint = 0xFF;
    USB_OUTEPT(usbx, i_index)->doepint = 0xFF;
  }

  /* clear endpoint flag */
  dev->daint = 0xFFFFFFFF;

  /*clear endpoint interrupt mask */
  dev->daintmsk = 0x10001;

  /* enable out endpoint xfer, eptdis, setup interrupt mask */
  dev->doepmsk_bit.xfercmsk = TRUE;
  dev->doepmsk_bit.eptdismsk = TRUE;
  dev->doepmsk_bit.setupmsk = TRUE;

  /* enable in endpoint xfer, eptdis, timeout interrupt mask */
  dev->diepmsk_bit.xfercmsk = TRUE;
  dev->diepmsk_bit.eptdismsk = TRUE;
  dev->diepmsk_bit.timeoutmsk = TRUE;

  /* set device address to 0 */
  usb_set_address(usbx, 0);

  /* enable endpoint 0 */
  usb_ept0_start(usbx);

  /* usb connect state set to default */
  udev->conn_state = USB_CONN_STATE_DEFAULT;

  /* user define reset event */
  if(udev->class_handler->event_handler)
    udev->class_handler->event_handler(udev, USBD_RESET_EVENT);
}

/**
  * @brief  usb device sof interrupt request handler.
  * @param  udev: to the structure of usbd_core_type
  * @retval none
  */
void usbd_sof_handler(usbd_core_type *udev)
{
  /* user sof handler in class define */
  if(udev->class_handler->sof_handler)
    udev->class_handler->sof_handler(udev);
}

/**
  * @brief  usb device suspend interrupt request handler.
  * @param  udev: to the structure of usbd_core_type
  * @retval none
  */
void usbd_suspend_handler(usbd_core_type *udev)
{
  otg_global_type *usbx = udev->usb_reg;

  if(OTG_DEVICE(usbx)->dsts_bit.suspsts)
  {
    /* save connect state */
    udev->old_conn_state = udev->conn_state;

    /* set current state to suspend */
    udev->conn_state = USB_CONN_STATE_SUSPENDED;

    /* enter suspend mode */
    usbd_enter_suspend(udev);

    /* user suspend handler */
    if(udev->class_handler->event_handler != 0)
      udev->class_handler->event_handler(udev, USBD_SUSPEND_EVENT);
  }
}

/**
  * @brief  usb device wakup interrupt request handler.
  * @param  udev: to the structure of usbd_core_type
  * @retval none
  */
void usbd_wakeup_handler(usbd_core_type *udev)
{
  otg_global_type *usbx = udev->usb_reg;

  /* clear remote wakeup bit */
  OTG_DEVICE(usbx)->dctl_bit.rwkupsig = FALSE;

  /* exit suspend mode */
  usb_open_phy_clk(udev->usb_reg);

  /* restore connect state */
  udev->conn_state = udev->old_conn_state;

    /* user suspend handler */
  if(udev->class_handler->event_handler != 0)
    udev->class_handler->event_handler(udev, USBD_WAKEUP_EVENT);
}
/**
  * @}
  */

/**
  * @}
  */

/**
  * @}
  */
