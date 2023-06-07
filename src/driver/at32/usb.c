#include "driver/usb.h"

#include <cdc_class.h>
#include <cdc_desc.h>
#include <usb_conf.h>
#include <usb_core.h>
#include <usbd_int.h>

#include "driver/gpio.h"
#include "driver/time.h"

extern volatile bool usb_device_configured;

static otg_core_type otg_core_struct;

static volatile bool tx_stalled = true;

static void usb_enable_clock() {
  crm_usb_clock_source_select(CRM_USB_CLOCK_SOURCE_HICK);

  crm_periph_clock_enable(CRM_ACC_PERIPH_CLOCK, TRUE);

  acc_write_c1(7980);
  acc_write_c2(8000);
  acc_write_c3(8020);
  acc_sof_select(ACC_SOF_OTG1);

  acc_calibration_mode_enable(ACC_CAL_HICKTRIM, TRUE);
}

void usb_drv_init() {
  gpio_config_t gpio_init;
  gpio_init.mode = GPIO_ALTERNATE;
  gpio_init.output = GPIO_PUSHPULL;
  gpio_init.drive = GPIO_DRIVE_HIGH;
  gpio_init.pull = GPIO_NO_PULL;
  gpio_pin_init_af(PIN_A11, gpio_init, GPIO_MUX_10);
  gpio_pin_init_af(PIN_A12, gpio_init, GPIO_MUX_10);

  crm_periph_clock_enable(CRM_OTGFS1_PERIPH_CLOCK, TRUE);
  usb_enable_clock();
  nvic_irq_enable(OTGFS1_IRQn, 0, 0);

  usbd_init(&otg_core_struct,
            USB_FULL_SPEED_CORE_ID,
            0,
            &cdc_class_handler,
            &cdc_desc_handler);
}

void usb_cdc_rx_handler() {
  usb_device_configured = true;

  static uint8_t buf[USBD_CDC_IN_MAXPACKET_SIZE];

  const uint16_t len = usb_vcp_get_rxdata(&otg_core_struct.dev, buf);
  if (len == 0) {
    return;
  }
  ring_buffer_write_multi(&usb_rx_buffer, buf, len);
}

void usb_cdc_tx_handler() {
  usb_device_configured = true;

  static volatile bool did_zlp = false;

  static uint8_t buf[USBD_CDC_OUT_MAXPACKET_SIZE];
  const uint32_t len = ring_buffer_read_multi(&usb_tx_buffer, buf, USBD_CDC_OUT_MAXPACKET_SIZE);

  usb_vcp_send_data(&otg_core_struct.dev, buf, len);

  if (len) {
    usb_vcp_send_data(&otg_core_struct.dev, buf, len);
    tx_stalled = false;

    // transfers smaller than max size count as zlp
    if (len == USBD_CDC_OUT_MAXPACKET_SIZE) {
      did_zlp = false;
    } else {
      did_zlp = true;
    }
  } else {
    if (!did_zlp) {
      // no zlp sent, flush now
      usb_vcp_send_data(&otg_core_struct.dev, 0, 0);
      tx_stalled = false;
    } else {
      tx_stalled = true;
    }
  }
}

void usb_cdc_kickoff_tx() {
  if (!tx_stalled) {
    return;
  }

  usb_vcp_send_data(&otg_core_struct.dev, 0, 0);
}

uint32_t usb_serial_read(uint8_t *data, uint32_t len) {
  if (data == NULL || len == 0) {
    return 0;
  }
  return ring_buffer_read_multi(&usb_rx_buffer, data, len);
}

void usb_serial_write(uint8_t *data, uint32_t len) {
  if (data == NULL || len == 0) {
    return;
  }

  uint32_t written = 0;
  while (written < len) {
    written += ring_buffer_write_multi(&usb_tx_buffer, data + written, len - written);
    usb_cdc_kickoff_tx();
  }
}

void usb_delay_ms(uint32_t ms) {
  time_delay_ms(ms);
}

void usb_delay_us(uint32_t us) {
  time_delay_us(us);
}

void OTGFS1_IRQHandler() {
  usbd_irq_handler(&otg_core_struct);
}