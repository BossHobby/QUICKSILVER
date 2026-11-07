#include "driver/usb.h"

#include <hardware/irq.h>
#include <pico/unique_id.h>
#include <tusb.h>

#include "control/control.h"
#include "driver/interrupt.h"
#include "driver/time.h"

// TinyUSB defers bus events to tud_task(). It runs in a second USBCTRL_IRQ
// handler after the controller driver's own, so all USB work stays in that
// interrupt like on the other MCUs; tasks only touch the ring buffers and
// pend the interrupt to have them serviced.

static constexpr uint16_t USB_VID = 0x2E8A; // Raspberry Pi
static constexpr uint16_t USB_PID = 0x000A; // Pico SDK CDC

enum {
  USB_ITF_CDC,
  USB_ITF_CDC_DATA,
  USB_ITF_COUNT,
};

enum {
  USB_STR_LANGID,
  USB_STR_MANUFACTURER,
  USB_STR_PRODUCT,
  USB_STR_SERIAL,
};

static constexpr uint8_t USB_EP_CDC_NOTIF = 0x81;
static constexpr uint8_t USB_EP_CDC_OUT = 0x02;
static constexpr uint8_t USB_EP_CDC_IN = 0x82;
static constexpr uint16_t USB_CONFIG_LEN = TUD_CONFIG_DESC_LEN + TUD_CDC_DESC_LEN;

static const tusb_desc_device_t usb_device_desc = {
    .bLength = sizeof(tusb_desc_device_t),
    .bDescriptorType = TUSB_DESC_DEVICE,
    .bcdUSB = 0x0200,
    .bDeviceClass = TUSB_CLASS_MISC,
    .bDeviceSubClass = MISC_SUBCLASS_COMMON,
    .bDeviceProtocol = MISC_PROTOCOL_IAD,
    .bMaxPacketSize0 = CFG_TUD_ENDPOINT0_SIZE,
    .idVendor = USB_VID,
    .idProduct = USB_PID,
    .bcdDevice = 0x0100,
    .iManufacturer = USB_STR_MANUFACTURER,
    .iProduct = USB_STR_PRODUCT,
    .iSerialNumber = USB_STR_SERIAL,
    .bNumConfigurations = 1,
};

static const uint8_t usb_config_desc[] = {
    TUD_CONFIG_DESCRIPTOR(1, USB_ITF_COUNT, 0, USB_CONFIG_LEN, 0, 500),
    TUD_CDC_DESCRIPTOR(USB_ITF_CDC, 0, USB_EP_CDC_NOTIF, 8, USB_EP_CDC_OUT, USB_EP_CDC_IN, CFG_TUD_CDC_EP_BUFSIZE),
};

extern "C" const uint8_t *tud_descriptor_device_cb() {
  return (const uint8_t *)&usb_device_desc;
}

extern "C" const uint8_t *tud_descriptor_configuration_cb(uint8_t index) {
  return usb_config_desc;
}

extern "C" const uint16_t *tud_descriptor_string_cb(uint8_t index, uint16_t langid) {
  static uint16_t desc[33];

  char serial[2 * PICO_UNIQUE_BOARD_ID_SIZE_BYTES + 1];
  const char *str = nullptr;
  switch (index) {
  case USB_STR_LANGID:
    desc[1] = 0x0409;
    desc[0] = (TUSB_DESC_STRING << 8) | 4;
    return desc;
  case USB_STR_MANUFACTURER:
    str = "QUICKSILVER";
    break;
  case USB_STR_PRODUCT:
    str = (const char *)target.name;
    break;
  case USB_STR_SERIAL:
    pico_get_unique_board_id_string(serial, sizeof(serial));
    str = serial;
    break;
  default:
    return nullptr;
  }

  uint32_t len = 0;
  while (str[len] != '\0' && len < 32) {
    desc[1 + len] = str[len];
    len++;
  }
  desc[0] = (TUSB_DESC_STRING << 8) | (2 * len + 2);
  return desc;
}

extern "C" void tud_umount_cb() {
  flags.usb_active = 0;
}

extern "C" void tud_suspend_cb(bool remote_wakeup_en) {
  flags.usb_active = 0;
}

static void usb_cdc_service() {
  uint8_t buf[CFG_TUD_CDC_EP_BUFSIZE];

  // Leave data in TinyUSB's FIFO while the ring is full; the host is NAKed.
  while (tud_cdc_available() && ring_buffer_free(&usb_rx_buffer) >= sizeof(buf)) {
    const uint32_t len = tud_cdc_read(buf, sizeof(buf));
    ring_buffer_write_multi(&usb_rx_buffer, buf, len);
    flags.usb_active = 1;
  }

  bool wrote = false;
  while (tud_cdc_write_available() >= sizeof(buf)) {
    const uint32_t len = ring_buffer_read_multi(&usb_tx_buffer, buf, sizeof(buf));
    if (len == 0)
      break;
    tud_cdc_write(buf, len);
    wrote = true;
  }
  if (wrote)
    tud_cdc_write_flush();
}

static void usb_irq_handler() {
  tud_task();
  if (tud_mounted())
    usb_cdc_service();
}

static void usb_kick() {
  irq_set_pending(USBCTRL_IRQ);
}

void usb_drv_init() {
  tud_init(0);
  irq_add_shared_handler(USBCTRL_IRQ, usb_irq_handler, PICO_SHARED_IRQ_HANDLER_LOWEST_ORDER_PRIORITY);
  interrupt_enable(USBCTRL_IRQ_IRQn, USB_PRIORITY);
}

uint32_t usb_serial_read(uint8_t *data, uint32_t len) {
  if (data == NULL || len == 0) {
    return 0;
  }
  const uint32_t read = ring_buffer_read_multi(&usb_rx_buffer, data, len);
  usb_kick();
  return read;
}

void usb_serial_write(uint8_t *data, uint32_t len) {
  if (data == NULL || len == 0) {
    return;
  }

  // Drop the rest once the host stops draining TX so callers cannot hang.
  uint32_t written = 0;
  uint32_t last_progress = time_millis();
  while (written < len && flags.usb_active) {
    const uint32_t count = ring_buffer_write_multi(&usb_tx_buffer, data + written, len - written);
    usb_kick();
    if (count) {
      written += count;
      last_progress = time_millis();
    } else if (time_millis() - last_progress > USB_WRITE_TIMEOUT_MS) {
      break;
    }
  }
}
