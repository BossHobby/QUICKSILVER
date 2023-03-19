#include "driver/usb.h"

#include "core/project.h"
#include "driver/gpio.h"
#include "driver/interrupt.h"
#include "driver/time.h"

#include "util/ring_buffer.h"

#include <stdarg.h>
#include <stdio.h>
#include <string.h>

#include <usb.h>
#include <usb_cdc.h>

#define CDC_EP0_SIZE 0x08
#define CDC_RXD_EP 0x02
#define CDC_TXD_EP 0x83
#define CDC_DATA_SZ 0x40
#define CDC_NTF_EP 0x81
#define CDC_NTF_SZ 0x08

#define BUFFER_SIZE 4096

#define CDC_PROTOCOL USB_PROTO_NONE

struct cdc_config {
  struct usb_config_descriptor config;
  struct usb_iad_descriptor comm_iad;
  struct usb_interface_descriptor comm;
  struct usb_cdc_header_desc cdc_hdr;
  struct usb_cdc_call_mgmt_desc cdc_mgmt;
  struct usb_cdc_acm_desc cdc_acm;
  struct usb_cdc_union_desc cdc_union;
  struct usb_endpoint_descriptor comm_ep;
  struct usb_interface_descriptor data;
  struct usb_endpoint_descriptor data_eprx;
  struct usb_endpoint_descriptor data_eptx;
} __attribute__((packed));

static const struct usb_device_descriptor device_desc = {
    .bLength = sizeof(struct usb_device_descriptor),
    .bDescriptorType = USB_DTYPE_DEVICE,
    .bcdUSB = VERSION_BCD(2, 0, 0),
    .bDeviceClass = USB_CLASS_IAD,
    .bDeviceSubClass = USB_SUBCLASS_IAD,
    .bDeviceProtocol = USB_PROTO_IAD,
    .bMaxPacketSize0 = CDC_EP0_SIZE,
    .idVendor = 0x0483,
    .idProduct = 0x5740,
    .bcdDevice = VERSION_BCD(1, 0, 0),
    .iManufacturer = 1,
    .iProduct = 2,
    .iSerialNumber = INTSERIALNO_DESCRIPTOR,
    .bNumConfigurations = 1,
};

static const struct cdc_config config_desc = {
    .config = {
        .bLength = sizeof(struct usb_config_descriptor),
        .bDescriptorType = USB_DTYPE_CONFIGURATION,
        .wTotalLength = sizeof(struct cdc_config),
        .bNumInterfaces = 2,
        .bConfigurationValue = 1,
        .iConfiguration = NO_DESCRIPTOR,
        .bmAttributes = USB_CFG_ATTR_RESERVED | USB_CFG_ATTR_SELFPOWERED,
        .bMaxPower = USB_CFG_POWER_MA(100),
    },
    .comm_iad = {
        .bLength = sizeof(struct usb_iad_descriptor),
        .bDescriptorType = USB_DTYPE_INTERFASEASSOC,
        .bFirstInterface = 0,
        .bInterfaceCount = 2,
        .bFunctionClass = USB_CLASS_CDC,
        .bFunctionSubClass = USB_CDC_SUBCLASS_ACM,
        .bFunctionProtocol = CDC_PROTOCOL,
        .iFunction = NO_DESCRIPTOR,
    },
    .comm = {
        .bLength = sizeof(struct usb_interface_descriptor),
        .bDescriptorType = USB_DTYPE_INTERFACE,
        .bInterfaceNumber = 0,
        .bAlternateSetting = 0,
        .bNumEndpoints = 1,
        .bInterfaceClass = USB_CLASS_CDC,
        .bInterfaceSubClass = USB_CDC_SUBCLASS_ACM,
        .bInterfaceProtocol = CDC_PROTOCOL,
        .iInterface = NO_DESCRIPTOR,
    },
    .cdc_hdr = {
        .bFunctionLength = sizeof(struct usb_cdc_header_desc),
        .bDescriptorType = USB_DTYPE_CS_INTERFACE,
        .bDescriptorSubType = USB_DTYPE_CDC_HEADER,
        .bcdCDC = VERSION_BCD(1, 1, 0),
    },
    .cdc_mgmt = {
        .bFunctionLength = sizeof(struct usb_cdc_call_mgmt_desc),
        .bDescriptorType = USB_DTYPE_CS_INTERFACE,
        .bDescriptorSubType = USB_DTYPE_CDC_CALL_MANAGEMENT,
        .bmCapabilities = 0,
        .bDataInterface = 1,

    },
    .cdc_acm = {
        .bFunctionLength = sizeof(struct usb_cdc_acm_desc),
        .bDescriptorType = USB_DTYPE_CS_INTERFACE,
        .bDescriptorSubType = USB_DTYPE_CDC_ACM,
        .bmCapabilities = 0,
    },
    .cdc_union = {
        .bFunctionLength = sizeof(struct usb_cdc_union_desc),
        .bDescriptorType = USB_DTYPE_CS_INTERFACE,
        .bDescriptorSubType = USB_DTYPE_CDC_UNION,
        .bMasterInterface0 = 0,
        .bSlaveInterface0 = 1,
    },
    .comm_ep = {
        .bLength = sizeof(struct usb_endpoint_descriptor),
        .bDescriptorType = USB_DTYPE_ENDPOINT,
        .bEndpointAddress = CDC_NTF_EP,
        .bmAttributes = USB_EPTYPE_INTERRUPT,
        .wMaxPacketSize = CDC_NTF_SZ,
        .bInterval = 0xFF,
    },
    .data = {
        .bLength = sizeof(struct usb_interface_descriptor),
        .bDescriptorType = USB_DTYPE_INTERFACE,
        .bInterfaceNumber = 1,
        .bAlternateSetting = 0,
        .bNumEndpoints = 2,
        .bInterfaceClass = USB_CLASS_CDC_DATA,
        .bInterfaceSubClass = USB_SUBCLASS_NONE,
        .bInterfaceProtocol = USB_PROTO_NONE,
        .iInterface = NO_DESCRIPTOR,
    },
    .data_eprx = {
        .bLength = sizeof(struct usb_endpoint_descriptor),
        .bDescriptorType = USB_DTYPE_ENDPOINT,
        .bEndpointAddress = CDC_RXD_EP,
        .bmAttributes = USB_EPTYPE_BULK,
        .wMaxPacketSize = CDC_DATA_SZ,
        .bInterval = 0x01,
    },
    .data_eptx = {
        .bLength = sizeof(struct usb_endpoint_descriptor),
        .bDescriptorType = USB_DTYPE_ENDPOINT,
        .bEndpointAddress = CDC_TXD_EP,
        .bmAttributes = USB_EPTYPE_BULK,
        .wMaxPacketSize = CDC_DATA_SZ,
        .bInterval = 0x01,
    },
};

static const struct usb_string_descriptor lang_desc = USB_ARRAY_DESC(USB_LANGID_ENG_US);
static const struct usb_string_descriptor manuf_desc_en = USB_STRING_DESC("QUICKSILVER");
static const struct usb_string_descriptor prod_desc_en = USB_STRING_DESC("QUICKSILVER");
static const struct usb_string_descriptor *const dtable[] = {
    &lang_desc,
    &manuf_desc_en,
    &prod_desc_en,
};

static struct usb_cdc_line_coding cdc_line = {
    .dwDTERate = 921600,
    .bCharFormat = USB_CDC_1_STOP_BITS,
    .bParityType = USB_CDC_NO_PARITY,
    .bDataBits = 8,
};

static usbd_device udev;
static uint32_t ubuf[0x20];

static volatile bool usb_device_configured = false;

static uint8_t tx_buffer_data[BUFFER_SIZE];
static ring_buffer_t tx_buffer = {
    .buffer = tx_buffer_data,
    .head = 0,
    .tail = 0,
    .size = BUFFER_SIZE,
};

static uint8_t rx_buffer_data[BUFFER_SIZE];
static ring_buffer_t rx_buffer = {
    .buffer = rx_buffer_data,
    .head = 0,
    .tail = 0,
    .size = BUFFER_SIZE,
};

static usbd_respond cdc_getdesc(usbd_ctlreq *req, void **address, uint16_t *length) {
  const uint8_t dtype = req->wValue >> 8;
  const uint8_t dnumber = req->wValue & 0xFF;
  const void *desc;
  uint16_t len = 0;
  switch (dtype) {
  case USB_DTYPE_DEVICE:
    desc = &device_desc;
    break;
  case USB_DTYPE_CONFIGURATION:
    desc = &config_desc;
    len = sizeof(config_desc);
    break;
  case USB_DTYPE_STRING:
    if (dnumber < 3) {
      desc = dtable[dnumber];
    } else {
      return usbd_fail;
    }
    break;
  default:
    return usbd_fail;
  }
  if (len == 0) {
    len = ((struct usb_header_descriptor *)desc)->bLength;
  }
  *address = (void *)desc;
  *length = len;
  return usbd_ack;
}

static usbd_respond cdc_control(usbd_device *dev, usbd_ctlreq *req, usbd_rqc_callback *callback) {
  if (((USB_REQ_RECIPIENT | USB_REQ_TYPE) & req->bmRequestType) == (USB_REQ_INTERFACE | USB_REQ_CLASS) && req->wIndex == 0) {
    switch (req->bRequest) {
    case USB_CDC_SET_CONTROL_LINE_STATE:
      return usbd_ack;
    case USB_CDC_SET_LINE_CODING:
      memcpy(&cdc_line, req->data, sizeof(cdc_line));
      return usbd_ack;
    case USB_CDC_GET_LINE_CODING:
      dev->status.data_ptr = &cdc_line;
      dev->status.data_count = sizeof(cdc_line);
      return usbd_ack;
    default:
      return usbd_fail;
    }
  }
  return usbd_fail;
}

static volatile bool rx_stalled = false;

static void cdc_rxonly(usbd_device *dev, uint8_t event, uint8_t ep) {
  if (ring_buffer_free(&rx_buffer) <= CDC_DATA_SZ) {
    interrupt_disable(OTG_FS_IRQn);
    rx_stalled = true;
    return;
  }
  rx_stalled = false;

  static uint8_t buf[CDC_DATA_SZ];
  const int32_t len = usbd_ep_read(dev, ep, buf, CDC_DATA_SZ);
  if (len == 0) {
    return;
  }
  ring_buffer_write_multi(&rx_buffer, buf, len);
}

static void cdc_kickoff_rx() {
  if (!rx_stalled) {
    return;
  }

  cdc_rxonly(&udev, 0, CDC_RXD_EP);
  interrupt_enable(OTG_FS_IRQn, USB_PRIORITY);
}

static volatile bool tx_stalled = true;

static void cdc_txonly(usbd_device *dev, uint8_t event, uint8_t ep) {
  static volatile bool did_zlp = false;

  static uint8_t buf[CDC_DATA_SZ];
  const uint32_t len = ring_buffer_read_multi(&tx_buffer, buf, CDC_DATA_SZ);

  if (len) {
    usbd_ep_write(dev, ep, buf, len);
    tx_stalled = false;

    // transfers smaller than max size count as zlp
    if (len == CDC_DATA_SZ) {
      did_zlp = false;
    } else {
      did_zlp = true;
    }
  } else {
    if (!did_zlp) {
      // no zlp sent, flush now
      usbd_ep_write(dev, ep, 0, 0);
      tx_stalled = false;
    } else {
      tx_stalled = true;
    }
  }
}

static void cdc_kickoff_tx() {
  if (!tx_stalled) {
    return;
  }

  cdc_txonly(&udev, 0, CDC_TXD_EP);
}

static usbd_respond cdc_setconf(usbd_device *dev, uint8_t cfg) {
  switch (cfg) {
  case 0:
    /* deconfiguring device */
    usbd_ep_deconfig(dev, CDC_NTF_EP);
    usbd_ep_deconfig(dev, CDC_TXD_EP);
    usbd_ep_deconfig(dev, CDC_RXD_EP);
    usbd_reg_endpoint(dev, CDC_RXD_EP, 0);
    usbd_reg_endpoint(dev, CDC_TXD_EP, 0);
    usb_device_configured = false;
    return usbd_ack;

  case 1:
    /* configuring device */
    usbd_ep_config(dev, CDC_RXD_EP, USB_EPTYPE_BULK, CDC_DATA_SZ);
    usbd_ep_config(dev, CDC_TXD_EP, USB_EPTYPE_BULK, CDC_DATA_SZ);
    usbd_ep_config(dev, CDC_NTF_EP, USB_EPTYPE_INTERRUPT, CDC_NTF_SZ);
    usbd_reg_endpoint(dev, CDC_RXD_EP, cdc_rxonly);
    usbd_reg_endpoint(dev, CDC_TXD_EP, cdc_txonly);
    usb_device_configured = true;
    return usbd_ack;

  default:
    usb_device_configured = false;
    return usbd_fail;
  }
}

void OTG_FS_IRQHandler() {
  usbd_poll(&udev);
}

void usb_init() {
  LL_GPIO_InitTypeDef gpio_init;
  gpio_init.Mode = LL_GPIO_MODE_ALTERNATE;
  gpio_init.OutputType = LL_GPIO_OUTPUT_PUSHPULL;
  gpio_init.Speed = LL_GPIO_SPEED_FREQ_HIGH;
  gpio_init.Pull = LL_GPIO_PULL_NO;

#ifdef STM32H7
  gpio_pin_init_af(&gpio_init, PIN_A11, GPIO_AF10_OTG1_FS);
  gpio_pin_init_af(&gpio_init, PIN_A12, GPIO_AF10_OTG1_FS);
#else
  gpio_pin_init_af(&gpio_init, PIN_A11, GPIO_AF10_OTG_FS);
  gpio_pin_init_af(&gpio_init, PIN_A12, GPIO_AF10_OTG_FS);
#endif

#ifdef USB_DETECT_PIN
  gpio_init.Mode = LL_GPIO_MODE_INPUT;
  gpio_init.OutputType = LL_GPIO_OUTPUT_OPENDRAIN;
  gpio_init.Speed = LL_GPIO_SPEED_FREQ_HIGH;
  gpio_init.Pull = LL_GPIO_PULL_NO;
  gpio_pin_init(&gpio_init, USB_DETECT_PIN);
#endif

  interrupt_enable(OTG_FS_IRQn, USB_PRIORITY);

  usbd_init(&udev, &usbd_hw, CDC_EP0_SIZE, ubuf, sizeof(ubuf));
  usbd_reg_config(&udev, cdc_setconf);
  usbd_reg_control(&udev, cdc_control);
  usbd_reg_descr(&udev, cdc_getdesc);

  usbd_enable(&udev, true);
  usbd_connect(&udev, true);

#ifdef STM32H7
  LL_PWR_EnableUSBVoltageDetector();
  LL_PWR_DisableUSBReg();

  while (!LL_PWR_IsActiveFlag_USB()) {
    time_delay_ms(100);
  }
#endif
}

uint8_t usb_detect() {
#ifdef USB_DETECT_PIN
  const uint8_t usb_connect = gpio_pin_read(USB_DETECT_PIN);
  if (usb_connect != 1) {
    // no usb connetion, bail
    return 0;
  }
#endif

  return usb_device_configured;
}

uint32_t usb_serial_read(uint8_t *data, uint32_t len) {
  if (data == NULL || len == 0) {
    return 0;
  }
  const uint32_t read = ring_buffer_read_multi(&rx_buffer, data, len);
  cdc_kickoff_rx();
  return read;
}

uint8_t usb_serial_read_byte() {
  uint8_t byte = 0;
  for (uint32_t timeout = 1000; usb_serial_read(&byte, 1) != 1 && timeout; --timeout) {
    time_delay_us(10);
    __NOP();
  }
  return byte;
}

void usb_serial_write(uint8_t *data, uint32_t len) {
  if (data == NULL || len == 0) {
    return;
  }

  uint32_t written = 0;
  while (written < len) {
    written += ring_buffer_write_multi(&tx_buffer, data + written, len - written);
    cdc_kickoff_tx();
  }
}

void usb_serial_print(char *str) {
  usb_serial_write((uint8_t *)str, strlen(str));
}

void usb_serial_printf(const char *fmt, ...) {
  const size_t size = strlen(fmt) + 128;
  char str[size];

  memset(str, 0, size);

  va_list args;
  va_start(args, fmt);
  vsnprintf(str, size, fmt, args);
  va_end(args);

  usb_serial_print(str);
}
