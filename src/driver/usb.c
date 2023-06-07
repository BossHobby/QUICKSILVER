#include "driver/usb.h"

#include "core/project.h"
#include "driver/gpio.h"

#include <stdarg.h>
#include <stdio.h>
#include <string.h>

extern void usb_drv_init();

volatile bool usb_device_configured = false;

static uint8_t tx_buffer_data[USB_BUFFER_SIZE];
ring_buffer_t usb_tx_buffer = {
    .buffer = tx_buffer_data,
    .head = 0,
    .tail = 0,
    .size = USB_BUFFER_SIZE,
};

static uint8_t rx_buffer_data[USB_BUFFER_SIZE];
ring_buffer_t usb_rx_buffer = {
    .buffer = rx_buffer_data,
    .head = 0,
    .tail = 0,
    .size = USB_BUFFER_SIZE,
};

void usb_init() {
  if (target.usb_detect != PIN_NONE) {
    gpio_config_t gpio_init;
    gpio_init.mode = GPIO_INPUT;
    gpio_init.output = GPIO_OPENDRAIN;
    gpio_init.drive = GPIO_DRIVE_HIGH;
    gpio_init.pull = GPIO_NO_PULL;
    gpio_pin_init(target.usb_detect, gpio_init);
  }
  usb_drv_init();
}

uint8_t usb_detect() {
  if (target.usb_detect) {
    if (!gpio_pin_read(target.usb_detect)) {
      return 0;
    }
  }

  return usb_device_configured;
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
