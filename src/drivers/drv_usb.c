#include "drv_usb.h"

#include "defines.h"
#include "drv_gpio.h"
#include "drv_time.h"

#ifdef F4

#include <stdarg.h>
#include <stdio.h>
#include <string.h>

void usb_init(void) {

#ifdef USB_DETECT_PIN
  LL_GPIO_InitTypeDef gpio_init;
  gpio_init.Mode = LL_GPIO_MODE_INPUT;
  gpio_init.OutputType = LL_GPIO_OUTPUT_OPENDRAIN;
  gpio_init.Speed = LL_GPIO_SPEED_FREQ_HIGH;
  gpio_init.Pull = LL_GPIO_PULL_NO;
  gpio_pin_init(&gpio_init, USB_DETECT_PIN);
#endif
}

uint8_t usb_detect(void) {
#ifdef USB_DETECT_PIN
  const uint8_t usb_connect = gpio_pin_read(USB_DETECT_PIN);
  if (usb_connect != 1) {
    // no usb connetion, bail
    return 0;
  }
#endif

  return 1;
}

uint32_t usb_serial_read(uint8_t *data, uint32_t len) {
  if (data == NULL || len == 0) {
    return 0;
  }
  // TODO
  return 0;
}

uint8_t usb_serial_read_byte(void) {
  uint8_t byte = 0;
  for (uint32_t timeout = 1000; usb_serial_read(&byte, 1) != 1 && timeout; --timeout) {
    timer_delay_us(10);
    __WFI();
  }
  return byte;
}

void usb_serial_write(uint8_t *data, uint32_t len) {
  if (data == NULL || len == 0) {
    return;
  }
  // TODO
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

#endif
