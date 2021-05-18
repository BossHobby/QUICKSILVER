/*
The MIT License

Copyright (c) 2017 Mike Morrison

Permission is hereby granted, free of charge, to any person obtaining a copy
of this software and associated documentation files (the "Software"), to deal
in the Software without restriction, including without limitation the rights
to use, copy, modify, merge, publish, distribute, sublicense, and/or sell
copies of the Software, and to permit persons to whom the Software is
furnished to do so, subject to the following conditions:

The above copyright notice and this permission notice shall be included in all
copies or substantial portions of the Software.

THE SOFTWARE IS PROVIDED "AS IS", WITHOUT WARRANTY OF ANY KIND, EXPRESS OR
IMPLIED, INCLUDING BUT NOT LIMITED TO THE WARRANTIES OF MERCHANTABILITY,
FITNESS FOR A PARTICULAR PURPOSE AND NONINFRINGEMENT. IN NO EVENT SHALL THE
AUTHORS OR COPYRIGHT HOLDERS BE LIABLE FOR ANY CLAIM, DAMAGES OR OTHER
LIABILITY, WHETHER IN AN ACTION OF CONTRACT, TORT OR OTHERWISE, ARISING FROM,
OUT OF OR IN CONNECTION WITH THE SOFTWARE OR THE USE OR OTHER DEALINGS IN THE
SOFTWARE.
*/
#include "drv_serial_soft.h"

#include "defines.h"
#include "drv_gpio.h"

uint32_t softserial_micros_per_bit = (uint32_t)(1000000 / 9600);
uint32_t softserial_micros_per_bit_half = (uint32_t)(1000000 / 9600) * .5;
uint32_t esc_micros_per_bit = (uint32_t)(1000000 / 19200);

#ifdef F4
#define SET_TX_HIGH(data) data->tx_port->BSRRL = data->tx_pin
#define SET_RX_HIGH(data) data->rx_port->BSRRL = data->rx_pin
#define SET_TX_LOW(data) data->tx_port->BSRRH = data->tx_pin
#endif

#define SET_LED1_ON gpio_pin_set(LED1PIN)
#define SET_LED1_OFF gpio_pin_reset(LED1PIN)

#define SET_LED2_ON gpio_pin_set(LED2PIN)
#define SET_LED2_OFF gpio_pin_reset(LED2PIN)

#define START_BIT(data) SET_TX_LOW(data)
#define STOP_BIT(data) SET_TX_HIGH(data)
#define IS_RX_HIGH(data) (data->rx_port->IDR & data->rx_pin)

static SoftSerialData_t globalSerialData = {0};

static int softserial_is_1wire(const SoftSerialData_t *data) {
  return data->tx_port == data->rx_port && data->tx_pin == data->rx_pin;
}

static void softserial_init_rx(const SoftSerialData_t *data) {
  if (0 != data->rx_port) {
    GPIO_InitTypeDef gpio_init = {0};
    gpio_init.GPIO_Mode = GPIO_Mode_IN;
    gpio_init.GPIO_Pin = data->rx_pin;
    gpio_init.GPIO_OType = GPIO_OType_OD;
    gpio_init.GPIO_PuPd = GPIO_PuPd_UP;
    gpio_init.GPIO_Speed = GPIO_Speed_50MHz;
    GPIO_Init(data->rx_port, &gpio_init);
    SET_RX_HIGH(data);
  }
}
static void softserial_init_tx(const SoftSerialData_t *data) {
  if (0 != data->tx_port) {
    GPIO_InitTypeDef gpio_init = {0};
    gpio_init.GPIO_Mode = GPIO_Mode_OUT;
    gpio_init.GPIO_Pin = data->tx_pin;
    gpio_init.GPIO_OType = GPIO_OType_PP;
    gpio_init.GPIO_PuPd = GPIO_PuPd_NOPULL;
    gpio_init.GPIO_Speed = GPIO_Speed_50MHz;
    GPIO_Init(data->tx_port, &gpio_init);
    SET_TX_HIGH(data);
  }
}

SoftSerialData_t softserial_init(GPIO_TypeDef *tx_port, uint16_t tx_pin, GPIO_TypeDef *rx_port, uint16_t rx_pin, uint32_t baudrate) {
  if (0 == tx_port && 0 == rx_port) {
    SoftSerialData_t data = {0};
    return data;
  }

  globalSerialData.tx_port = tx_port;
  globalSerialData.tx_pin = tx_pin;
  globalSerialData.rx_port = rx_port;
  globalSerialData.rx_pin = rx_pin;

  softserial_init_tx(&globalSerialData);
  softserial_init_rx(&globalSerialData);

  globalSerialData.baud = baudrate;
  globalSerialData.micros_per_bit = (uint32_t)(1000000 / baudrate);
  globalSerialData.micros_per_bit_half = globalSerialData.micros_per_bit * .5;

  return globalSerialData;
}

int softserial_read_byte(uint8_t *byte) {
  return softserial_read_byte_ex(&globalSerialData, byte);
}

void softserial_set_input(const SoftSerialData_t *data) {
  SET_LED1_ON;
  if (softserial_is_1wire(data))
    softserial_init_rx(data);
}

void softserial_set_output(const SoftSerialData_t *data) {
  SET_LED1_OFF;
  if (softserial_is_1wire(data))
    softserial_init_tx(data);
  delay(20);
}

// return 1 on success
int softserial_read_byte_ex(const SoftSerialData_t *data, uint8_t *byte) {
  int i = 0;
  uint8_t b = 0;

  uint32_t time_start = gettime();
  uint32_t time_next = time_start;
  while (!IS_RX_HIGH(data)) {
    time_next = gettime(); //wait for start bit
    if (time_next - time_start > 10000)
      return 0;
  }
  while (IS_RX_HIGH(data)) // start bit falling edge
  {
    time_next = gettime(); //wait for start bit
    if (time_next - time_start > 10000)
      return 0;
  }

  time_next += data->micros_per_bit_half; // move away from edge to center of bit

  for (; i < 8; ++i) {
    time_next += data->micros_per_bit;
    delay_until(time_next);
    b >>= 1;
    if (IS_RX_HIGH(data))
      b |= 0x80;
  }

  time_next += data->micros_per_bit;
  delay_until(time_next); // move away from edge

  if (!(IS_RX_HIGH(data))) // stop bit
  {
    // error no stop bit
    *byte = 0;
    return 0;
  }

  *byte = b;
  return 1;
}

void softserial_write_byte(uint8_t byte) {
  softserial_write_byte_ex(&globalSerialData, byte);
}

void softserial_write_byte_ex(const SoftSerialData_t *data, uint8_t byte) {

  int i = 0;

  START_BIT(data);
  uint32_t next_time = gettime();

  for (; i < 8; ++i) {
    next_time += data->micros_per_bit;
    delay_until(next_time);

    if (0x01 & byte)
      SET_TX_HIGH(data);
    else
      SET_TX_LOW(data);
    byte = byte >> 1;
  }
  next_time += data->micros_per_bit;
  delay_until(next_time);

  STOP_BIT(data);
  next_time += data->micros_per_bit;
  delay_until(next_time);
}
