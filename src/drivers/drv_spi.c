#include "drv_spi.h"

#include "usb_configurator.h"

#define GPIO_PIN MAKE_PIN_DEF
#define SPI_PORT(chan, sck_pin, miso_pin, mosi_pin) \
  {                                                 \
      .channel_index = chan,                        \
      .channel = SPI##chan,                         \
      .gpio_af = GPIO_AF_SPI##chan,                 \
      .sck = sck_pin,                               \
      .miso = miso_pin,                             \
      .mosi = mosi_pin,                             \
  },

spi_port_def_t spi_port_defs[SPI_PORTS_MAX] = {
    {},
    SPI_PORTS};

#undef SPI_PORT
#undef GPIO_PIN

#define PORT spi_port_defs[port]
#define SCK_PIN gpio_pin_defs[PORT.sck]
#define MISO_PIN gpio_pin_defs[PORT.miso]
#define MOSI_PIN gpio_pin_defs[PORT.mosi]

extern int liberror;

void spi_enable_rcc(spi_ports_t port) {
  switch (PORT.channel_index) {
  case 1:
    RCC_APB2PeriphClockCmd(RCC_APB2Periph_SPI1, ENABLE);
    break;
  case 2:
    RCC_APB1PeriphClockCmd(RCC_APB1Periph_SPI2, ENABLE);
    break;
  case 3:
    RCC_APB1PeriphClockCmd(RCC_APB1Periph_SPI3, ENABLE);
    break;
  }
}

void spi_csn_enable(gpio_pins_t nss) {
  GPIO_ResetBits(gpio_pin_defs[nss].port, gpio_pin_defs[nss].pin);
}

void spi_csn_disable(gpio_pins_t nss) {
  GPIO_SetBits(gpio_pin_defs[nss].port, gpio_pin_defs[nss].pin);
}

void spi_init_pins(spi_ports_t port, gpio_pins_t nss) {
  GPIO_InitTypeDef GPIO_InitStructure;

  GPIO_InitStructure.GPIO_Pin = SCK_PIN.pin;
  GPIO_InitStructure.GPIO_Mode = GPIO_Mode_AF;
  GPIO_InitStructure.GPIO_Speed = GPIO_Speed_50MHz;
  GPIO_InitStructure.GPIO_OType = GPIO_OType_PP;
  GPIO_InitStructure.GPIO_PuPd = GPIO_PuPd_UP;
  GPIO_Init(SCK_PIN.port, &GPIO_InitStructure);

  GPIO_InitStructure.GPIO_Pin = MOSI_PIN.pin;
  GPIO_InitStructure.GPIO_Mode = GPIO_Mode_AF;
  GPIO_InitStructure.GPIO_Speed = GPIO_Speed_50MHz;
  GPIO_InitStructure.GPIO_OType = GPIO_OType_PP;
  GPIO_InitStructure.GPIO_PuPd = GPIO_PuPd_UP;
  GPIO_Init(MOSI_PIN.port, &GPIO_InitStructure);

  GPIO_InitStructure.GPIO_Pin = MISO_PIN.pin;
  GPIO_InitStructure.GPIO_Mode = GPIO_Mode_AF;
  GPIO_InitStructure.GPIO_Speed = GPIO_Speed_50MHz;
  GPIO_InitStructure.GPIO_OType = GPIO_OType_PP;
  GPIO_InitStructure.GPIO_PuPd = GPIO_PuPd_UP;
  GPIO_Init(MISO_PIN.port, &GPIO_InitStructure);

  GPIO_PinAFConfig(SCK_PIN.port, SCK_PIN.pin_source, PORT.gpio_af);
  GPIO_PinAFConfig(MISO_PIN.port, MISO_PIN.pin_source, PORT.gpio_af);
  GPIO_PinAFConfig(MOSI_PIN.port, MOSI_PIN.pin_source, PORT.gpio_af);

  GPIO_InitStructure.GPIO_Pin = gpio_pin_defs[nss].pin;
  GPIO_InitStructure.GPIO_Mode = GPIO_Mode_OUT;
  GPIO_InitStructure.GPIO_Speed = GPIO_Speed_50MHz;
  GPIO_InitStructure.GPIO_OType = GPIO_OType_PP;
  GPIO_InitStructure.GPIO_PuPd = GPIO_PuPd_NOPULL;
  GPIO_Init(gpio_pin_defs[nss].port, &GPIO_InitStructure);
}

uint8_t spi_transfer_byte(spi_ports_t port, uint8_t data) {
  for (uint16_t timeout = 0x1000; SPI_I2S_GetFlagStatus(PORT.channel, SPI_I2S_FLAG_TXE) == RESET; timeout--) {
    if (timeout == 0) {
      //liberror will trigger failloop 7 during boot, or 20 liberrors will trigger failloop 8 in flight
      liberror++;
      return 0;
    }
  }

  SPI_I2S_SendData(PORT.channel, data);

  for (uint16_t timeout = 0x1000; SPI_I2S_GetFlagStatus(PORT.channel, SPI_I2S_FLAG_RXNE) == RESET; timeout--) {
    if (timeout == 0) {
      //liberror will trigger failloop 7 during boot, or 20 liberrors will trigger failloop 8 in flight
      liberror++;
      return 0;
    }
  }

  for (uint16_t timeout = 0x1000; SPI_I2S_GetFlagStatus(PORT.channel, SPI_I2S_FLAG_BSY) == SET; timeout--) {
    if (timeout == 0) {
      //liberror will trigger failloop 7 during boot, or 20 liberrors will trigger failloop 8 in flight
      liberror++;
      return 0;
    }
  }

  return SPI_I2S_ReceiveData(PORT.channel);
}