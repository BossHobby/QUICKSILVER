#include "drv_spi.h"

#define PIN MAKE_PIN_DEF
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
#undef PIN

#define PORT spi_port_defs[port]
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

uint8_t spi_transfer_byte(spi_ports_t port, uint8_t data) {
  for (uint16_t timeout = 1000; SPI_I2S_GetFlagStatus(PORT.channel, SPI_I2S_FLAG_TXE) == RESET; timeout--) {
    if (timeout == 0) {
      //liberror will trigger failloop 7 during boot, or 20 liberrors will trigger failloop 8 in flight
      liberror++;
      return 0;
    }
  }

  SPI_I2S_SendData(PORT.channel, data);

  for (uint16_t timeout = 1000; SPI_I2S_GetFlagStatus(PORT.channel, SPI_I2S_FLAG_RXNE) == RESET; timeout--) {
    if (timeout == 0) {
      //liberror will trigger failloop 7 during boot, or 20 liberrors will trigger failloop 8 in flight
      liberror++;
      return 0;
    }
  }

  return SPI_I2S_ReceiveData(PORT.channel);
}