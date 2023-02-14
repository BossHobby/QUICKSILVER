#include "driver/spi_sx127x.h"

#include "driver/exti.h"
#include "driver/spi.h"
#include "driver/time.h"
#include "project.h"

#if defined(USE_SX127X)

#define PORT spi_port_defs[SX12XX_SPI_PORT]

static volatile uint8_t dio0_active = 0;

void sx127x_init() {
  spi_init_pins(SX12XX_SPI_PORT, SX12XX_NSS_PIN);

  LL_GPIO_InitTypeDef gpio_init;
  gpio_init.Mode = LL_GPIO_MODE_INPUT;
  gpio_init.Speed = LL_GPIO_SPEED_FREQ_VERY_HIGH;
  gpio_init.OutputType = LL_GPIO_OUTPUT_OPENDRAIN;
  gpio_init.Pull = LL_GPIO_PULL_NO;
  gpio_pin_init(&gpio_init, SX12XX_DIO0_PIN);

  exti_enable(SX12XX_DIO0_PIN);

  spi_enable_rcc(SX12XX_SPI_PORT);

  LL_SPI_DeInit(PORT.channel);
  LL_SPI_InitTypeDef SPI_InitStructure;
  SPI_InitStructure.TransferDirection = LL_SPI_FULL_DUPLEX;
  SPI_InitStructure.Mode = LL_SPI_MODE_MASTER;
  SPI_InitStructure.DataWidth = LL_SPI_DATAWIDTH_8BIT;
  SPI_InitStructure.ClockPolarity = LL_SPI_POLARITY_LOW;
  SPI_InitStructure.ClockPhase = LL_SPI_PHASE_1EDGE;
  SPI_InitStructure.NSS = LL_SPI_NSS_SOFT;
  SPI_InitStructure.BaudRate = LL_SPI_BAUDRATEPRESCALER_DIV8;
  SPI_InitStructure.BitOrder = LL_SPI_MSB_FIRST;
  SPI_InitStructure.CRCCalculation = LL_SPI_CRCCALCULATION_DISABLE;
  SPI_InitStructure.CRCPoly = 7;
  LL_SPI_Init(PORT.channel, &SPI_InitStructure);
  LL_SPI_Enable(PORT.channel);

  spi_init_dev(SX12XX_SPI_PORT);
}

void sx127x_handle_exti() {
  if (LL_EXTI_IsActiveFlag_0_31(EXTI_LINE) != RESET) {
    LL_EXTI_ClearFlag_0_31(EXTI_LINE);

    if (gpio_pin_read(SX12XX_DIO0_PIN)) {
      dio0_active = 1;
    }
  }
}

uint8_t sx127x_read_dio0() {
  if (dio0_active) {
    dio0_active = 0;
    return 1;
  }
  return 0;
}

uint8_t sx127x_read_reg(uint8_t reg) {
  spi_csn_enable(SX12XX_NSS_PIN);
  spi_transfer_byte(SX12XX_SPI_PORT, reg | SX127x_READ);
  const uint8_t ret = spi_transfer_byte(SX12XX_SPI_PORT, 0xFF);
  spi_csn_disable(SX12XX_NSS_PIN);
  return ret;
}

uint8_t sx127x_write_reg(uint8_t reg, uint8_t data) {
  spi_csn_enable(SX12XX_NSS_PIN);
  spi_transfer_byte(SX12XX_SPI_PORT, reg | SX127x_WRITE);
  const uint8_t ret = spi_transfer_byte(SX12XX_SPI_PORT, data);
  spi_csn_disable(SX12XX_NSS_PIN);
  return ret;
}

uint8_t sx127x_set_reg(uint8_t reg, uint8_t value, uint8_t msb, uint8_t lsb) {
  if ((msb > 7) || (lsb > 7) || (lsb > msb)) {
    return 0;
  }

  uint8_t current_value = sx127x_read_reg(reg);
  uint8_t mask = ~((0b11111111 << (msb + 1)) | (0b11111111 >> (8 - lsb)));
  uint8_t new_value = (current_value & ~mask) | (value & mask);
  return sx127x_write_reg(reg, new_value);
}

void sx127x_write_reg_burst(uint8_t reg, uint8_t *data, uint8_t size) {
  spi_csn_enable(SX12XX_NSS_PIN);

  spi_transfer_byte(SX12XX_SPI_PORT, reg | SX127x_WRITE);
  for (uint8_t i = 0; i < size; i++) {
    spi_transfer_byte(SX12XX_SPI_PORT, data[i]);
  }

  spi_csn_disable(SX12XX_NSS_PIN);
}

void sx127x_set_mode(sx127x_radio_op_modes_t mode) {
  sx127x_write_reg(SX127x_OP_MODE | SX127x_OPMODE_LORA, mode);
}

uint8_t sx127x_detect() {
  for (uint8_t tries = 0; tries < 10; tries++) {
    volatile uint8_t version = sx127x_read_reg(SX127x_VERSION);
    if (version == 0x12) {
      return 1;
    }
    time_delay_us(200);
  }
  return 0;
}

void sx127x_read_fifo(uint8_t *data, uint8_t size) {
  spi_csn_enable(SX12XX_NSS_PIN);

  spi_transfer_byte(SX12XX_SPI_PORT, SX127x_FIFO | SX127x_READ);
  for (uint8_t i = 0; i < size; i++) {
    data[i] = spi_transfer_byte(SX12XX_SPI_PORT, 0xFF);
  }

  spi_csn_disable(SX12XX_NSS_PIN);
}

void sx127x_write_fifo(uint8_t *data, uint8_t size) {
  spi_csn_enable(SX12XX_NSS_PIN);

  spi_transfer_byte(SX12XX_SPI_PORT, SX127x_FIFO | SX127x_WRITE);
  for (uint8_t i = 0; i < size; i++) {
    spi_transfer_byte(SX12XX_SPI_PORT, data[i]);
  }

  spi_csn_disable(SX12XX_NSS_PIN);
}

#endif