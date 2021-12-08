#include "drv_spi_sx12xx.h"

#include <stm32f4xx_ll_bus.h>
#include <stm32f4xx_ll_exti.h>
#include <stm32f4xx_ll_spi.h>
#include <stm32f4xx_ll_system.h>

#include "drv_spi.h"
#include "drv_time.h"
#include "project.h"

#define PORT spi_port_defs[SX12XX_SPI_PORT]

#define EXTI_LINE LL_EXTI_LINE_9

void sx12xx_init() {
  spi_init_pins(SX12XX_SPI_PORT, SX12XX_NSS_PIN);

  LL_GPIO_InitTypeDef gpio_init;
  gpio_init.Mode = LL_GPIO_MODE_INPUT;
  gpio_init.Speed = LL_GPIO_SPEED_FREQ_VERY_HIGH;
  gpio_init.OutputType = LL_GPIO_OUTPUT_OPENDRAIN;
  gpio_init.Pull = LL_GPIO_PULL_NO;
  gpio_pin_init(&gpio_init, SX12XX_DIO0_PIN);

  LL_APB2_GRP1_EnableClock(LL_APB2_GRP1_PERIPH_SYSCFG);
  LL_SYSCFG_SetEXTISource(LL_SYSCFG_EXTI_PORTA, LL_SYSCFG_EXTI_LINE9);

  LL_EXTI_InitTypeDef exti_init;
  exti_init.Line_0_31 = EXTI_LINE;
  exti_init.LineCommand = ENABLE;
  exti_init.Mode = LL_EXTI_MODE_IT;
  exti_init.Trigger = LL_EXTI_TRIGGER_RISING;
  LL_EXTI_Init(&exti_init);

  LL_EXTI_EnableIT_0_31(EXTI_LINE);

  NVIC_EnableIRQ(EXTI9_5_IRQn);
  NVIC_SetPriority(EXTI9_5_IRQn, 0);

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

  // Dummy read to clear receive buffer
  while (LL_SPI_IsActiveFlag_TXE(PORT.channel) == RESET)
    ;
  LL_SPI_ReceiveData8(PORT.channel);
}

static volatile uint8_t dio0_active = 0;
static volatile uint32_t packet_time = 0;

void EXTI9_5_IRQHandler() {
  if (LL_EXTI_IsActiveFlag_0_31(EXTI_LINE) != RESET) {
    LL_EXTI_ClearFlag_0_31(EXTI_LINE);
    dio0_active = 1;
  }
}

uint8_t sx12xx_read_dio0() {
  if (dio0_active) {
    dio0_active = 0;
    return 1;
  }
  return 0;
}

uint32_t sx12xx_packet_time() {
  return packet_time;
}

uint8_t sx12xx_read_reg(uint8_t reg) {
  spi_csn_enable(SX12XX_NSS_PIN);
  spi_transfer_byte(SX12XX_SPI_PORT, reg | SX127x_READ);
  const uint8_t ret = spi_transfer_byte(SX12XX_SPI_PORT, 0xFF);
  spi_csn_disable(SX12XX_NSS_PIN);
  return ret;
}

uint8_t sx12xx_write_reg(uint8_t reg, uint8_t data) {
  spi_csn_enable(SX12XX_NSS_PIN);
  spi_transfer_byte(SX12XX_SPI_PORT, reg | SX127x_WRITE);
  const uint8_t ret = spi_transfer_byte(SX12XX_SPI_PORT, data);
  spi_csn_disable(SX12XX_NSS_PIN);
  return ret;
}

uint8_t sx12xx_set_reg(uint8_t reg, uint8_t value, uint8_t msb, uint8_t lsb) {
  if ((msb > 7) || (lsb > 7) || (lsb > msb)) {
    return 0;
  }

  uint8_t current_value = sx12xx_read_reg(reg);
  uint8_t mask = ~((0b11111111 << (msb + 1)) | (0b11111111 >> (8 - lsb)));
  uint8_t new_value = (current_value & ~mask) | (value & mask);
  return sx12xx_write_reg(reg, new_value);
}

void sx12xx_write_reg_burst(uint8_t reg, uint8_t *data, uint8_t size) {
  spi_csn_enable(SX12XX_NSS_PIN);

  spi_transfer_byte(SX12XX_SPI_PORT, reg | SX127x_WRITE);
  for (uint8_t i = 0; i < size; i++) {
    spi_transfer_byte(SX12XX_SPI_PORT, data[i]);
  }

  spi_csn_disable(SX12XX_NSS_PIN);
}

void sx12xx_set_mode(sx12xx_radio_op_modes_t mode) {
  sx12xx_write_reg(SX127x_OP_MODE | SX127x_OPMODE_LORA, mode);
}

uint8_t sx12xx_detect() {
  for (uint8_t tries = 0; tries < 10; tries++) {
    volatile uint8_t version = sx12xx_read_reg(SX127x_VERSION);
    if (version == 0x12) {
      return 1;
    }
    time_delay_us(200);
  }
  return 0;
}

void sx12xx_read_fifo(uint8_t *data, uint8_t size) {
  spi_csn_enable(SX12XX_NSS_PIN);

  spi_transfer_byte(SX12XX_SPI_PORT, SX127x_FIFO | SX127x_READ);
  for (uint8_t i = 0; i < size; i++) {
    data[i] = spi_transfer_byte(SX12XX_SPI_PORT, 0xFF);
  }

  spi_csn_disable(SX12XX_NSS_PIN);
}

void sx12xx_write_fifo(uint8_t *data, uint8_t size) {
  spi_csn_enable(SX12XX_NSS_PIN);

  spi_transfer_byte(SX12XX_SPI_PORT, SX127x_FIFO | SX127x_WRITE);
  for (uint8_t i = 0; i < size; i++) {
    spi_transfer_byte(SX12XX_SPI_PORT, data[i]);
  }

  spi_csn_disable(SX12XX_NSS_PIN);
}