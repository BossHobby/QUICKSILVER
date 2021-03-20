#include "drv_spi_cc2500.h"

#include <stm32f4xx_ll_spi.h>

#include "drv_spi.h"
#include "drv_time.h"
#include "project.h"

#if defined(F4) && defined(USE_CC2500)

#define PORT spi_port_defs[CC2500_SPI_PORT]

uint8_t cc2500_read_gdo0() {
  return gpio_pin_read(CC2500_GDO0_PIN);
}

static void cc2500_hardware_init() {
  spi_init_pins(CC2500_SPI_PORT, CC2500_NSS);

  LL_GPIO_InitTypeDef gpio_init;
  gpio_init.Mode = LL_GPIO_MODE_OUTPUT;
  gpio_init.Speed = LL_GPIO_SPEED_FREQ_HIGH;
  gpio_init.OutputType = LL_GPIO_OUTPUT_PUSHPULL;
  gpio_init.Pull = LL_GPIO_PULL_NO;

#if defined(USE_CC2500_PA_LNA)
  // turn antenna on

#if defined(CC2500_LNA_EN_PIN)
  gpio_pin_init(&gpio_init, CC2500_LNA_EN_PIN);
  gpio_pin_set(CC2500_LNA_EN_PIN);
#endif

  // turn tx off
  gpio_pin_init(&gpio_init, CC2500_TX_EN_PIN);
  gpio_pin_reset(CC2500_TX_EN_PIN);
#if defined(USE_CC2500_DIVERSITY)
  // choose b?
  gpio_pin_init(&gpio_init, CC2500_ANT_SEL_PIN);
  gpio_pin_set(CC2500_ANT_SEL_PIN);
#endif

#endif // USE_CC2500_PA_LNA

  // GDO0
  gpio_init.Mode = LL_GPIO_MODE_INPUT;
  gpio_init.Speed = LL_GPIO_SPEED_FREQ_HIGH;
  gpio_init.OutputType = LL_GPIO_OUTPUT_OPENDRAIN;
  gpio_init.Pull = LL_GPIO_PULL_DOWN;
  gpio_pin_init(&gpio_init, CC2500_GDO0_PIN);

  spi_enable_rcc(CC2500_SPI_PORT);

  LL_SPI_DeInit(PORT.channel);
  LL_SPI_InitTypeDef SPI_InitStructure;
  SPI_InitStructure.TransferDirection = LL_SPI_FULL_DUPLEX;
  SPI_InitStructure.Mode = LL_SPI_MODE_MASTER;
  SPI_InitStructure.DataWidth = LL_SPI_DATAWIDTH_8BIT;
  SPI_InitStructure.ClockPolarity = LL_SPI_POLARITY_LOW;
  SPI_InitStructure.ClockPhase = LL_SPI_PHASE_1EDGE;
  SPI_InitStructure.NSS = LL_SPI_NSS_SOFT;
  SPI_InitStructure.BaudRate = LL_SPI_BAUDRATEPRESCALER_DIV4;
  SPI_InitStructure.BitOrder = LL_SPI_MSB_FIRST;
  SPI_InitStructure.CRCCalculation = LL_SPI_CRCCALCULATION_DISABLE;
  SPI_InitStructure.CRCPoly = 7;
  LL_SPI_Init(PORT.channel, &SPI_InitStructure);
  LL_SPI_Enable(PORT.channel);

  // Dummy read to clear receive buffer
  while (LL_SPI_IsActiveFlag_TXE(PORT.channel) == RESET)
    ;
  LL_SPI_ReceiveData8(PORT.channel);

  spi_dma_init(CC2500_SPI_PORT);
}

void cc2500_strobe(uint8_t address) {
  spi_csn_enable(CC2500_NSS);
  spi_transfer_byte(CC2500_SPI_PORT, address);
  spi_csn_disable(CC2500_NSS);
}

uint8_t cc2500_get_status() {
  spi_csn_enable(CC2500_NSS);
  const uint8_t status = spi_transfer_byte(CC2500_SPI_PORT, 0xFF);
  spi_csn_disable(CC2500_NSS);
  return status;
}

uint8_t cc2500_write_reg(uint8_t reg, uint8_t data) {
  spi_csn_enable(CC2500_NSS);
  spi_transfer_byte(CC2500_SPI_PORT, reg | CC2500_WRITE_SINGLE);
  const uint8_t ret = spi_transfer_byte(CC2500_SPI_PORT, data);
  spi_csn_disable(CC2500_NSS);
  return ret;
}

uint8_t cc2500_read_reg(uint8_t reg) {
  return cc2500_write_reg(reg | CC2500_READ_SINGLE, 0xFF);
}

static uint8_t cc2500_read_multi(uint8_t reg, uint8_t data, uint8_t *result, uint8_t len) {
  spi_csn_enable(CC2500_NSS);

  uint8_t buffer[len + 1];
  buffer[0] = reg;
  for (uint8_t i = 0; i < len; i++) {
    buffer[i + 1] = data;
  }

  spi_dma_transfer_bytes(CC2500_SPI_PORT, buffer, len + 1);
  for (uint8_t i = 0; i < len; i++) {
    result[i] = buffer[i + 1];
  }

  spi_csn_disable(CC2500_NSS);
  return buffer[0];
}

/*
static uint8_t cc2500_write_multi(uint8_t reg, uint8_t *data, uint8_t len) {
  spi_csn_enable(CC2500_NSS);

  uint8_t buffer[len + 1];
  buffer[0] = reg;
  for (uint8_t i = 0; i < len; i++) {
    buffer[i + 1] = data[i];
  }

  spi_dma_transfer_bytes(CC2500_SPI_PORT, buffer, len + 1);
  spi_csn_disable(CC2500_NSS);
  return buffer[0];
}
*/

static uint8_t cc2500_write_multi(uint8_t reg, uint8_t *data, uint8_t len) {
  spi_csn_enable(CC2500_NSS);

  const uint8_t ret = spi_transfer_byte(CC2500_SPI_PORT, reg);
  for (uint8_t i = 0; i < len; i++) {
    spi_transfer_byte(CC2500_SPI_PORT, data[i]);
  }

  spi_csn_disable(CC2500_NSS);
  return ret;
}

uint8_t cc2500_read_fifo(uint8_t *result, uint8_t len) {
  return cc2500_read_multi(CC2500_FIFO | CC2500_READ_BURST, 0xFF, result, len);
}

uint8_t cc2500_write_fifo(uint8_t *data, uint8_t len) {
  // flush tx fifo
  cc2500_strobe(CC2500_SFTX);

  const uint8_t ret = cc2500_write_multi(CC2500_FIFO | CC2500_WRITE_BURST, data, len);

  // and send!
  cc2500_strobe(CC2500_STX);

  return ret;
}

void cc2500_reset() {
  cc2500_strobe(CC2500_SRES);
  timer_delay_us(1000); // 1000us
  cc2500_strobe(CC2500_SIDLE);
}

void cc2500_init() {
  cc2500_hardware_init();
  cc2500_reset();
}

void cc2500_switch_antenna() {
#if defined(USE_CC2500_PA_LNA) && defined(USE_CC2500_DIVERSITY)
  static uint8_t alternative_selected = 1;
  if (alternative_selected == 1) {
    gpio_pin_reset(CC2500_ANT_SEL_PIN);
  } else {
    gpio_pin_set(CC2500_ANT_SEL_PIN);
  }
  alternative_selected = alternative_selected ? 0 : 1;
#endif
}

void cc2500_enter_rxmode() {
#if defined(USE_CC2500_PA_LNA)

#if defined(CC2500_LNA_EN_PIN)
  gpio_pin_set(CC2500_LNA_EN_PIN);
#endif

  gpio_pin_reset(CC2500_TX_EN_PIN);
#endif
}

void cc2500_enter_txmode() {
#if defined(USE_CC2500_PA_LNA)

#if defined(CC2500_LNA_EN_PIN)
  gpio_pin_reset(CC2500_LNA_EN_PIN);
#endif

  gpio_pin_set(CC2500_TX_EN_PIN);
#endif
}

void cc2500_set_power(uint8_t power) {
  static const uint8_t patable[8] = {
      0xC5, // -12dbm
      0x97, // -10dbm
      0x6E, // -8dbm
      0x7F, // -6dbm
      0xA9, // -4dbm
      0xBB, // -2dbm
      0xFE, // 0dbm
      0xFF  // 1.5dbm
  };

  if (power > 7)
    power = 7;

  cc2500_write_reg(CC2500_PATABLE, patable[power]);
}

#endif