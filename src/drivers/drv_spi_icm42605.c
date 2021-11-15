#include "drv_spi_icm42605.h"

#include "drv_gpio.h"
#include "drv_spi.h"
#include "drv_spi_gyro.h"
#include "drv_time.h"
#include "project.h"

#define PORT spi_port_defs[GYRO_SPI_PORT]

static void icm42605_reinit_slow() {
  spi_dma_wait_for_ready(GYRO_SPI_PORT);
  LL_SPI_Disable(PORT.channel);

  // SPI Config
  LL_SPI_DeInit(PORT.channel);
  LL_SPI_InitTypeDef spi_init;
  spi_init.TransferDirection = LL_SPI_FULL_DUPLEX;
  spi_init.Mode = LL_SPI_MODE_MASTER;
  spi_init.DataWidth = LL_SPI_DATAWIDTH_8BIT;
  spi_init.ClockPolarity = LL_SPI_POLARITY_HIGH;
  spi_init.ClockPhase = LL_SPI_PHASE_2EDGE;
  spi_init.NSS = LL_SPI_NSS_SOFT;
  spi_init.BaudRate = LL_SPI_BAUDRATEPRESCALER_DIV64;
  spi_init.BitOrder = LL_SPI_MSB_FIRST;
  spi_init.CRCCalculation = LL_SPI_CRCCALCULATION_DISABLE;
  spi_init.CRCPoly = 7;
  LL_SPI_Init(PORT.channel, &spi_init);

  LL_SPI_Enable(PORT.channel);
}

static void icm42605_reinit_fast(void) {
  spi_dma_wait_for_ready(GYRO_SPI_PORT);
  LL_SPI_Disable(PORT.channel);

  // SPI Config
  LL_SPI_DeInit(PORT.channel);
  LL_SPI_InitTypeDef spi_init;
  spi_init.TransferDirection = LL_SPI_FULL_DUPLEX;
  spi_init.Mode = LL_SPI_MODE_MASTER;
  spi_init.DataWidth = LL_SPI_DATAWIDTH_8BIT;
  spi_init.ClockPolarity = LL_SPI_POLARITY_HIGH;
  spi_init.ClockPhase = LL_SPI_PHASE_2EDGE;
  spi_init.NSS = LL_SPI_NSS_SOFT;
  spi_init.BaudRate = LL_SPI_BAUDRATEPRESCALER_DIV4;
  spi_init.BitOrder = LL_SPI_MSB_FIRST;
  spi_init.CRCCalculation = LL_SPI_CRCCALCULATION_DISABLE;
  spi_init.CRCPoly = 7;
  LL_SPI_Init(PORT.channel, &spi_init);

  LL_SPI_Enable(PORT.channel);
}

void icm42605_init() {

  spi_init_pins(GYRO_SPI_PORT, GYRO_NSS);

// Interrupt GPIO
#ifdef GYRO_INT
  LL_GPIO_InitTypeDef gpio_init;
  gpio_init.Mode = LL_GPIO_MODE_INPUT;
  gpio_init.OutputType = LL_GPIO_OUTPUT_PUSHPULL;
  gpio_init.Pull = LL_GPIO_PULL_UP;
  gpio_init.Speed = LL_GPIO_SPEED_FREQ_HIGH;
  gpio_pin_init(&gpio_init, GYRO_INT);
#endif

  spi_enable_rcc(GYRO_SPI_PORT);

  icm42605_reinit_slow();

  // Dummy read to clear receive buffer
  while (LL_SPI_IsActiveFlag_TXE(PORT.channel) == RESET)
    ;
  LL_SPI_ReceiveData8(PORT.channel);

  spi_dma_init(GYRO_SPI_PORT);
}

uint8_t icm42605_read(uint8_t reg) {
  icm42605_reinit_slow();

  uint8_t buffer[2] = {reg | 0x80, 0x00};

  spi_csn_enable(GYRO_NSS);
  spi_dma_transfer_bytes(GYRO_SPI_PORT, buffer, 2);
  spi_csn_disable(GYRO_NSS);

  return buffer[1];
}

void icm42605_write(uint8_t reg, uint8_t data) {
  icm42605_reinit_slow();

  uint8_t buffer[2] = {reg, data};

  spi_csn_enable(GYRO_NSS);
  spi_dma_transfer_bytes(GYRO_SPI_PORT, buffer, 2);
  spi_csn_disable(GYRO_NSS);
}

void icm42605_read_data(uint8_t reg, uint8_t *data, uint32_t size) {
  icm42605_reinit_fast();

  uint8_t buffer[size + 1];

  buffer[0] = reg | 0x80;
  for (uint32_t i = 0; i < size; i++) {
    buffer[i + 1] = 0x0;
  }

  spi_csn_enable(GYRO_NSS);
  spi_dma_transfer_bytes(GYRO_SPI_PORT, buffer, size + 1);
  spi_csn_disable(GYRO_NSS);

  for (int i = 1; i < size + 1; i++) {
    data[i - 1] = buffer[i];
  }
}