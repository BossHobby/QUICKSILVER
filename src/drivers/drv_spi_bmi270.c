#include "drv_spi_bmi270.h"

#include "drv_gpio.h"
#include "drv_spi.h"
#include "drv_spi_gyro.h"
#include "drv_time.h"
#include "project.h"

#define BMI270_ID 0x24

#define PORT spi_port_defs[GYRO_SPI_PORT]

const uint8_t bmi270_maximum_fifo_config_file[] = {
    0xc8, 0x2e, 0x00, 0x2e, 0x80, 0x2e, 0x1a, 0x00, 0xc8, 0x2e, 0x00, 0x2e, 0xc8, 0x2e, 0x00, 0x2e, 0xc8, 0x2e, 0x00,
    0x2e, 0xc8, 0x2e, 0x00, 0x2e, 0xc8, 0x2e, 0x00, 0x2e, 0xc8, 0x2e, 0x00, 0x2e, 0x90, 0x32, 0x21, 0x2e, 0x59, 0xf5,
    0x10, 0x30, 0x21, 0x2e, 0x6a, 0xf5, 0x1a, 0x24, 0x22, 0x00, 0x80, 0x2e, 0x3b, 0x00, 0xc8, 0x2e, 0x44, 0x47, 0x22,
    0x00, 0x37, 0x00, 0xa4, 0x00, 0xff, 0x0f, 0xd1, 0x00, 0x07, 0xad, 0x80, 0x2e, 0x00, 0xc1, 0x80, 0x2e, 0x00, 0xc1,
    0x80, 0x2e, 0x00, 0xc1, 0x80, 0x2e, 0x00, 0xc1, 0x80, 0x2e, 0x00, 0xc1, 0x80, 0x2e, 0x00, 0xc1, 0x80, 0x2e, 0x00,
    0xc1, 0x80, 0x2e, 0x00, 0xc1, 0x80, 0x2e, 0x00, 0xc1, 0x80, 0x2e, 0x00, 0xc1, 0x80, 0x2e, 0x00, 0xc1, 0x00, 0x00,
    0x00, 0x00, 0x00, 0x00, 0x11, 0x24, 0xfc, 0xf5, 0x80, 0x30, 0x40, 0x42, 0x50, 0x50, 0x00, 0x30, 0x12, 0x24, 0xeb,
    0x00, 0x03, 0x30, 0x00, 0x2e, 0xc1, 0x86, 0x5a, 0x0e, 0xfb, 0x2f, 0x21, 0x2e, 0xfc, 0xf5, 0x13, 0x24, 0x63, 0xf5,
    0xe0, 0x3c, 0x48, 0x00, 0x22, 0x30, 0xf7, 0x80, 0xc2, 0x42, 0xe1, 0x7f, 0x3a, 0x25, 0xfc, 0x86, 0xf0, 0x7f, 0x41,
    0x33, 0x98, 0x2e, 0xc2, 0xc4, 0xd6, 0x6f, 0xf1, 0x30, 0xf1, 0x08, 0xc4, 0x6f, 0x11, 0x24, 0xff, 0x03, 0x12, 0x24,
    0x00, 0xfc, 0x61, 0x09, 0xa2, 0x08, 0x36, 0xbe, 0x2a, 0xb9, 0x13, 0x24, 0x38, 0x00, 0x64, 0xbb, 0xd1, 0xbe, 0x94,
    0x0a, 0x71, 0x08, 0xd5, 0x42, 0x21, 0xbd, 0x91, 0xbc, 0xd2, 0x42, 0xc1, 0x42, 0x00, 0xb2, 0xfe, 0x82, 0x05, 0x2f,
    0x50, 0x30, 0x21, 0x2e, 0x21, 0xf2, 0x00, 0x2e, 0x00, 0x2e, 0xd0, 0x2e, 0xf0, 0x6f, 0x02, 0x30, 0x02, 0x42, 0x20,
    0x26, 0xe0, 0x6f, 0x02, 0x31, 0x03, 0x40, 0x9a, 0x0a, 0x02, 0x42, 0xf0, 0x37, 0x05, 0x2e, 0x5e, 0xf7, 0x10, 0x08,
    0x12, 0x24, 0x1e, 0xf2, 0x80, 0x42, 0x83, 0x84, 0xf1, 0x7f, 0x0a, 0x25, 0x13, 0x30, 0x83, 0x42, 0x3b, 0x82, 0xf0,
    0x6f, 0x00, 0x2e, 0x00, 0x2e, 0xd0, 0x2e, 0x12, 0x40, 0x52, 0x42, 0x00, 0x2e, 0x12, 0x40, 0x52, 0x42, 0x3e, 0x84,
    0x00, 0x40, 0x40, 0x42, 0x7e, 0x82, 0xe1, 0x7f, 0xf2, 0x7f, 0x98, 0x2e, 0x6a, 0xd6, 0x21, 0x30, 0x23, 0x2e, 0x61,
    0xf5, 0xeb, 0x2c, 0xe1, 0x6f};

static void bmi270_reinit_slow() {
  spi_dma_wait_for_ready(GYRO_SPI_PORT);
  LL_SPI_Disable(PORT.channel);

  LL_SPI_DeInit(PORT.channel);
  LL_SPI_InitTypeDef spi_init;
  spi_init.TransferDirection = LL_SPI_FULL_DUPLEX;
  spi_init.Mode = LL_SPI_MODE_MASTER;
  spi_init.DataWidth = LL_SPI_DATAWIDTH_8BIT;
  spi_init.ClockPolarity = LL_SPI_POLARITY_HIGH;
  spi_init.ClockPhase = LL_SPI_PHASE_2EDGE;
  spi_init.NSS = LL_SPI_NSS_SOFT;
  spi_init.BaudRate = spi_find_divder(MHZ_TO_HZ(0.5));
  spi_init.BitOrder = LL_SPI_MSB_FIRST;
  spi_init.CRCCalculation = LL_SPI_CRCCALCULATION_DISABLE;
  spi_init.CRCPoly = 7;
  LL_SPI_Init(PORT.channel, &spi_init);

  LL_SPI_Enable(PORT.channel);
}

static void bmi270_reinit_fast(void) {
  spi_dma_wait_for_ready(GYRO_SPI_PORT);
  LL_SPI_Disable(PORT.channel);

  LL_SPI_DeInit(PORT.channel);
  LL_SPI_InitTypeDef spi_init;
  spi_init.TransferDirection = LL_SPI_FULL_DUPLEX;
  spi_init.Mode = LL_SPI_MODE_MASTER;
  spi_init.DataWidth = LL_SPI_DATAWIDTH_8BIT;
  spi_init.ClockPolarity = LL_SPI_POLARITY_HIGH;
  spi_init.ClockPhase = LL_SPI_PHASE_2EDGE;
  spi_init.NSS = LL_SPI_NSS_SOFT;
  spi_init.BaudRate = spi_find_divder(MHZ_TO_HZ(21));
  spi_init.BitOrder = LL_SPI_MSB_FIRST;
  spi_init.CRCCalculation = LL_SPI_CRCCALCULATION_DISABLE;
  spi_init.CRCPoly = 7;
  LL_SPI_Init(PORT.channel, &spi_init);

  LL_SPI_Enable(PORT.channel);
}

static void bmi270_init() {

  // put the device in spi mode by toggeling CS
  gpio_pin_reset(GYRO_NSS);
  time_delay_ms(1);
  gpio_pin_set(GYRO_NSS);
  time_delay_ms(10);

  spi_init_pins(GYRO_SPI_PORT, GYRO_NSS);

#ifdef GYRO_INT
  LL_GPIO_InitTypeDef gpio_init;
  gpio_init.Mode = LL_GPIO_MODE_INPUT;
  gpio_init.OutputType = LL_GPIO_OUTPUT_PUSHPULL;
  gpio_init.Pull = LL_GPIO_PULL_UP;
  gpio_init.Speed = LL_GPIO_SPEED_FREQ_HIGH;
  gpio_pin_init(&gpio_init, GYRO_INT);
#endif

  spi_enable_rcc(GYRO_SPI_PORT);
  bmi270_reinit_slow();
  spi_init_dev(GYRO_SPI_PORT);
}

uint8_t bmi270_detect() {
  bmi270_init();

  const uint8_t id = bmi270_read(BMI270_REG_CHIP_ID);
  switch (id) {
  case BMI270_ID:
    return GYRO_TYPE_BMI270;

  default:
    return GYRO_TYPE_INVALID;
  }
}

void bmi270_configure() {
  bmi270_init();

  bmi270_write(BMI270_REG_CMD, BMI270_CMD_SOFTRESET);
  time_delay_ms(100);

  // put the device in spi mode by toggeling CS
  gpio_pin_reset(GYRO_NSS);
  time_delay_ms(1);
  gpio_pin_set(GYRO_NSS);
  time_delay_ms(10);

  bmi270_write(BMI270_REG_PWR_CONF, 0x0);
  time_delay_ms(1);

  bmi270_write(BMI270_REG_INIT_CTRL, 0x0);
  time_delay_ms(1);

  bmi270_write_data(BMI270_REG_INIT_DATA, (uint8_t *)bmi270_maximum_fifo_config_file, sizeof(bmi270_maximum_fifo_config_file));
  time_delay_ms(10);

  bmi270_write(BMI270_REG_INIT_CTRL, 0x1);
  time_delay_ms(1);

  bmi270_write(BMI270_REG_ACC_CONF, (BMI270_ACC_CONF_HP << 7) | (BMI270_ACC_CONF_BWP << 4) | BMI270_ACC_CONF_ODR800);
  time_delay_ms(1);

  bmi270_write(BMI270_REG_ACC_RANGE, BMI270_ACC_RANGE_16G);
  time_delay_ms(1);

  bmi270_write(BMI270_REG_GYRO_CONF, (BMI270_GYRO_CONF_FILTER_PERF << 7) | (BMI270_GYRO_CONF_NOISE_PERF << 6) | (BMI270_GYRO_CONF_BWP << 4) | BMI270_GYRO_CONF_ODR3200);
  time_delay_ms(1);

  bmi270_write(BMI270_REG_GYRO_RANGE, BMI270_GYRO_RANGE_2000DPS);
  time_delay_ms(1);

  bmi270_write(BMI270_REG_INT_MAP_DATA, BMI270_INT_MAP_DATA_DRDY_INT1);
  time_delay_ms(1);

  bmi270_write(BMI270_REG_INT1_IO_CTRL, BMI270_INT1_IO_CTRL_PINMODE);
  time_delay_ms(1);

  bmi270_write(BMI270_REG_PWR_CONF, BMI270_PWR_CONF);
  time_delay_ms(1);

  bmi270_write(BMI270_REG_PWR_CTRL, BMI270_PWR_CTRL);
  time_delay_ms(1);
}

uint8_t bmi270_read(uint8_t reg) {
  bmi270_reinit_slow();

  uint8_t buffer[3] = {reg | 0x80, 0x0, 0x0};

  spi_csn_enable(GYRO_NSS);
  spi_dma_transfer_bytes(GYRO_SPI_PORT, buffer, 3);
  spi_csn_disable(GYRO_NSS);

  return buffer[2];
}

void bmi270_write(uint8_t reg, uint8_t data) {
  bmi270_reinit_slow();

  uint8_t buffer[2] = {reg, data};

  spi_csn_enable(GYRO_NSS);
  spi_dma_transfer_bytes(GYRO_SPI_PORT, buffer, 2);
  spi_csn_disable(GYRO_NSS);
}

void bmi270_write_data(uint8_t reg, uint8_t *data, uint32_t size) {
  bmi270_reinit_slow();

  uint8_t buffer[size + 1];

  buffer[0] = reg;
  for (uint32_t i = 0; i < size; i++) {
    buffer[i + 1] = data[i];
  }

  spi_csn_enable(GYRO_NSS);
  spi_dma_transfer_bytes(GYRO_SPI_PORT, buffer, size + 1);
  spi_csn_disable(GYRO_NSS);
}

void bmi270_read_data(uint8_t reg, uint8_t *data, uint32_t size) {
  bmi270_reinit_fast();

  uint8_t buffer[size + 2];

  buffer[0] = reg | 0x80;
  for (uint32_t i = 0; i < (size + 1); i++) {
    buffer[i + 1] = 0x0;
  }

  spi_csn_enable(GYRO_NSS);
  spi_dma_transfer_bytes(GYRO_SPI_PORT, buffer, size + 2);
  spi_csn_disable(GYRO_NSS);

  for (int i = 2; i < size + 2; i++) {
    data[i - 2] = buffer[i];
  }
}