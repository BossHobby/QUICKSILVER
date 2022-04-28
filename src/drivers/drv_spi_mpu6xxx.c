#include "drv_spi_mpu6xxx.h"

#include <stdio.h>

#include "drv_gpio.h"
#include "drv_spi.h"
#include "drv_spi_gyro.h"
#include "drv_time.h"
#include "project.h"

#define MPU6000_ID (0x68)
#define MPU6500_ID (0x70)

#define ICM20601_ID (0xAC)
#define ICM20602_ID (0x12)
#define ICM20608_ID (0xAF)
#define ICM20649_ID (0xE1)
#define ICM20689_ID (0x98)

#define SPI_SPEED_INIT MHZ_TO_HZ(0.5)

#define PORT spi_port_defs[GYRO_SPI_PORT]

static void mpu6xxx_reinit_slow() {
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
  spi_init.BaudRate = spi_find_divder(SPI_SPEED_INIT);
  spi_init.BitOrder = LL_SPI_MSB_FIRST;
  spi_init.CRCCalculation = LL_SPI_CRCCALCULATION_DISABLE;
  spi_init.CRCPoly = 7;
  LL_SPI_Init(PORT.channel, &spi_init);

  LL_SPI_Enable(PORT.channel);
}

static void mpu6xxx_reinit_fast() {
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

  switch (gyro_type) {
  default:
  case GYRO_TYPE_ICM20649:
    spi_init.BaudRate = spi_find_divder(MHZ_TO_HZ(7));
    break;

  case GYRO_TYPE_ICM20601:
  case GYRO_TYPE_ICM20608:
  case GYRO_TYPE_ICM20689:
    spi_init.BaudRate = spi_find_divder(MHZ_TO_HZ(8));
    break;

  case GYRO_TYPE_ICM20602:
    spi_init.BaudRate = spi_find_divder(MHZ_TO_HZ(10.5));
    break;

  case GYRO_TYPE_MPU6000:
  case GYRO_TYPE_MPU6500:
    spi_init.BaudRate = spi_find_divder(MHZ_TO_HZ(21));
    break;
  }

  spi_init.BitOrder = LL_SPI_MSB_FIRST;
  spi_init.CRCCalculation = LL_SPI_CRCCALCULATION_DISABLE;
  spi_init.CRCPoly = 7;
  LL_SPI_Init(PORT.channel, &spi_init);

  LL_SPI_Enable(PORT.channel);
}

static void mpu6xxx_init() {

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
  mpu6xxx_reinit_slow();
  spi_init_dev(GYRO_SPI_PORT);
}

uint8_t mpu6xxx_detect() {
  mpu6xxx_init();

  const uint8_t id = mpu6xxx_read(MPU_RA_WHO_AM_I);
  switch (id) {
  case MPU6000_ID:
    return GYRO_TYPE_MPU6000;
  case MPU6500_ID:
    return GYRO_TYPE_MPU6500;
  case ICM20601_ID:
    return GYRO_TYPE_ICM20601;
  case ICM20602_ID:
    return GYRO_TYPE_ICM20602;
  case ICM20608_ID:
    return GYRO_TYPE_ICM20608;
  case ICM20649_ID:
    return GYRO_TYPE_ICM20649;
  case ICM20689_ID:
    return GYRO_TYPE_ICM20689;
  default:
    return GYRO_TYPE_INVALID;
  }
}

void mpu6xxx_configure() {
  mpu6xxx_init();

  mpu6xxx_write(MPU_RA_PWR_MGMT_1, MPU_BIT_H_RESET); // reg 107 soft reset  MPU_BIT_H_RESET
  time_delay_ms(100);
  mpu6xxx_write(MPU_RA_SIGNAL_PATH_RESET, MPU_RESET_SIGNAL_PATHWAYS);
  time_delay_ms(100);
  mpu6xxx_write(MPU_RA_PWR_MGMT_1, MPU_CLK_SEL_PLLGYROX); // reg 107 set pll clock to 1 for x axis reference
  time_delay_ms(100);
  mpu6xxx_write(MPU_RA_USER_CTRL, MPU_BIT_I2C_IF_DIS); // reg 106 to 16 enabling spi
  time_delay_us(1500);
  mpu6xxx_write(MPU_RA_PWR_MGMT_2, MPU_BITS_STDBY_MODE_OFF); // reg 108 disable standbye mode to 0
  time_delay_us(1500);
  mpu6xxx_write(MPU_RA_SMPLRT_DIV, MPU_BIT_SMPLRT_DIVIDER_OFF); // reg 25 sample rate divider to 0
  time_delay_us(1500);
  mpu6xxx_write(MPU_RA_CONFIG, MPU_BITS_DLPF_CFG_256HZ); // reg 26 dlpf to 0 - 8khz
  time_delay_us(1500);
  mpu6xxx_write(MPU_RA_ACCEL_CONFIG, MPU_BITS_FS_16G); // reg 28 accel scale to 16G
  time_delay_us(1500);
  mpu6xxx_write(MPU_RA_GYRO_CONFIG, MPU_BITS_FS_2000DPS); // reg 27 gyro scale to 2000deg/s
  time_delay_us(1500);
  mpu6xxx_write(MPU_RA_INT_ENABLE, MPU_BIT_INT_STATUS_DATA); // reg 56 data ready enable interrupt to 1
  time_delay_us(1500);
}

// blocking dma read of a single register
uint8_t mpu6xxx_read(uint8_t reg) {
  mpu6xxx_reinit_slow();

  uint8_t buffer[2] = {reg | 0x80, 0x00};

  spi_csn_enable(GYRO_NSS);
  spi_dma_transfer_bytes(GYRO_SPI_PORT, buffer, 2);
  spi_csn_disable(GYRO_NSS);

  return buffer[1];
}

// blocking dma write of a single register
void mpu6xxx_write(uint8_t reg, uint8_t data) {
  mpu6xxx_reinit_slow();

  uint8_t buffer[2] = {reg, data};

  spi_csn_enable(GYRO_NSS);
  spi_dma_transfer_bytes(GYRO_SPI_PORT, buffer, 2);
  spi_csn_disable(GYRO_NSS);
}

void mpu6xxx_read_data(uint8_t reg, uint8_t *data, uint32_t size) {
  mpu6xxx_reinit_fast();

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
