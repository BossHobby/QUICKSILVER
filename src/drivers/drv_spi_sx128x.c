#include "drv_spi_sx128x.h"

#include <stdbool.h>
#include <string.h>

#include <stm32f4xx_ll_bus.h>
#include <stm32f4xx_ll_exti.h>
#include <stm32f4xx_ll_spi.h>
#include <stm32f4xx_ll_system.h>

#include "drv_exti.h"
#include "drv_spi.h"
#include "drv_time.h"
#include "project.h"

#if defined(USE_SX128X)

#define PORT spi_port_defs[SX12XX_SPI_PORT]

static volatile uint8_t dio0_active = 0;

static uint32_t busy_timeout = 1000;

void sx128x_init() {
  spi_init_pins(SX12XX_SPI_PORT, SX12XX_NSS_PIN);

  LL_GPIO_InitTypeDef gpio_init;
  gpio_init.Mode = LL_GPIO_MODE_OUTPUT;
  gpio_init.Speed = LL_GPIO_SPEED_FREQ_HIGH;
  gpio_init.OutputType = LL_GPIO_OUTPUT_PUSHPULL;
  gpio_init.Pull = LL_GPIO_PULL_NO;
  gpio_pin_init(&gpio_init, SX12XX_RESET_PIN);

  gpio_init.Mode = LL_GPIO_MODE_INPUT;
  gpio_init.Speed = LL_GPIO_SPEED_FREQ_LOW;
  gpio_init.OutputType = LL_GPIO_OUTPUT_PUSHPULL;
  gpio_init.Pull = LL_GPIO_PULL_NO;
  gpio_pin_init(&gpio_init, SX12XX_BUSY_PIN);

  gpio_pin_init(&gpio_init, SX12XX_DIO0_PIN);
  exti_enable(SX12XX_DIO0_PIN, LL_EXTI_TRIGGER_RISING);

  spi_enable_rcc(SX12XX_SPI_PORT);

  LL_SPI_DeInit(PORT.channel);
  LL_SPI_InitTypeDef SPI_InitStructure;
  SPI_InitStructure.TransferDirection = LL_SPI_FULL_DUPLEX;
  SPI_InitStructure.Mode = LL_SPI_MODE_MASTER;
  SPI_InitStructure.DataWidth = LL_SPI_DATAWIDTH_8BIT;
  SPI_InitStructure.ClockPolarity = LL_SPI_POLARITY_LOW;
  SPI_InitStructure.ClockPhase = LL_SPI_PHASE_1EDGE;
  SPI_InitStructure.NSS = LL_SPI_NSS_SOFT;
  SPI_InitStructure.BaudRate = spi_find_divder(MHZ_TO_HZ(10.5));
  SPI_InitStructure.BitOrder = LL_SPI_MSB_FIRST;
  SPI_InitStructure.CRCCalculation = LL_SPI_CRCCALCULATION_DISABLE;
  SPI_InitStructure.CRCPoly = 7;
  LL_SPI_Init(PORT.channel, &SPI_InitStructure);
  LL_SPI_Enable(PORT.channel);

  // Dummy read to clear receive buffer
  while (LL_SPI_IsActiveFlag_TXE(PORT.channel) == RESET)
    ;
  LL_SPI_ReceiveData8(PORT.channel);

  spi_dma_init(SX12XX_SPI_PORT);
}

void sx128x_reset() {
  gpio_pin_reset(SX12XX_RESET_PIN);
  time_delay_us(50);
  gpio_pin_set(SX12XX_RESET_PIN);
  time_delay_us(10);
}

void sx128x_set_busy_timeout(uint32_t timeout) {
  busy_timeout = timeout;
}

void sx128x_handle_dio0_exti(bool level) {
  if (!level) {
    return;
  }
  dio0_active = 1;
}

void sx128x_handle_busy_exti(bool level) {
}

uint8_t sx128x_read_dio0() {
  register uint8_t active = 0;

  do {
    active = dio0_active;
  } while (active != dio0_active);

  if (active) {
    dio0_active = 0;
    return 1;
  }
  return 0;
}

static bool sx128x_is_busy() {
  return gpio_pin_read(SX12XX_BUSY_PIN);
}

static bool sx128x_wait_for_ready() {
  const uint32_t start = time_micros();
  while (sx128x_is_busy()) {
    if ((time_micros() - start) > busy_timeout) {
      return false;
    }
    __NOP();
  }
  return true;
}

void sx128x_read_register_burst(const uint16_t reg, uint8_t *data, const uint8_t size) {
  const uint8_t buf[4] = {
      (SX1280_RADIO_READ_REGISTER),
      ((reg & 0xFF00) >> 8),
      (reg & 0x00FF),
      0x00,
  };

  sx128x_wait_for_ready();

  spi_csn_enable(SX12XX_NSS_PIN);
  for (uint8_t i = 0; i < 4; i++) {
    spi_transfer_byte(SX12XX_SPI_PORT, buf[i]);
  }
  for (uint8_t i = 0; i < size; i++) {
    data[i] = spi_transfer_byte(SX12XX_SPI_PORT, 0xFF);
  }
  spi_csn_disable(SX12XX_NSS_PIN);
}

uint8_t sx128x_read_register(const uint16_t reg) {
  uint8_t data = 0;
  sx128x_read_register_burst(reg, &data, 1);
  return data;
}

void sx128x_write_register_burst(const uint16_t reg, const uint8_t *data, const uint8_t size) {
  uint8_t buf[size + 3];
  buf[0] = (uint8_t)SX1280_RADIO_WRITE_REGISTER;
  buf[1] = ((reg & 0xFF00) >> 8);
  buf[2] = (reg & 0x00FF);
  memcpy(buf + 3, data, size);

  sx128x_wait_for_ready();

  spi_csn_enable(SX12XX_NSS_PIN);
  spi_dma_transfer_bytes(SX12XX_SPI_PORT, buf, size + 3);
  spi_csn_disable(SX12XX_NSS_PIN);
}

void sx128x_write_register(const uint16_t reg, const uint8_t val) {
  sx128x_write_register_burst(reg, &val, 1);
}

void sx128x_read_command_burst(const sx128x_commands_t cmd, uint8_t *data, const uint8_t size) {
  uint8_t buf[size + 2];
  buf[0] = (uint8_t)cmd;
  buf[1] = 0x0;
  memset(buf + 2, 0xFF, size);

  sx128x_wait_for_ready();

  spi_csn_enable(SX12XX_NSS_PIN);
  spi_dma_transfer_bytes(SX12XX_SPI_PORT, buf, size + 2);
  spi_csn_disable(SX12XX_NSS_PIN);

  memcpy(data, buf + 2, size);
}

void sx128x_write_command(const sx128x_commands_t cmd, const uint8_t val) {
  sx128x_wait_for_ready();

  spi_csn_enable(SX12XX_NSS_PIN);
  spi_transfer_byte(SX12XX_SPI_PORT, (uint8_t)cmd);
  spi_transfer_byte(SX12XX_SPI_PORT, val);
  spi_csn_disable(SX12XX_NSS_PIN);
}

void sx128x_write_command_burst(const sx128x_commands_t cmd, const uint8_t *data, const uint8_t size) {
  uint8_t buf[size + 1];
  buf[0] = (uint8_t)cmd;
  memcpy(buf + 1, data, size);

  sx128x_wait_for_ready();

  spi_csn_enable(SX12XX_NSS_PIN);
  spi_dma_transfer_bytes(SX12XX_SPI_PORT, buf, size + 1);
  spi_csn_disable(SX12XX_NSS_PIN);
}

void sx128x_set_mode(const sx128x_modes_t mode) {
  switch (mode) {
  case SX1280_MODE_SLEEP:
    sx128x_write_command(SX1280_RADIO_SET_SLEEP, 0x01);
    break;

  case SX1280_MODE_CALIBRATION:
    break;

  case SX1280_MODE_STDBY_RC:
    sx128x_write_command(SX1280_RADIO_SET_STANDBY, SX1280_STDBY_RC);
    break;

  case SX1280_MODE_STDBY_XOSC:
    sx128x_write_command(SX1280_RADIO_SET_STANDBY, SX1280_STDBY_XOSC);
    break;

  case SX1280_MODE_FS:
    sx128x_write_command(SX1280_RADIO_SET_FS, 0x00);
    break;

  case SX1280_MODE_RX: {
    uint8_t buf[3];
    buf[0] = 0x00; // periodBase = 1ms, page 71 datasheet, set to FF for cont RX
    buf[1] = 0xFF;
    buf[2] = 0xFF;
    sx128x_write_command_burst(SX1280_RADIO_SET_RX, buf, 3);
    break;
  }

  case SX1280_MODE_TX: {
    uint8_t buf[3];
    //uses timeout Time-out duration = periodBase * periodBaseCount
    buf[0] = 0x00; // periodBase = 1ms, page 71 datasheet
    buf[1] = 0xFF; // no timeout set for now
    buf[2] = 0xFF; // TODO dynamic timeout based on expected onairtime
    sx128x_write_command_burst(SX1280_RADIO_SET_TX, buf, 3);
    break;
  }

  case SX1280_MODE_CAD:
  default:
    break;
  }
}

void sx128x_config_lora_mod_params(const sx128x_lora_bandwidths_t bw, const sx128x_lora_spreading_factors_t sf, const sx128x_lora_coding_rates_t cr) {
  // Care must therefore be taken to ensure that modulation parameters are set using the command
  // SetModulationParam() only after defining the packet type SetPacketType() to be used

  uint8_t rfparams[3] = {0};

  rfparams[0] = (uint8_t)sf;
  rfparams[1] = (uint8_t)bw;
  rfparams[2] = (uint8_t)cr;

  sx128x_write_command_burst(SX1280_RADIO_SET_MODULATIONPARAMS, rfparams, 3);

  switch (sf) {
  case SX1280_LORA_SF5:
  case SX1280_LORA_SF6:
    sx128x_write_register(0x925, 0x1E); // for SF5 or SF6
    break;
  case SX1280_LORA_SF7:
  case SX1280_LORA_SF8:
    sx128x_write_register(0x925, 0x37); // for SF7 or SF8
    break;
  default:
    sx128x_write_register(0x925, 0x32); // for SF9, SF10, SF11, SF12
  }
}

void sx128x_set_packet_params(const uint8_t preamble_length, const sx128x_lora_packet_lengths_modes_t header_type, const uint8_t payload_length, const sx128x_lora_crc_modes_t crc, const sx128x_lora_iq_modes_t invert_iq) {
  const uint8_t buf[7] = {
      preamble_length,
      header_type,
      payload_length,
      crc,
      invert_iq,
      0x00,
      0x00,
  };
  sx128x_write_command_burst(SX1280_RADIO_SET_PACKETPARAMS, buf, 7);
}

void sx128x_set_frequency(const uint32_t freq) {
  const uint8_t buf[3] = {
      (uint8_t)((freq >> 16) & 0xFF),
      (uint8_t)((freq >> 8) & 0xFF),
      (uint8_t)(freq & 0xFF),
  };
  sx128x_write_command_burst(SX1280_RADIO_SET_RFFREQUENCY, buf, 3);
}

void sx128x_set_fifo_addr(const uint8_t tx_base_addr, const uint8_t rx_base_addr) {
  const uint8_t buf[2] = {
      tx_base_addr,
      rx_base_addr,
  };
  sx128x_write_command_burst(SX1280_RADIO_SET_BUFFERBASEADDRESS, buf, 2);
}

void sx128x_set_dio_irq_params(const uint16_t irq_mask, const uint16_t dio1_mask, const uint16_t dio2_mask, const uint16_t dio3_mask) {
  const uint8_t buf[8] = {
      (uint8_t)((irq_mask >> 8) & 0x00FF),
      (uint8_t)(irq_mask & 0x00FF),
      (uint8_t)((dio1_mask >> 8) & 0x00FF),
      (uint8_t)(dio1_mask & 0x00FF),
      (uint8_t)((dio2_mask >> 8) & 0x00FF),
      (uint8_t)(dio2_mask & 0x00FF),
      (uint8_t)((dio3_mask >> 8) & 0x00FF),
      (uint8_t)(dio3_mask & 0x00FF),
  };
  sx128x_write_command_burst(SX1280_RADIO_SET_DIOIRQPARAMS, buf, 8);
}

void sx128x_clear_irq_status(const uint16_t irq_mask) {
  const uint8_t buf[2] = {
      (uint8_t)(((uint16_t)irq_mask >> 8) & 0x00FF),
      (uint8_t)((uint16_t)irq_mask & 0x00FF),
  };
  sx128x_write_command_burst(SX1280_RADIO_CLR_IRQSTATUS, buf, 2);
}

uint16_t sx128x_get_irq_status() {
  uint8_t status[2] = {0, 0};
  sx128x_read_command_burst(SX1280_RADIO_GET_IRQSTATUS, status, 2);
  return status[0] << 8 | status[1];
}

void sx128x_read_rx_buffer(uint8_t *data, const uint8_t size) {
  uint8_t buffer_status[2] = {0, 0};
  sx128x_read_command_burst(SX1280_RADIO_GET_RXBUFFERSTATUS, buffer_status, 2);

  uint8_t buf[size + 3];
  buf[0] = (uint8_t)SX1280_RADIO_READ_BUFFER;
  buf[1] = buffer_status[1];
  buf[2] = 0x00;
  memset(buf + 3, 0xFF, size);

  sx128x_wait_for_ready();

  spi_csn_enable(SX12XX_NSS_PIN);
  spi_dma_transfer_bytes(SX12XX_SPI_PORT, buf, size + 3);
  spi_csn_disable(SX12XX_NSS_PIN);

  memcpy(data, buf + 3, size);
}

void sx128x_write_tx_buffer(const uint8_t offset, const uint8_t *data, const uint8_t size) {
  uint8_t buf[size + 2];
  buf[0] = (uint8_t)SX1280_RADIO_WRITE_BUFFER;
  buf[1] = offset;
  memcpy(buf + 2, data, size);

  sx128x_wait_for_ready();

  spi_csn_enable(SX12XX_NSS_PIN);
  spi_dma_transfer_bytes(SX12XX_SPI_PORT, buf, size + 2);
  spi_csn_disable(SX12XX_NSS_PIN);
}

void sx128x_set_output_power(const int8_t power) {
  const uint8_t buf[2] = {
      power + 18,
      (uint8_t)SX1280_RADIO_RAMP_04_US,
  };
  sx128x_write_command_burst(SX1280_RADIO_SET_TXPARAMS, buf, 2);
}

#endif