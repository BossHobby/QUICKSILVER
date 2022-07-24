#include "drv_spi_cc2500.h"

#include "drv_spi.h"
#include "drv_time.h"
#include "project.h"

#if defined(USE_CC2500)

#define SPI_SPEED MHZ_TO_HZ(10.5)

static spi_bus_device_t bus = {
    .port = CC2500_SPI_PORT,
    .nss = CC2500_NSS_PIN,

    .auto_continue = true,
};

uint8_t cc2500_read_gdo0() {
#ifdef CC2500_GDO0_PIN
  return gpio_pin_read(CC2500_GDO0_PIN);
#else
  return cc2500_get_status() & 0xF;
#endif
}

static void cc2500_hardware_init() {
#if defined(USE_CC2500_PA_LNA)
  {
    LL_GPIO_InitTypeDef gpio_init;
    gpio_init.Mode = LL_GPIO_MODE_OUTPUT;
    gpio_init.Speed = LL_GPIO_SPEED_FREQ_HIGH;
    gpio_init.OutputType = LL_GPIO_OUTPUT_PUSHPULL;
    gpio_init.Pull = LL_GPIO_PULL_NO;

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
  }
#endif // USE_CC2500_PA_LNA

#ifdef CC2500_GDO0_PIN
  {
    // GDO0
    LL_GPIO_InitTypeDef gpio_init;
    gpio_init.Mode = LL_GPIO_MODE_INPUT;
    gpio_init.Speed = LL_GPIO_SPEED_FREQ_HIGH;
    gpio_init.OutputType = LL_GPIO_OUTPUT_OPENDRAIN;
    gpio_init.Pull = LL_GPIO_PULL_DOWN;
    gpio_pin_init(&gpio_init, CC2500_GDO0_PIN);
  }
#endif

  spi_bus_device_init(&bus);
  spi_bus_device_reconfigure(&bus, SPI_MODE_LEADING_EDGE, SPI_SPEED);
}

void cc2500_strobe(uint8_t address) {
  spi_txn_t *txn = spi_txn_init(&bus, NULL);
  spi_txn_add_seg_const(txn, address);
  spi_txn_submit_continue(&bus, txn);
}

void cc2500_strobe_sync(uint8_t address) {
  cc2500_strobe(address);
  spi_txn_wait(&bus);
}

uint8_t cc2500_get_status() {
  uint8_t status = 0;

  spi_txn_t *txn = spi_txn_init(&bus, NULL);
  spi_txn_add_seg(txn, &status, NULL, 1);
  spi_txn_submit_wait(&bus, txn);

  return status;
}

void cc2500_write_reg(uint8_t reg, uint8_t data) {
  spi_txn_t *txn = spi_txn_init(&bus, NULL);
  spi_txn_add_seg_const(txn, reg | CC2500_WRITE_SINGLE);
  spi_txn_add_seg_const(txn, data);
  spi_txn_submit_continue(&bus, txn);
}

uint8_t cc2500_read_reg(uint8_t reg) {
  uint8_t ret = 0;

  spi_txn_t *txn = spi_txn_init(&bus, NULL);
  spi_txn_add_seg_const(txn, reg | CC2500_READ_SINGLE);
  spi_txn_add_seg(txn, &ret, NULL, 1);
  spi_txn_submit_wait(&bus, txn);

  return ret;
}

static uint8_t cc2500_read_multi(uint8_t reg, uint8_t *result, uint8_t len) {
  spi_txn_t *txn = spi_txn_init(&bus, NULL);
  spi_txn_add_seg(txn, &reg, &reg, 1);
  spi_txn_add_seg(txn, result, NULL, len);
  spi_txn_submit_wait(&bus, txn);

  return reg;
}

static void cc2500_write_multi(uint8_t reg, uint8_t *data, uint8_t len) {
  spi_txn_t *txn = spi_txn_init(&bus, NULL);
  spi_txn_add_seg_const(txn, reg);
  spi_txn_add_seg(txn, NULL, data, len);
  spi_txn_submit(txn);

  // needs to sync ???
  spi_txn_continue_ex(&bus, true);
}

uint8_t cc2500_read_fifo(uint8_t *result, uint8_t len) {
  return cc2500_read_multi(CC2500_FIFO | CC2500_READ_BURST, result, len);
}

void cc2500_write_fifo(uint8_t *data, uint8_t len) {
  cc2500_strobe(CC2500_SFTX);
  cc2500_write_multi(CC2500_FIFO | CC2500_WRITE_BURST, data, len);
  cc2500_strobe(CC2500_STX);
}

void cc2500_reset() {
  cc2500_strobe_sync(CC2500_SRES);
  time_delay_us(1000);
  cc2500_strobe_sync(CC2500_SIDLE);
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