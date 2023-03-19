#include "driver/spi_cc2500.h"

#include "core/project.h"
#include "driver/spi.h"
#include "driver/time.h"
#include "util/util.h"

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
  const spi_txn_segment_t segs[] = {
      spi_make_seg_const(address),
  };
  spi_seg_submit_continue(&bus, NULL, segs);
}

void cc2500_strobe_sync(uint8_t address) {
  cc2500_strobe(address);
  spi_txn_wait(&bus);
}

uint8_t cc2500_get_status() {
  uint8_t status = 0;

  const spi_txn_segment_t segs[] = {
      spi_make_seg_buffer(&status, NULL, 1),
  };
  spi_seg_submit_wait(&bus, segs);

  return status;
}

void cc2500_write_reg(uint8_t reg, uint8_t data) {
  const spi_txn_segment_t segs[] = {
      spi_make_seg_const(reg | CC2500_WRITE_SINGLE),
      spi_make_seg_const(data),
  };
  spi_seg_submit_continue(&bus, NULL, segs);
}

uint8_t cc2500_read_reg(uint8_t reg) {
  uint8_t ret = 0;

  const spi_txn_segment_t segs[] = {
      spi_make_seg_const(reg | CC2500_READ_SINGLE),
      spi_make_seg_buffer(&ret, NULL, 1),
  };
  spi_seg_submit_wait(&bus, segs);

  return ret;
}

static uint8_t cc2500_read_multi(uint8_t reg, uint8_t *result, uint8_t len) {
  const spi_txn_segment_t segs[] = {
      spi_make_seg_buffer(&reg, &reg, 1),
      spi_make_seg_buffer(result, NULL, len),
  };
  spi_seg_submit_wait(&bus, segs);

  return reg;
}

static void cc2500_write_multi(uint8_t reg, uint8_t *data, uint8_t len) {
  const spi_txn_segment_t segs[] = {
      spi_make_seg_const(reg),
      spi_make_seg_buffer(NULL, data, len),
  };
  spi_seg_submit_wait(&bus, segs);
}

void cc2500_set_channel(uint8_t channel, uint8_t *cal_data) {
  {
    const spi_txn_segment_t segs[] = {
        spi_make_seg_const(CC2500_SIDLE),
    };
    spi_seg_submit(&bus, NULL, segs);
  }
  {
    const spi_txn_segment_t segs[] = {
        spi_make_seg_const(CC2500_FSCAL3 | CC2500_WRITE_BURST),
        spi_make_seg_buffer(NULL, cal_data, 3),
    };
    spi_seg_submit(&bus, NULL, segs);
  }
  {
    const spi_txn_segment_t segs[] = {
        spi_make_seg_const(CC2500_CHANNR | CC2500_WRITE_SINGLE),
        spi_make_seg_const(channel),
    };
    spi_seg_submit(&bus, NULL, segs);
  }
  spi_txn_continue(&bus);
}

uint8_t cc2500_packet_size() {
  if (cc2500_read_gdo0() == 0) {
    return 0;
  }

  // there is a bug in the cc2500
  // see p3 http:// www.ti.com/lit/er/swrz002e/swrz002e.pdf
  // workaround: read len register very quickly twice:

  // try this 10 times befor giving up:
  for (uint8_t i = 0; i < 10; i++) {
    uint8_t len1 = 0;
    uint8_t len2 = 0;

    {
      const spi_txn_segment_t segs[] = {
          spi_make_seg_const(CC2500_RXBYTES | CC2500_READ_SINGLE),
          spi_make_seg_buffer(&len1, NULL, 1),
          spi_make_seg_const(CC2500_RXBYTES | CC2500_READ_SINGLE),
          spi_make_seg_buffer(&len2, NULL, 1),
      };
      spi_seg_submit_wait(&bus, segs);
    }

    // valid len found?
    if (len1 == len2) {
      return len1;
    }
  }

  return 0;
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