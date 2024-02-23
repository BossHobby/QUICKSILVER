#include "driver/spi_cc2500.h"

#include "core/project.h"
#include "driver/spi.h"
#include "driver/time.h"
#include "util/util.h"

#ifdef USE_RX_SPI_FRSKY

#define SPI_SPEED MHZ_TO_HZ(10.5)

static spi_bus_device_t bus = {
    .auto_continue = true,
};

static bool cc2500_spi_device_valid(const target_rx_spi_device_t *dev) {
  if (dev->port == SPI_PORT_INVALID || dev->nss == PIN_NONE) {
    return false;
  }
  return true;
}

static bool cc2500_hardware_init() {
  if (!cc2500_spi_device_valid(&target.rx_spi)) {
    return false;
  }

  {
    gpio_config_t gpio_init;
    gpio_init.mode = GPIO_OUTPUT;
    gpio_init.drive = GPIO_DRIVE_HIGH;
    gpio_init.output = GPIO_PUSHPULL;
    gpio_init.pull = GPIO_NO_PULL;

    // turn antenna on
    if (target.rx_spi.lna_en != PIN_NONE) {
      gpio_pin_init(target.rx_spi.lna_en, gpio_init);
      gpio_pin_set(target.rx_spi.lna_en);
    }

    // turn tx off
    if (target.rx_spi.tx_en != PIN_NONE) {
      gpio_pin_init(target.rx_spi.tx_en, gpio_init);
      gpio_pin_reset(target.rx_spi.tx_en);
    }

    // choose b?
    if (target.rx_spi.ant_sel != PIN_NONE) {
      gpio_pin_init(target.rx_spi.ant_sel, gpio_init);
      gpio_pin_set(target.rx_spi.ant_sel);
    }
  }

  if (target.rx_spi.exti != PIN_NONE) {
    // GDO0
    gpio_config_t gpio_init;
    gpio_init.mode = GPIO_INPUT;
    gpio_init.drive = GPIO_DRIVE_HIGH;
    gpio_init.output = GPIO_OPENDRAIN;
    gpio_init.pull = GPIO_DOWN_PULL;
    gpio_pin_init(target.rx_spi.exti, gpio_init);
  }

  bus.port = target.rx_spi.port;
  bus.nss = target.rx_spi.nss;
  spi_bus_device_init(&bus);
  spi_bus_device_reconfigure(&bus, SPI_MODE_LEADING_EDGE, SPI_SPEED);

  return true;
}

uint8_t cc2500_read_gdo0() {
  if (target.rx_spi.exti != PIN_NONE) {
    return gpio_pin_read(target.rx_spi.exti);
  }
  return cc2500_get_status() & 0xF;
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
      spi_make_seg_const(reg | CC2500_WRITE_SINGLE, data),
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
        spi_make_seg_const(CC2500_CHANNR | CC2500_WRITE_SINGLE, channel),
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

static bool cc2500_dectect() {
  const uint8_t chipPartNum = cc2500_read_reg(CC2500_PARTNUM | CC2500_READ_BURST); // CC2500 read registers chip part num
  const uint8_t chipVersion = cc2500_read_reg(CC2500_VERSION | CC2500_READ_BURST); // CC2500 read registers chip version
  if (chipPartNum == 0x80 && chipVersion == 0x03) {
    return true;
  }
  return false;
}

bool cc2500_init() {
  if (!cc2500_hardware_init()) {
    return false;
  }
  cc2500_reset();
  return cc2500_dectect();
}

void cc2500_switch_antenna() {
  if (target.rx_spi.ant_sel != PIN_NONE) {
    static uint8_t ant_selected = 1;
    if (ant_selected == 1) {
      gpio_pin_reset(target.rx_spi.ant_sel);
    } else {
      gpio_pin_set(target.rx_spi.ant_sel);
    }
    ant_selected = ant_selected ? 0 : 1;
  }
}

void cc2500_enter_rxmode() {
  if (target.rx_spi.lna_en != PIN_NONE) {
    gpio_pin_set(target.rx_spi.lna_en);
  }
  if (target.rx_spi.tx_en != PIN_NONE) {
    gpio_pin_reset(target.rx_spi.tx_en);
  }
}

void cc2500_enter_txmode() {
  if (target.rx_spi.lna_en != PIN_NONE) {
    gpio_pin_reset(target.rx_spi.lna_en);
  }
  if (target.rx_spi.tx_en != PIN_NONE) {
    gpio_pin_set(target.rx_spi.tx_en);
  }
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