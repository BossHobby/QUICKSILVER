#include "driver/rx/a7105.h"

#include "core/project.h"
#include "driver/exti.h"
#include "driver/spi.h"
#include "driver/time.h"
#include "util/util.h"

#ifdef USE_RX_SPI_FLYSKY

static spi_bus_device_t bus = {};

static volatile uint32_t irq_timestamp = 0;
static volatile bool irq_triggered = false;

// See: drv_exti.c
// This is called when the A7105 GPIO1 pin goes high triggering our external interrupt
void a7105_handle_exti(bool pin_state) {
  if (pin_state) {
    irq_timestamp = time_micros();
    irq_triggered = true;
  }
}

void a7105_write_reg(a7105_reg_t reg, uint8_t data) {
  const spi_txn_segment_t segs[] = {
      spi_make_seg_const(reg, data),
  };
  spi_seg_submit_continue(&bus, segs);
}

static void a7105_write_multi(uint8_t reg, const uint8_t *data, uint8_t len) {
  const spi_txn_segment_t segs[] = {
      spi_make_seg_const(reg),
      spi_make_seg_buffer(NULL, data, len),
  };
  spi_seg_submit_wait(&bus, segs);
}

uint8_t a7105_read_reg(a7105_reg_t reg) {
  uint8_t ret = 0;

  const spi_txn_segment_t segs[] = {
      spi_make_seg_const(reg | 0x40),
      spi_make_seg_buffer(&ret, NULL, 1),
  };
  spi_seg_submit_wait(&bus, segs);

  return ret;
}

static void a7105_read_multi(uint8_t reg, uint8_t *result, uint8_t len) {
  const spi_txn_segment_t segs[] = {
      spi_make_seg_const(reg),
      spi_make_seg_buffer(result, NULL, len),
  };
  spi_seg_submit_wait(&bus, segs);
}

void a7105_strobe(a7105_strobe_t address) {
  // If setting RX or TX mode then we'll want to be interrupted
  // after it has completed
  if ((A7105_RX == address) || (A7105_TX == address)) {
    exti_interrupt_enable(target.rx_spi.exti);
  } else {
    exti_interrupt_disable(target.rx_spi.exti);
  }

  const spi_txn_segment_t segs[] = {
      spi_make_seg_const(address),
  };
  spi_seg_submit_continue(&bus, segs);
}

void a7105_read_fifo(uint8_t *data, uint8_t num) {
  if (data) {
    // pg 65 "16. FIFO (First In First Out)"
    // The fifo is 64 bytes in size, so ensure we don't exceed that limit
    if (num > 64) {
      num = 64;
    }

    // Reset read pointer and read the requested number of bytes
    a7105_strobe(A7105_RST_RDPTR);
    a7105_read_multi((uint8_t)A7105_05_FIFO_DATA | 0x40, data, num);
  }
}

void a7105_write_fifo(const uint8_t *data, uint8_t num) {
  if (data) {
    // pg 65 "16. FIFO (First In First Out)"
    // The fifo is 64 bytes in size, so ensure we don't exceed that limit
    if (num > 64) {
      num = 64;
    }

    // Reset write pointer and write the requested number of bytes
    a7105_strobe(A7105_RST_WRPTR);
    a7105_write_multi((uint8_t)A7105_05_FIFO_DATA, data, num);
  }
}

static bool a7105_spi_device_valid(const target_rx_spi_device_t *dev) {
  if (dev->port == SPI_PORT_INVALID || dev->nss == PIN_NONE) {
    return false;
  }
  if (target.rx_spi.exti == PIN_NONE) {
    return false;
  }
  return true;
}

// Initializes the target hardware for using the A7105 transceiver
// Configures GPIO pin for interrupt trigger
// Also configures EXTI and SPI system for working with the A7105
// PA14 is used as external interrupt with rising edge trigger connected to A7105
static bool a7105_hardware_init() {
  if (!a7105_spi_device_valid(&target.rx_spi)) {
    return false;
  }

  // target.rx_spi.exti is used for triggering interrupt
  gpio_config_t gpio_init;
  gpio_init.mode = GPIO_INPUT;
  gpio_init.drive = GPIO_DRIVE_NORMAL;
  gpio_init.output = GPIO_OPENDRAIN;
  gpio_init.pull = GPIO_NO_PULL;
  gpio_pin_init(target.rx_spi.exti, gpio_init);
  exti_enable(target.rx_spi.exti, EXTI_TRIG_RISING);
  exti_interrupt_disable(target.rx_spi.exti);

  // Configure SPI
  // A7105 data sheet for SPI says max 10Mhz FIFO clock frequency
  // and latches on rising edge of clock
  bus.port = target.rx_spi.port;
  bus.nss = target.rx_spi.nss;
  spi_bus_device_init(&bus);
  spi_bus_device_reconfigure(&bus, SPI_MODE_LEADING_EDGE, MHZ_TO_HZ(10));

  return true;
}

bool a7105_detect() {
  if (!a7105_hardware_init()) {
    return false;
  }

  a7105_write_reg((uint8_t)A7105_00_MODE, 0x00);

  uint32_t data;
  a7105_read_multi((uint8_t)A7105_06_ID_DATA, (uint8_t *)&data, sizeof(data));
  return data == 0;
}

void a7105_init(const uint8_t *regs, uint8_t size) {
  if (!a7105_hardware_init()) {
    return;
  }
  exti_interrupt_disable(target.rx_spi.exti);

  // Write 0 to MODE register to reset the A7105
  a7105_write_reg((uint8_t)A7105_00_MODE, 0x00);

  // 9.2.7 ID DATA Register (Address: 06h)
  // Write the ID: 0x5475c52A
  const uint32_t id = 0x5475c52A;
  uint8_t data[4];
  data[0] = (id >> 24) & 0xFF;
  data[1] = (id >> 16) & 0xFF;
  data[2] = (id >> 8) & 0xFF;
  data[3] = (id >> 0) & 0xFF;
  a7105_write_multi((uint8_t)A7105_06_ID_DATA, &data[0], sizeof(data));

  // Configure the A7105 transceiver by writing the specified register values
  for (unsigned i = 0; i < size; i++) {
    // A value of 0xFF indicates we should skip the register
    if (regs[i] != 0xFF) {
      a7105_write_reg((a7105_reg_t)i, regs[i]);
    }
  }

  a7105_strobe(A7105_STANDBY);

  // pg 15: "9.2.3 Calibration Control Register (Address: 02h)"
  // Set FBC: IF Filter Bank calibration enable (Auto clear when done).
  a7105_write_reg(A7105_02_CALC, 0x01);

  unsigned timeout = 1000;
  while ((a7105_read_reg(A7105_02_CALC) != 0) && timeout--) {
    // spin until calibration has completed (or timeout)
  }

  // 9.2.35 IF Calibration Register I (Address: 22h)
  a7105_read_reg(A7105_22_IF_CALIB_I);

  // 9.2.37 VCO current Calibration Register (Address: 24h
  // MVCS=1 so that VCOC is used. VCOC is set to 011b (3), the recommended value
  a7105_write_reg(A7105_24_VCO_CURCAL, 0x13);
  a7105_write_reg(A7105_25_VCO_SBCAL_I, 0x09);
  a7105_strobe(A7105_STANDBY);
}

// if EXTI occurred then this returns true and updates irq_time with the time
// when it occurred, otherwise returns false
// Note: external interrupt is configured to only occur after setting RX or TX mode
bool a7105_rx_tx_irq_time(uint32_t *irq_time) {
  if (irq_triggered) {
    irq_triggered = false;
    *irq_time = irq_timestamp;
    return true;
  }
  return false;
}

#endif