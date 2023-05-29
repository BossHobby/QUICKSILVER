#include "driver/spi_sx128x.h"

#include <stdbool.h>
#include <string.h>

#include "core/project.h"
#include "driver/exti.h"
#include "driver/spi.h"
#include "driver/time.h"
#include "rx/rx_spi.h"
#include "util/util.h"

#ifdef USE_RX_SPI_EXPRESS_LRS

static bool sx128x_poll_for_not_busy();

FAST_RAM static spi_bus_device_t bus = {
    .auto_continue = true,
    .poll_fn = sx128x_poll_for_not_busy,
};

volatile uint8_t dio0_active = 0;
volatile uint16_t irq_status = 0;

static uint32_t busy_timeout = 1000;
static uint8_t payload_len = 8;

extern volatile uint32_t packet_time;
extern volatile uint8_t packet_status[2];

static bool sx128x_spi_device_valid(const target_rx_spi_device_t *dev) {
  if (dev->port == SPI_PORT_INVALID || dev->nss == PIN_NONE) {
    return false;
  }
  if (target.rx_spi.reset == PIN_NONE || target.rx_spi.busy == PIN_NONE || target.rx_spi.exti == PIN_NONE) {
    return false;
  }
  return true;
}

bool sx128x_init() {
  if (!sx128x_spi_device_valid(&target.rx_spi)) {
    return false;
  }

  gpio_config_t gpio_init;
  gpio_init.mode = GPIO_OUTPUT;
  gpio_init.drive = GPIO_DRIVE_HIGH;
  gpio_init.output = GPIO_PUSHPULL;
  gpio_init.pull = GPIO_NO_PULL;
  gpio_pin_init(target.rx_spi.reset, gpio_init);

  gpio_init.mode = GPIO_INPUT;
  gpio_init.drive = GPIO_DRIVE_HIGH;
  gpio_init.output = GPIO_PUSHPULL;
  gpio_init.pull = GPIO_NO_PULL;
  gpio_pin_init(target.rx_spi.busy, gpio_init);
  if (target.rx_spi.busy_exti) {
    exti_enable(target.rx_spi.busy, EXTI_TRIG_FALLING);
  }

  gpio_pin_init(target.rx_spi.exti, gpio_init);
  exti_enable(target.rx_spi.exti, EXTI_TRIG_RISING);

  bus.port = target.rx_spi.port;
  bus.nss = target.rx_spi.nss;
  spi_bus_device_init(&bus);
  spi_bus_device_reconfigure(&bus, true, MHZ_TO_HZ(10));

  return true;
}

void sx128x_reset() {
  gpio_pin_reset(target.rx_spi.reset);
  time_delay_us(50);
  gpio_pin_set(target.rx_spi.reset);
  time_delay_us(10);
}

void sx128x_set_busy_timeout(uint32_t timeout) {
  busy_timeout = timeout;
}

static bool sx128x_is_busy() {
  return gpio_pin_read(target.rx_spi.busy);
}

static bool sx128x_poll_for_not_busy() {
  if (target.rx_spi.busy_exti) {
    return !sx128x_is_busy();
  }

  const uint32_t start = time_micros();
  while (sx128x_is_busy()) {
    if ((time_micros() - start) > busy_timeout) {
      break;
    }
    __NOP();
  }
  return true;
}

#define read_command_txn(cmd, data, size) \
  spi_make_seg_const(cmd), spi_make_seg_const(0x0), spi_make_seg_buffer(data, NULL, size)

static void sx128x_set_dio0_active() {
  packet_time = time_micros();
  dio0_active = 1;
}

static void sx128x_handle_irq_status() {
  const uint16_t irq = ((irq_status & 0xFF) << 8 | ((irq_status >> 8) & 0xFF));
  if ((irq & SX1280_IRQ_RX_DONE)) {
    static uint8_t buffer_status[2] = {0, 0};
    {
      static const spi_txn_segment_t segs[] = {
          read_command_txn(SX1280_RADIO_GET_RXBUFFERSTATUS, buffer_status, 2),
      };
      spi_seg_submit(&bus, NULL, segs);
    }
    {
      const spi_txn_segment_t segs[] = {
          spi_make_seg_const(SX1280_RADIO_READ_BUFFER),
          spi_make_seg_delay(NULL, buffer_status + 1, 1),
          spi_make_seg_const(0x00),
          spi_make_seg_buffer((uint8_t *)rx_spi_packet, NULL, payload_len),
      };
      spi_seg_submit(&bus, NULL, segs);
    }
    {
      static const spi_txn_segment_t segs[] = {
          read_command_txn(SX1280_RADIO_GET_PACKETSTATUS, (uint8_t *)packet_status, 2),
      };
      spi_seg_submit(&bus, sx128x_set_dio0_active, segs);
    }
  } else if ((irq & SX1280_IRQ_TX_DONE)) {
    sx128x_set_mode_async(SX1280_MODE_RX);
  }
}

void sx128x_wait() {
  uint32_t start = time_micros();
  while (!spi_txn_ready(&bus)) {
    if ((time_micros() - start) > busy_timeout) {
      if (target.rx_spi.busy_exti) {
        bus.poll_fn = NULL;
        spi_txn_continue(&bus);
        bus.poll_fn = sx128x_poll_for_not_busy;

        start = time_micros();
      }
      continue;
    }
    spi_txn_continue(&bus);
    __NOP();
  }
}

void sx128x_handle_dio0_exti(bool level) {
  if (!level) {
    return;
  }
  {
    static const spi_txn_segment_t segs[] = {
        read_command_txn(SX1280_RADIO_GET_IRQSTATUS, (uint8_t *)&irq_status, 2),
    };
    spi_seg_submit(&bus, NULL, segs);
  }
  {
    static const spi_txn_segment_t segs[] = {
        spi_make_seg_const(SX1280_RADIO_CLR_IRQSTATUS),
        spi_make_seg_const((uint8_t)(((uint16_t)SX1280_IRQ_RADIO_ALL >> 8) & 0x00FF)),
        spi_make_seg_const((uint8_t)((uint16_t)SX1280_IRQ_RADIO_ALL & 0x00FF)),
    };
    spi_seg_submit(&bus, sx128x_handle_irq_status, segs);
  }
  spi_txn_continue(&bus);
}

void sx128x_handle_busy_exti(bool level) {
  if (!level) {
    spi_txn_continue(&bus);
  }
}

uint16_t sx128x_read_dio0() {
  register uint8_t active = 0;

  do {
    active = dio0_active;
  } while (active != dio0_active);

  if (!active) {
    return 0;
  }

  dio0_active = 0;
  return ((irq_status & 0xFF) << 8 | ((irq_status >> 8) & 0xFF));
}

void sx128x_read_register_burst(const uint16_t reg, uint8_t *data, const uint8_t size) {
  const uint8_t buf[4] = {
      (SX1280_RADIO_READ_REGISTER),
      ((reg & 0xFF00) >> 8),
      (reg & 0x00FF),
      0x00,
  };

  const spi_txn_segment_t segs[] = {
      spi_make_seg_buffer(NULL, buf, 4),
      spi_make_seg_buffer(data, NULL, size),
  };
  spi_seg_submit(&bus, NULL, segs);
}

uint8_t sx128x_read_register(const uint16_t reg) {
  uint8_t data = 0;
  sx128x_read_register_burst(reg, &data, 1);
  sx128x_wait();
  return data;
}

void sx128x_write_register_burst(const uint16_t reg, const uint8_t *data, const uint8_t size) {
  uint8_t buf[3] = {
      (uint8_t)SX1280_RADIO_WRITE_REGISTER,
      ((reg & 0xFF00) >> 8),
      (reg & 0x00FF),
  };

  const spi_txn_segment_t segs[] = {
      spi_make_seg_buffer(NULL, buf, 3),
      spi_make_seg_buffer(NULL, data, size),
  };
  spi_seg_submit(&bus, NULL, segs);
}

void sx128x_write_register(const uint16_t reg, const uint8_t val) {
  sx128x_write_register_burst(reg, &val, 1);
}

void sx128x_read_command_burst(const sx128x_commands_t cmd, uint8_t *data, const uint8_t size) {
  const spi_txn_segment_t segs[] = {
      read_command_txn(cmd, data, size),
  };
  spi_seg_submit(&bus, NULL, segs);
}

void sx128x_write_command_burst(const sx128x_commands_t cmd, const uint8_t *data, const uint8_t size) {
  const spi_txn_segment_t segs[] = {
      spi_make_seg_const(cmd),
      spi_make_seg_buffer(NULL, data, size),
  };
  spi_seg_submit(&bus, NULL, segs);
}

void sx128x_write_command(const sx128x_commands_t cmd, const uint8_t val) {
  sx128x_write_command_burst(cmd, &val, 1);
}

void sx128x_write_tx_buffer(const uint8_t offset, const volatile uint8_t *data, const uint8_t size) {
  const uint8_t buf[2] = {
      (uint8_t)SX1280_RADIO_WRITE_BUFFER,
      offset,
  };
  const spi_txn_segment_t segs[] = {
      spi_make_seg_buffer(NULL, buf, 2),
      spi_make_seg_buffer(NULL, (uint8_t *)data, size),
  };
  spi_seg_submit(&bus, NULL, segs);
}

void sx128x_set_mode_async(const sx128x_modes_t mode) {
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
    // uses timeout Time-out duration = periodBase * periodBaseCount
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

  spi_txn_continue(&bus);
}

void sx128x_set_mode(const sx128x_modes_t mode) {
  sx128x_set_mode_async(mode);
  sx128x_wait();
}

void sx128x_set_lora_mod_params(const sx128x_lora_bandwidths_t bw, const sx128x_lora_spreading_factors_t sf, const sx128x_lora_coding_rates_t cr) {
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
  sx128x_wait();
}

void sx128x_set_flrc_mod_params(const uint8_t bw, const uint8_t cr, const uint8_t bt) {
  const uint8_t rfparams[3] = {bw, cr, bt};
  sx128x_write_command_burst(SX1280_RADIO_SET_MODULATIONPARAMS, rfparams, 3);
  sx128x_wait();
}

void sx128x_set_flrc_packet_params(const uint8_t header_type, const uint8_t preamble_length, const uint8_t payload_length, uint32_t sync_word, uint16_t crc_seed, uint8_t cr) {
  payload_len = payload_length;

  uint8_t buf[7] = {
      preamble_length,
      SX1280_FLRC_SYNC_WORD_LEN_P32S,
      SX1280_FLRC_RX_MATCH_SYNC_WORD_1,
      header_type,
      payload_length,
      SX1280_FLRC_CRC_3_BYTE,
      0x08,
  };
  sx128x_write_command_burst(SX1280_RADIO_SET_PACKETPARAMS, buf, 7);
  sx128x_wait();

  // CRC seed (use dedicated cipher)
  buf[0] = (uint8_t)(crc_seed >> 8);
  buf[1] = (uint8_t)crc_seed;
  sx128x_write_register_burst(SX1280_REG_FLRC_CRC_SEED, buf, 2);
  sx128x_wait();

  // Set sync_word1
  buf[0] = (uint8_t)(sync_word >> 24);
  buf[1] = (uint8_t)(sync_word >> 16);
  buf[2] = (uint8_t)(sync_word >> 8);
  buf[3] = (uint8_t)sync_word;

  // DS_SX1280-1_V3.2.pdf - 16.4 FLRC Modem: Increased PER in FLRC Packets with Synch Word
  if (((cr == SX1280_FLRC_CR_1_2) || (cr == SX1280_FLRC_CR_3_4)) &&
      ((buf[0] == 0x8C && buf[1] == 0x38) || (buf[0] == 0x63 && buf[1] == 0x0E))) {
    uint8_t temp = buf[0];
    buf[0] = buf[1];
    buf[1] = temp;
    // For SX1280_FLRC_CR_3_4 the datasheet also says
    // "In addition to this the two LSB values XX XX must not be in the range 0x0000 to 0x3EFF"
    if (cr == SX1280_FLRC_CR_3_4 && buf[3] <= 0x3e)
      buf[3] |= 0x80; // 0x80 or 0x40 would work
  }
  sx128x_write_register_burst(SX1280_REG_FLRC_SYNC_WORD, buf, 4);
  sx128x_wait();

  // Set Synch Address Control to zero bit errors permissible
  uint8_t sync_addr_ctrl = sx128x_read_register(SX1280_REG_FLRC_SYNC_ADDR_CTRL);
  sync_addr_ctrl &= SX1280_REG_FLRC_SYNC_ADDR_CTRL_ZERO_MASK;
  sx128x_write_register(SX1280_REG_FLRC_SYNC_ADDR_CTRL, sync_addr_ctrl);
}

void sx128x_set_lora_packet_params(const uint8_t preamble_length, const sx128x_lora_packet_lengths_modes_t header_type, const uint8_t payload_length, const sx128x_lora_crc_modes_t crc, const sx128x_lora_iq_modes_t invert_iq) {
  payload_len = payload_length;

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
  sx128x_wait();
}

void sx128x_set_frequency(const uint32_t freq) {
  const uint8_t buf[3] = {
      (uint8_t)((freq >> 16) & 0xFF),
      (uint8_t)((freq >> 8) & 0xFF),
      (uint8_t)(freq & 0xFF),
  };
  sx128x_write_command_burst(SX1280_RADIO_SET_RFFREQUENCY, buf, 3);
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
  sx128x_wait();
}

void sx128x_clear_irq_status(const uint16_t irq_mask) {
  const uint8_t buf[2] = {
      (uint8_t)(((uint16_t)irq_mask >> 8) & 0x00FF),
      (uint8_t)((uint16_t)irq_mask & 0x00FF),
  };
  sx128x_write_command_burst(SX1280_RADIO_CLR_IRQSTATUS, buf, 2);
  sx128x_wait();
}

void sx128x_set_output_power(const int8_t power) {
  const uint8_t buf[2] = {
      power + 18,
      (uint8_t)SX1280_RADIO_RAMP_04_US,
  };
  sx128x_write_command_burst(SX1280_RADIO_SET_TXPARAMS, buf, 2);
  sx128x_wait();
}

#endif