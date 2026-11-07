#include "driver/i2c.h"

#include <hardware/i2c.h>

#include "driver/interrupt.h"

#ifdef USE_I2C

const i2c_port_def_t i2c_port_defs[I2C_PORT_MAX] = {
    {},
    {
        .channel_index = 0,
        .channel = i2c0_hw,
        .rcc = RESETS_RESET_I2C0_BITS,
        .event_irq = I2C0_IRQ_IRQn,
    },
    {
        .channel_index = 1,
        .channel = i2c1_hw,
        .rcc = RESETS_RESET_I2C1_BITS,
        .event_irq = I2C1_IRQ_IRQn,
    },
};

// Bytes read so far; txn->offset counts commands queued to the TX FIFO.
static uint8_t i2c_rx_offset[I2C_PORT_MAX];

static constexpr uint32_t I2C_FIFO_DEPTH = 16;
static constexpr uint32_t I2C_TXN_INTR = I2C_IC_INTR_MASK_M_TX_EMPTY_BITS | I2C_IC_INTR_MASK_M_RX_FULL_BITS |
                                         I2C_IC_INTR_MASK_M_STOP_DET_BITS | I2C_IC_INTR_MASK_M_TX_ABRT_BITS;

void i2c_device_init(i2c_ports_t port) {
  const i2c_port_def_t *def = &i2c_port_defs[port];

  // i2c_init() also cycles the block through reset.
  i2c_init(I2C_INSTANCE(def->channel_index), 400000);

  i2c_hw_t *hw = def->channel;
  hw->intr_mask = 0;
  hw->tx_tl = 0;
  hw->rx_tl = 0;
  interrupt_enable(def->event_irq, I2C_PRIORITY);
}

// The controller sends START on the first command and a RESTART where the
// direction turns, then STOP after the command flagged with it.
static void i2c_txn_start(const i2c_bus_device_t *bus, i2c_txn_mode_t mode, const uint8_t reg, uint8_t *data, const uint32_t size) {
  i2c_wait_idle(bus);

  i2c_txn_t *txn = &i2c_dev[bus->port].txn;
  txn->address = bus->address;
  txn->mode = mode;
  txn->status = TXN_REG;
  txn->reg = reg;
  txn->data = data;
  txn->size = size;
  txn->offset = 0;
  i2c_rx_offset[bus->port] = 0;

  i2c_hw_t *hw = i2c_port_defs[bus->port].channel;
  hw->enable = 0;
  hw->tar = bus->address;
  hw->enable = 1;
  hw->intr_mask = I2C_TXN_INTR;
}

void i2c_write_reg_bytes(const i2c_bus_device_t *bus, const uint8_t reg, const uint8_t *data, const uint32_t size) {
  i2c_txn_start(bus, MODE_WRITE, reg, (uint8_t *)data, size);
}

void i2c_read_reg_bytes(const i2c_bus_device_t *bus, const uint8_t reg, uint8_t *data, const uint32_t size) {
  i2c_txn_start(bus, MODE_READ, reg, data, size);
}

static void i2c_fill_tx(i2c_txn_t *txn, i2c_hw_t *hw) {
  while (hw->txflr < I2C_FIFO_DEPTH) {
    if (txn->status == TXN_REG) {
      hw->data_cmd = txn->reg | (txn->size == 0 ? I2C_IC_DATA_CMD_STOP_BITS : 0);
      txn->status = TXN_DATA;
      continue;
    }
    if (txn->offset == txn->size) {
      hw_clear_bits(&hw->intr_mask, I2C_IC_INTR_MASK_M_TX_EMPTY_BITS);
      return;
    }

    const bool first = txn->offset == 0;
    const bool last = txn->offset + 1 == txn->size;
    uint32_t cmd = last ? I2C_IC_DATA_CMD_STOP_BITS : 0;
    if (txn->mode == MODE_READ)
      cmd |= I2C_IC_DATA_CMD_CMD_BITS | (first ? I2C_IC_DATA_CMD_RESTART_BITS : 0);
    else
      cmd |= txn->data[txn->offset];
    hw->data_cmd = cmd;
    txn->offset++;
  }
}

static void i2c_irq_handler(const i2c_ports_t port) {
  i2c_txn_t *txn = &i2c_dev[port].txn;
  i2c_hw_t *hw = i2c_port_defs[port].channel;
  const uint32_t status = hw->intr_stat;

  if (status & I2C_IC_INTR_STAT_R_RX_FULL_BITS) {
    while (hw->rxflr && i2c_rx_offset[port] < txn->size)
      txn->data[i2c_rx_offset[port]++] = hw->data_cmd;
  }

  // A NACK or lost arbitration flushes the FIFO; the controller still ends
  // with STOP, which completes the transaction below.
  if (status & I2C_IC_INTR_STAT_R_TX_ABRT_BITS) {
    (void)hw->clr_tx_abrt;
    hw_clear_bits(&hw->intr_mask, I2C_IC_INTR_MASK_M_TX_EMPTY_BITS);
  } else if (status & I2C_IC_INTR_STAT_R_TX_EMPTY_BITS) {
    i2c_fill_tx(txn, hw);
  }

  if (status & I2C_IC_INTR_STAT_R_STOP_DET_BITS) {
    (void)hw->clr_stop_det;
    hw->intr_mask = 0;
    txn->status = TXN_IDLE;
    i2c_notify_from_isr();
  }
}

extern "C" {
void I2C0_IRQ_Handler() { i2c_irq_handler(I2C_PORT1); }
void I2C1_IRQ_Handler() { i2c_irq_handler(I2C_PORT2); }
}

#endif
