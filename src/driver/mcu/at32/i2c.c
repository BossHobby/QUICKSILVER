#include "driver/i2c.h"

#include "driver/interrupt.h"

const i2c_port_def_t i2c_port_defs[I2C_PORT_MAX] = {
    {},
    {
        .channel_index = 1,
        .channel = I2C1,
        .rcc = RCC_ENCODE(I2C1),
        .event_irq = I2C1_EVT_IRQn,
        .err_irq = I2C1_ERR_IRQn,
    },
    {
        .channel_index = 2,
        .channel = I2C2,
        .rcc = RCC_ENCODE(I2C2),
        .event_irq = I2C2_EVT_IRQn,
        .err_irq = I2C2_ERR_IRQn,
    },
    {
        .channel_index = 3,
        .channel = I2C3,
        .rcc = RCC_ENCODE(I2C3),
        .event_irq = I2C3_EVT_IRQn,
        .err_irq = I2C3_ERR_IRQn,
    },
};

void i2c_device_init(i2c_ports_t port) {
  const i2c_port_def_t *def = &i2c_port_defs[port];
  rcc_enable(def->rcc);

  i2c_reset(def->channel);

  crm_clocks_freq_type crm_clk_freq;
  crm_clocks_freq_get(&crm_clk_freq);

  const uint32_t i2c_clkctrl = i2c_calc_clkctrl(crm_clk_freq.apb1_freq, 400, 0);
  i2c_init(def->channel, 0x0f, i2c_clkctrl);
  i2c_enable(def->channel, TRUE);

  interrupt_enable(def->event_irq, I2C_PRIORITY);
}

void i2c_write_reg_bytes(const i2c_bus_device_t *bus, const uint8_t reg, const uint8_t *data, const uint32_t size) {
  i2c_wait_idle(bus);

  i2c_txn_t *txn = &i2c_dev[bus->port].txn;
  txn->address = bus->address;
  txn->mode = MODE_WRITE;
  txn->status = TXN_REG;
  txn->reg = reg;
  txn->data = (uint8_t *)data;
  txn->size = size;
  txn->offset = 0;

  const i2c_port_def_t *def = &i2c_port_defs[bus->port];
  while (i2c_flag_get(def->channel, I2C_BUSYF_FLAG) != RESET)
    ;
  i2c_transmit_set(def->channel, bus->address << 1, 1 + size, I2C_SOFT_STOP_MODE, I2C_GEN_START_WRITE);
  i2c_interrupt_enable(def->channel, I2C_ERR_INT | I2C_TDC_INT | I2C_STOP_INT | I2C_ACKFIAL_INT | I2C_TD_INT, TRUE);
}

void i2c_read_reg_bytes(const i2c_bus_device_t *bus, const uint8_t reg, uint8_t *data, const uint32_t size) {
  i2c_wait_idle(bus);

  i2c_txn_t *txn = &i2c_dev[bus->port].txn;
  txn->address = bus->address;
  txn->mode = MODE_READ;
  txn->status = TXN_REG;
  txn->reg = reg;
  txn->data = (uint8_t *)data;
  txn->size = size;
  txn->offset = 0;

  const i2c_port_def_t *def = &i2c_port_defs[bus->port];
  while (i2c_flag_get(def->channel, I2C_BUSYF_FLAG) != RESET)
    ;
  i2c_transmit_set(def->channel, bus->address << 1, 1, I2C_SOFT_STOP_MODE, I2C_GEN_START_WRITE);
  i2c_interrupt_enable(def->channel, I2C_ERR_INT | I2C_TDC_INT | I2C_STOP_INT | I2C_ACKFIAL_INT | I2C_TD_INT | I2C_RD_INT, TRUE);
}

static void i2c_irq_handler(const i2c_ports_t port) {
  i2c_txn_t *txn = &i2c_dev[port].txn;
  const i2c_port_def_t *def = &i2c_port_defs[port];

  if (i2c_interrupt_flag_get(def->channel, I2C_TDIS_FLAG)) {
    if (txn->status == TXN_REG) {
      i2c_data_send(def->channel, txn->reg);
      txn->status = TXN_DATA;
    } else
      i2c_data_send(def->channel, txn->data[txn->offset++]);
  } else if (i2c_interrupt_flag_get(def->channel, I2C_RDBF_FLAG)) {
    txn->data[txn->offset++] = i2c_data_receive(def->channel);
    if (txn->offset == txn->size) {
      i2c_stop_generate(def->channel);
    }
  } else if (i2c_interrupt_flag_get(def->channel, I2C_TDC_FLAG)) {
    if (txn->status == TXN_DATA && txn->mode == MODE_READ) {
      i2c_interrupt_enable(def->channel, I2C_TDC_INT | I2C_TD_INT, FALSE);
      i2c_transmit_set(def->channel, txn->address << 1, txn->size, I2C_SOFT_STOP_MODE, I2C_GEN_START_READ);
    } else {
      i2c_stop_generate(def->channel);
    }
  } else if (i2c_interrupt_flag_get(def->channel, I2C_STOPF_FLAG)) {
    i2c_flag_clear(def->channel, I2C_STOPF_FLAG | I2C_TDC_FLAG | I2C_TDIS_FLAG | I2C_RDBF_FLAG);
    i2c_interrupt_enable(def->channel, I2C_ERR_INT | I2C_TDC_INT | I2C_STOP_INT | I2C_ACKFIAL_INT | I2C_TD_INT | I2C_RD_INT, FALSE);
    txn->status = TXN_IDLE;
  }
};

static void i2c_err_irq_handler(const i2c_ports_t port) {
  __NOP();
}

void I2C1_ERR_IRQHandler(void) {
  i2c_err_irq_handler(I2C_PORT1);
}
void I2C1_EVT_IRQHandler(void) {
  i2c_irq_handler(I2C_PORT1);
}
void I2C2_ERR_IRQHandler(void) {
  i2c_err_irq_handler(I2C_PORT2);
}
void I2C2_EVT_IRQHandler(void) {
  i2c_irq_handler(I2C_PORT2);
}
void I2C3_ERR_IRQHandler(void) {
  i2c_err_irq_handler(I2C_PORT3);
}
void I2C3_EVT_IRQHandler(void) {
  i2c_irq_handler(I2C_PORT3);
}
void I2C4_ERR_IRQHandler(void) {
  i2c_err_irq_handler(I2C_PORT4);
}
void I2C4_EVT_IRQHandler(void) {
  i2c_irq_handler(I2C_PORT4);
}
