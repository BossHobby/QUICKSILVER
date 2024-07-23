#include "driver/i2c.h"

#include "driver/interrupt.h"

const i2c_port_def_t i2c_port_defs[I2C_PORT_MAX] = {
    {},
    {
        .channel_index = 1,
        .channel = I2C1,
        .rcc = RCC_APB1_GRP1(I2C1),
        .event_irq = I2C1_EV_IRQn,
        .err_irq = I2C1_ER_IRQn,
    },
    {
        .channel_index = 2,
        .channel = I2C2,
        .rcc = RCC_APB1_GRP1(I2C2),
        .event_irq = I2C2_EV_IRQn,
        .err_irq = I2C2_ER_IRQn,
    },
    {
        .channel_index = 3,
        .channel = I2C3,
        .rcc = RCC_APB1_GRP1(I2C3),
        .event_irq = I2C3_EV_IRQn,
        .err_irq = I2C3_ER_IRQn,
    },
#ifdef I2C4
    {
        .channel_index = 4,
        .channel = I2C4,
#if defined(STM32H7)
        .rcc = RCC_APB4_GRP1(I2C4),
#elif defined(STM32G4)
        .rcc = RCC_APB1_GRP2(I2C4),
#else
        .rcc = RCC_APB1_GRP1(I2C4),
#endif
        .event_irq = I2C4_EV_IRQn,
        .err_irq = I2C4_ER_IRQn,
    },
#endif
};

void i2c_device_init(i2c_ports_t port) {
  const i2c_port_def_t *def = &i2c_port_defs[port];
  rcc_enable(def->rcc);

  LL_I2C_DeInit(def->channel);

  LL_I2C_InitTypeDef i2c_init;
  LL_I2C_StructInit(&i2c_init);
  i2c_init.PeripheralMode = LL_I2C_MODE_I2C;
#if defined(STM32F4)
  i2c_init.ClockSpeed = 400000;
  i2c_init.DutyCycle = LL_I2C_DUTYCYCLE_2;
#else
#if defined(STM32H7)
  const uint32_t pclk_freq = (port == I2C_PORT4) ? HAL_RCCEx_GetD3PCLK1Freq() : HAL_RCC_GetPCLK1Freq();
#else
  const uint32_t pclk_freq = HAL_RCC_GetPCLK1Freq();
#endif
  i2c_init.Timing = i2c_calc_clkctrl(pclk_freq, 400, 0);
  i2c_init.AnalogFilter = LL_I2C_ANALOGFILTER_ENABLE;
  i2c_init.DigitalFilter = 0;
#endif
  i2c_init.OwnAddress1 = 0;
  i2c_init.TypeAcknowledge = LL_I2C_NACK;
  i2c_init.OwnAddrSize = LL_I2C_OWNADDRESS1_7BIT;
  LL_I2C_Init(def->channel, &i2c_init);
  LL_I2C_DisableOwnAddress2(def->channel);
  LL_I2C_DisableGeneralCall(def->channel);
  LL_I2C_EnableClockStretching(def->channel);
#if defined(STM32F4)
  LL_I2C_EnableIT_EVT(def->channel);
  LL_I2C_EnableIT_ERR(def->channel);
#endif
  interrupt_enable(def->event_irq, I2C_PRIORITY);
  interrupt_enable(def->err_irq, I2C_PRIORITY);
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
#if defined(STM32F4)
  LL_I2C_GenerateStartCondition(def->channel);
#else
  SET_BIT(def->channel->CR1, LL_I2C_CR1_TXIE | LL_I2C_CR1_STOPIE | LL_I2C_CR1_TCIE);
  LL_I2C_HandleTransfer(def->channel, bus->address << 1, LL_I2C_ADDRSLAVE_7BIT, 1 + size, LL_I2C_MODE_SOFTEND, LL_I2C_GENERATE_START_WRITE);
#endif
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
#if defined(STM32F4)
  LL_I2C_GenerateStartCondition(def->channel);
#else
  SET_BIT(def->channel->CR1, LL_I2C_CR1_TXIE | LL_I2C_CR1_RXIE | LL_I2C_CR1_STOPIE | LL_I2C_CR1_TCIE);
  LL_I2C_HandleTransfer(def->channel, bus->address << 1, LL_I2C_ADDRSLAVE_7BIT, 1, LL_I2C_MODE_SOFTEND, LL_I2C_GENERATE_START_WRITE);
#endif
}

static void i2c_irq_handler(const i2c_ports_t port) {
  i2c_txn_t *txn = &i2c_dev[port].txn;
  const i2c_port_def_t *def = &i2c_port_defs[port];
#if defined(STM32F4)
  if (LL_I2C_IsActiveFlag_SB(def->channel)) {
    if (txn->size == 1)
      LL_I2C_AcknowledgeNextData(def->channel, LL_I2C_NACK);
    else
      LL_I2C_AcknowledgeNextData(def->channel, LL_I2C_ACK);
    LL_I2C_TransmitData8(def->channel, (txn->address << 1) | (txn->status == TXN_REG ? 0x0 : 0x1));
  } else if (LL_I2C_IsActiveFlag_ADDR(def->channel)) {
    LL_I2C_ClearFlag_ADDR(def->channel);
    if (txn->status == TXN_REG) {
      LL_I2C_TransmitData8(def->channel, txn->reg);
      LL_I2C_EnableIT_BUF(def->channel);
      txn->status = TXN_DATA;
    }
  } else if (LL_I2C_IsActiveFlag_RXNE(def->channel)) {
    txn->data[txn->offset++] = LL_I2C_ReceiveData8(def->channel);
    if (txn->offset == txn->size - 1) {
      LL_I2C_AcknowledgeNextData(def->channel, LL_I2C_NACK);
    }
    if (txn->offset == txn->size) {
      LL_I2C_GenerateStopCondition(def->channel);
      LL_I2C_DisableIT_BUF(def->channel);
      txn->status = TXN_IDLE;
    }
  } else if (LL_I2C_IsActiveFlag_TXE(def->channel)) {
    if (txn->status == TXN_DATA && txn->mode == MODE_READ) {
      LL_I2C_GenerateStartCondition(def->channel);
    } else if (txn->status == TXN_DATA) {
      if (txn->offset == txn->size) {
        LL_I2C_GenerateStopCondition(def->channel);
        txn->status = TXN_IDLE;
      } else {
        LL_I2C_TransmitData8(def->channel, txn->data[txn->offset++]);
      }
    }
  }
#else
  if (LL_I2C_IsActiveFlag_TXIS(def->channel)) {
    if (txn->status == TXN_REG) {
      LL_I2C_TransmitData8(def->channel, txn->reg);
      txn->status = TXN_DATA;
    } else
      LL_I2C_TransmitData8(def->channel, txn->data[txn->offset++]);
  } else if (LL_I2C_IsActiveFlag_RXNE(def->channel)) {
    txn->data[txn->offset++] = LL_I2C_ReceiveData8(def->channel);
    if (txn->offset == txn->size) {
      LL_I2C_GenerateStopCondition(def->channel);
    }
  } else if (LL_I2C_IsActiveFlag_TC(def->channel)) {
    if (txn->status == TXN_DATA && txn->mode == MODE_READ) {
      CLEAR_BIT(def->channel->CR1, LL_I2C_CR1_TXIE | LL_I2C_CR1_TCIE);
      LL_I2C_HandleTransfer(def->channel, txn->address << 1, LL_I2C_ADDRSLAVE_7BIT, txn->size, LL_I2C_MODE_SOFTEND, LL_I2C_GENERATE_START_READ);
    } else {
      LL_I2C_GenerateStopCondition(def->channel);
    }
  } else if (LL_I2C_IsActiveFlag_STOP(def->channel)) {
    LL_I2C_ClearFlag_STOP(def->channel);
    CLEAR_BIT(def->channel->CR1, LL_I2C_CR1_TXIE | LL_I2C_CR1_RXIE | LL_I2C_CR1_STOPIE | LL_I2C_CR1_TCIE);
    txn->status = TXN_IDLE;
  }
#endif
}

static void i2c_err_irq_handler(const i2c_ports_t port) {
  i2c_txn_t *txn = &i2c_dev[port].txn;
  const i2c_port_def_t *def = &i2c_port_defs[port];
#if defined(STM32F4)
  if (LL_I2C_IsActiveFlag_AF(def->channel)) {
    LL_I2C_ClearFlag_AF(def->channel);
    LL_I2C_GenerateStopCondition(def->channel);
    txn->status = TXN_IDLE;
  }
#endif
}

void I2C1_ER_IRQHandler(void) {
  i2c_err_irq_handler(I2C_PORT1);
}
void I2C1_EV_IRQHandler(void) {
  i2c_irq_handler(I2C_PORT1);
}
void I2C2_ER_IRQHandler(void) {
  i2c_err_irq_handler(I2C_PORT2);
}
void I2C2_EV_IRQHandler(void) {
  i2c_irq_handler(I2C_PORT2);
}
void I2C3_ER_IRQHandler(void) {
  i2c_err_irq_handler(I2C_PORT3);
}
void I2C3_EV_IRQHandler(void) {
  i2c_irq_handler(I2C_PORT3);
}
void I2C4_ER_IRQHandler(void) {
  i2c_err_irq_handler(I2C_PORT4);
}
void I2C4_EV_IRQHandler(void) {
  i2c_irq_handler(I2C_PORT4);
}
