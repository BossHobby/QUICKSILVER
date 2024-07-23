#pragma once

#include <stdbool.h>
#include <stdint.h>

#include "core/project.h"
#include "driver/dma.h"
#include "driver/gpio.h"
#include "driver/rcc.h"

typedef struct {
  uint8_t channel_index;
  i2c_port_t *channel;
  rcc_reg_t rcc;
  IRQn_Type event_irq;
  IRQn_Type err_irq;
} i2c_port_def_t;

typedef struct {
  i2c_ports_t port;
  uint8_t address;
} i2c_bus_device_t;

typedef enum {
  MODE_INVALID,
  MODE_READ,
  MODE_WRITE,
} i2c_txn_mode_t;

typedef enum {
  TXN_IDLE,
  TXN_REG,
  TXN_DATA,
} i2c_txn_status_t;

typedef struct {
  i2c_txn_mode_t mode;
  uint8_t address;
  volatile i2c_txn_status_t status;
  uint8_t reg;
  uint8_t *data;
  uint8_t size;
  uint8_t offset;
} i2c_txn_t;

typedef struct {
  bool is_init;
  i2c_txn_t txn;
} i2c_device_t;

extern i2c_device_t i2c_dev[I2C_PORT_MAX];
extern const i2c_port_def_t i2c_port_defs[I2C_PORT_MAX];

bool i2c_bus_device_init(const i2c_bus_device_t *bus);

static inline bool i2c_is_idle(const i2c_bus_device_t *bus) { return i2c_dev[bus->port].txn.status == TXN_IDLE; }
void i2c_wait_idle(const i2c_bus_device_t *bus);

void i2c_write_reg(const i2c_bus_device_t *bus, const uint8_t reg, const uint8_t data);
void i2c_write_reg_bytes(const i2c_bus_device_t *bus, const uint8_t reg, const uint8_t *data, const uint32_t size);

uint8_t i2c_read_reg(const i2c_bus_device_t *bus, const uint8_t reg);
void i2c_read_reg_bytes(const i2c_bus_device_t *bus, const uint8_t reg, uint8_t *data, const uint32_t size);

#define i2c_read_async(bus, reg, data, size)    \
  ({                                            \
    static bool is_done = false;                \
    const bool temp = is_done;                  \
    if (!is_done) {                             \
      i2c_read_reg_bytes(bus, reg, data, size); \
      is_done = true;                           \
    } else                                      \
      is_done = false;                          \
    temp;                                       \
  })

uint32_t i2c_calc_clkctrl(uint32_t pclk_freq, uint32_t i2c_freq_khz, uint32_t dfcoeff);
