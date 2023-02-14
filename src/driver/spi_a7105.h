#pragma once

#include <stdbool.h>
#include <stdint.h>

// Strobe commands to control internal state machine
// See section 10.4 Strobe Commands
typedef enum {
  A7105_SLEEP = 0x80,
  A7105_IDLE = 0x90,
  A7105_STANDBY = 0xA0,
  A7105_PLL = 0xB0,
  A7105_RX = 0xC0,
  A7105_TX = 0xD0,
  A7105_RST_WRPTR = 0xE0,
  A7105_RST_RDPTR = 0xF0
} a7105_strobe_t;

// Definitions for A7105 registers we will directly reference
typedef enum {
  A7105_00_MODE = 0x00,
  A7105_02_CALC = 0x02,
  A7105_05_FIFO_DATA = 0x05,
  A7105_06_ID_DATA = 0x06,
  A7105_0F_CHANNEL = 0x0F,
  A7105_22_IF_CALIB_I = 0x22,
  A7105_24_VCO_CURCAL = 0x24,
  A7105_25_VCO_SBCAL_I = 0x25
} a7105_reg_t;

// Bitfields of A7105_00_MODE register we're interested in
// Section 9.2.1 Mode Register (Address: 00h)
#define A7105_MODE_CRCF 0x20 // [0]: CRC pass. [1]: CRC error. (CRCF is read only, it is updated internally while receiving every packet.)
#define A7105_MODE_TRSR 0x02 // [0]: RX state. [1]: TX state. Serviceable if TRER=1 (TRX is enable).
#define A7105_MODE_TRER 0x01 // [0]: TRX is disabled. [1]: TRX is enabled.

void a7105_init(const uint8_t *regs, uint8_t size);

uint8_t a7105_read_reg(a7105_reg_t reg);
void a7105_write_reg(a7105_reg_t reg, uint8_t data);

void a7105_strobe(a7105_strobe_t address);
void a7105_read_fifo(uint8_t *data, uint8_t num);
void a7105_write_fifo(const uint8_t *data, uint8_t num);

bool a7105_rx_tx_irq_time(uint32_t *irq_time);
