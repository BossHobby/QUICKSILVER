#pragma once

#include <stdbool.h>
#include <stdint.h>

enum cc2500_w_registers {
  CC2500_SRES = 0x30,    // Reset chip.
  CC2500_SFSTXON = 0x31, // Enable and calibrate frequency synthesizer (if MCSM0.FS_AUTOCAL=1). If in RX (with CCA): Go to a wait state where only the synthesizer is running (for quick RX / TX turnaround).
  CC2500_SXOFF = 0x32,   // Turn off crystal oscillator.
  CC2500_SCAL = 0x33,    // Calibrate frequency synthesizer and turn it off. SCAL can be strobed from IDLE mode without setting manual calibration mode (MCSM0.FS_AUTOCAL=0)
  CC2500_SRX = 0x34,     // Enable RX. Perform calibration first if coming from IDLE and MCSM0.FS_AUTOCAL=1.
  CC2500_STX = 0x35,     // In IDLE state: Enable TX. Perform calibration first if MCSM0.FS_AUTOCAL=1. If in RX state and CCA is enabled: Only go to TX if channel is clear.
  CC2500_SIDLE = 0x36,   // Exit RX / TX, turn off frequency synthesizer and exit Wake-On-Radio mode if applicable.
  CC2500_SWOR = 0x38,    // Start automatic RX polling sequence (Wake-on-Radio) as described in Section 19.5 if WORCTRL.RC_PD=0.
  CC2500_SPWD = 0x39,    // Enter power down mode when CSn goes high.
  CC2500_SFRX = 0x3A,    // Flush the RX FIFO buffer. Only issue SFRX in IDLE or RXFIFO_OVERFLOW states.
  CC2500_SFTX = 0x3B,    // Flush the TX FIFO buffer. Only issue SFTX in IDLE or TXFIFO_UNDERFLOW states.
  CC2500_SWORRST = 0x3C, // Reset real time clock to Event1 value.
  CC2500_SNOP = 0x3D,    // No operation. May be used to get access to the chip status byte.
};

enum cc2500_r_registers {
  CC2500_PARTNUM = 0xF0,        // CC2500 part number
  CC2500_VERSION = 0xF1,        // Current version number
  CC2500_FREQEST = 0xF2,        // Frequency offset estimate
  CC2500_LQI = 0xF3,            // Demodulator estimate for Link Quality
  CC2500_RSSI = 0xF4,           // Received signal strength indication
  CC2500_MARCSTATE = 0xF5,      // Control state machine state
  CC2500_WORTIME1 = 0xF6,       // High byte of WOR timer
  CC2500_WORTIME0 = 0xF7,       // Low byte of WOR timer
  CC2500_PKTSTATUS = 0xF8,      // Current GDOx status and packet status
  CC2500_VCO_VC_DAC = 0xF9,     // Current setting from PLL calibration module
  CC2500_TXBYTES = 0xFA,        // Underflow and number of bytes in the TX FIFO
  CC2500_RXBYTES = 0xFB,        // Overflow and number of bytes in the RX FIFO
  CC2500_RCCTRL1_STATUS = 0xFC, // Last RC oscillator calibration result
  CC2500_RCCTRL0_STATUS = 0xFD, // Last RC oscillator calibration result
};

enum cc2500_rw_registers {
  CC2500_IOCFG2 = 0x00,   // GDO2 output pin configuration
  CC2500_IOCFG1 = 0x01,   // GDO1 output pin configuration
  CC2500_IOCFG0 = 0x02,   // GDO0 output pin configuration
  CC2500_FIFOTHR = 0x03,  // RX FIFO and TX FIFO thresholds
  CC2500_SYNC1 = 0x04,    // Sync word, high byte
  CC2500_SYNC0 = 0x05,    // Sync word, low byte
  CC2500_PKTLEN = 0x06,   // Packet length
  CC2500_PKTCTRL1 = 0x07, // Packet automation control
  CC2500_PKTCTRL0 = 0x08, // Packet automation control
  CC2500_ADDR = 0x09,     // Device address
  CC2500_CHANNR = 0x0A,   // Channel number
  CC2500_FSCTRL1 = 0x0B,  // Frequency synthesizer control
  CC2500_FSCTRL0 = 0x0C,  // Frequency synthesizer control
  CC2500_FREQ2 = 0x0D,    // Frequency control word, high byte
  CC2500_FREQ1 = 0x0E,    // Frequency control word, middle byte
  CC2500_FREQ0 = 0x0F,    // Frequency control word, low byte
  CC2500_MDMCFG4 = 0x10,  // Modem configuration
  CC2500_MDMCFG3 = 0x11,  // Modem configuration
  CC2500_MDMCFG2 = 0x12,  // Modem configuration
  CC2500_MDMCFG1 = 0x13,  // Modem configuration
  CC2500_MDMCFG0 = 0x14,  // Modem configuration
  CC2500_DEVIATN = 0x15,  // Modem deviation setting
  CC2500_MCSM2 = 0x16,    // Main Radio Control State Machine configuration
  CC2500_MCSM1 = 0x17,    // Main Radio Control State Machine configuration
  CC2500_MCSM0 = 0x18,    // Main Radio Control State Machine configuration
  CC2500_FOCCFG = 0x19,   // Frequency Offset Compensation configuration
  CC2500_BSCFG = 0x1A,    // Bit Synchronization configuration
  CC2500_AGCCTRL2 = 0x1B, // AGC control
  CC2500_AGCCTRL1 = 0x1C, // AGC control
  CC2500_AGCCTRL0 = 0x1D, // AGC control
  CC2500_WOREVT1 = 0x1E,  // High byte Event 0 timeout
  CC2500_WOREVT0 = 0x1F,  // Low byte Event 0 timeout
  CC2500_WORCTRL = 0x20,  // Wake On Radio control
  CC2500_FREND1 = 0x21,   // Front end RX configuration
  CC2500_FREND0 = 0x22,   // Front end TX configuration
  CC2500_FSCAL3 = 0x23,   // Frequency synthesizer calibration
  CC2500_FSCAL2 = 0x24,   // Frequency synthesizer calibration
  CC2500_FSCAL1 = 0x25,   // Frequency synthesizer calibration
  CC2500_FSCAL0 = 0x26,   // Frequency synthesizer calibration
  CC2500_RCCTRL1 = 0x27,  // RC oscillator configuration
  CC2500_RCCTRL0 = 0x28,  // RC oscillator configuration
  CC2500_FSTEST = 0x29,   // Frequency synthesizer calibration control
  CC2500_PTEST = 0x2A,    // Production test
  CC2500_AGCTEST = 0x2B,  // AGC test
  CC2500_TEST2 = 0x2C,    // Various test settings
  CC2500_TEST1 = 0x2D,    // Various test settings
  CC2500_TEST0 = 0x2E,    // Various test settings
  CC2500_PATABLE = 0x3E,  // table of 8 values
  CC2500_FIFO = 0x3F,     // FIFO access
};

enum cc2500_modifiers {
  CC2500_WRITE_SINGLE = 0x00,
  CC2500_WRITE_BURST = 0x40,
  CC2500_READ_SINGLE = 0x80,
  CC2500_READ_BURST = 0xC0,
};

bool cc2500_init();
void cc2500_reset();
void cc2500_strobe(uint8_t address);
void cc2500_strobe_sync(uint8_t address);
uint8_t cc2500_get_status();
uint8_t cc2500_read_reg(uint8_t reg);
void cc2500_write_reg(uint8_t reg, uint8_t data);
uint8_t cc2500_read_fifo(uint8_t *result, uint8_t len);
void cc2500_write_fifo(uint8_t *data, uint8_t len);
uint8_t cc2500_read_gdo0();
void cc2500_set_channel(uint8_t channel, uint8_t *cal_data);
uint8_t cc2500_packet_size();

void cc2500_enter_rxmode();
void cc2500_enter_txmode();

void cc2500_switch_antenna();
void cc2500_set_power(uint8_t power);