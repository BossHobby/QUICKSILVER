#pragma once

#include <stdint.h>

#define SX1280_REG_FREQ_ERR_CORRECTION 0x93C
#define SX1280_REG_FLRC_CRC_SEED 0x9C8
#define SX1280_REG_FLRC_SYNC_WORD 0x9CF
#define SX1280_REG_FLRC_SYNC_ADDR_CTRL 0x9CD
#define SX1280_REG_FLRC_SYNC_ADDR_CTRL_ZERO_MASK 0b11110000

enum sx128x_registers {
  SX128x_LR_FIRMWARE_VERSION_MSB = 0x0153,
};

typedef enum {
  SX1280_RADIO_GET_STATUS = 0xC0,
  SX1280_RADIO_WRITE_REGISTER = 0x18,
  SX1280_RADIO_READ_REGISTER = 0x19,
  SX1280_RADIO_WRITE_BUFFER = 0x1A,
  SX1280_RADIO_READ_BUFFER = 0x1B,
  SX1280_RADIO_SET_SLEEP = 0x84,
  SX1280_RADIO_SET_STANDBY = 0x80,
  SX1280_RADIO_SET_FS = 0xC1,
  SX1280_RADIO_SET_TX = 0x83,
  SX1280_RADIO_SET_RX = 0x82,
  SX1280_RADIO_SET_RXDUTYCYCLE = 0x94,
  SX1280_RADIO_SET_CAD = 0xC5,
  SX1280_RADIO_SET_TXCONTINUOUSWAVE = 0xD1,
  SX1280_RADIO_SET_TXCONTINUOUSPREAMBLE = 0xD2,
  SX1280_RADIO_SET_PACKETTYPE = 0x8A,
  SX1280_RADIO_GET_PACKETTYPE = 0x03,
  SX1280_RADIO_SET_RFFREQUENCY = 0x86,
  SX1280_RADIO_SET_TXPARAMS = 0x8E,
  SX1280_RADIO_SET_CADPARAMS = 0x88,
  SX1280_RADIO_SET_BUFFERBASEADDRESS = 0x8F,
  SX1280_RADIO_SET_MODULATIONPARAMS = 0x8B,
  SX1280_RADIO_SET_PACKETPARAMS = 0x8C,
  SX1280_RADIO_GET_RXBUFFERSTATUS = 0x17,
  SX1280_RADIO_GET_PACKETSTATUS = 0x1D,
  SX1280_RADIO_GET_RSSIINST = 0x1F,
  SX1280_RADIO_SET_DIOIRQPARAMS = 0x8D,
  SX1280_RADIO_GET_IRQSTATUS = 0x15,
  SX1280_RADIO_CLR_IRQSTATUS = 0x97,
  SX1280_RADIO_CALIBRATE = 0x89,
  SX1280_RADIO_SET_REGULATORMODE = 0x96,
  SX1280_RADIO_SET_SAVECONTEXT = 0xD5,
  SX1280_RADIO_SET_AUTOTX = 0x98,
  SX1280_RADIO_SET_AUTOFS = 0x9E,
  SX1280_RADIO_SET_LONGPREAMBLE = 0x9B,
  SX1280_RADIO_SET_UARTSPEED = 0x9D,
  SX1280_RADIO_SET_RANGING_ROLE = 0xA3,
} sx128x_commands_t;

typedef enum {
  SX1280_MODE_SLEEP = 0x00, // The radio is in sleep mode
  SX1280_MODE_CALIBRATION,  // The radio is in calibration mode
  SX1280_MODE_STDBY_RC,     // The radio is in standby mode with RC oscillator
  SX1280_MODE_STDBY_XOSC,   // The radio is in standby mode with XOSC oscillator
  SX1280_MODE_FS,           // The radio is in frequency synthesis mode
  SX1280_MODE_RX,           // The radio is in receive mode
  SX1280_MODE_TX,           // The radio is in transmit mode
  SX1280_MODE_CAD           // The radio is in channel activity detection mode
} sx128x_modes_t;

enum sx128x_irq_masks {
  SX1280_IRQ_RADIO_NONE = 0x0000,
  SX1280_IRQ_TX_DONE = 0x0001,
  SX1280_IRQ_RX_DONE = 0x0002,
  SX1280_IRQ_SYNCWORD_VALID = 0x0004,
  SX1280_IRQ_SYNCWORD_ERROR = 0x0008,
  SX1280_IRQ_HEADER_VALID = 0x0010,
  SX1280_IRQ_HEADER_ERROR = 0x0020,
  SX1280_IRQ_CRC_ERROR = 0x0040,
  SX1280_IRQ_RANGING_SLAVE_RESPONSE_DONE = 0x0080,
  SX1280_IRQ_RANGING_SLAVE_REQUEST_DISCARDED = 0x0100,
  SX1280_IRQ_RANGING_MASTER_RESULT_VALID = 0x0200,
  SX1280_IRQ_RANGING_MASTER_TIMEOUT = 0x0400,
  SX1280_IRQ_RANGING_SLAVE_REQUEST_VALID = 0x0800,
  SX1280_IRQ_CAD_DONE = 0x1000,
  SX1280_IRQ_CAD_DETECTED = 0x2000,
  SX1280_IRQ_RX_TX_TIMEOUT = 0x4000,
  SX1280_IRQ_PREAMBLE_DETECTED = 0x8000,
  SX1280_IRQ_RADIO_ALL = 0xFFFF,
};

enum sx128x_standby_modes {
  SX1280_STDBY_RC = 0x00,
  SX1280_STDBY_XOSC = 0x01,
};

enum sx128x_packet_type {
  SX1280_PACKET_TYPE_GFSK = 0x00,
  SX1280_PACKET_TYPE_LORA,
  SX1280_PACKET_TYPE_RANGING,
  SX1280_PACKET_TYPE_FLRC,
  SX1280_PACKET_TYPE_BLE,
  SX1280_PACKET_TYPE_NONE = 0x0F,
};

enum sx128x_ramp_times {
  SX1280_RADIO_RAMP_02_US = 0x00,
  SX1280_RADIO_RAMP_04_US = 0x20,
  SX1280_RADIO_RAMP_06_US = 0x40,
  SX1280_RADIO_RAMP_08_US = 0x60,
  SX1280_RADIO_RAMP_10_US = 0x80,
  SX1280_RADIO_RAMP_12_US = 0xA0,
  SX1280_RADIO_RAMP_16_US = 0xC0,
  SX1280_RADIO_RAMP_20_US = 0xE0,
};

typedef enum {
  SX1280_LORA_BW_0200 = 0x34,
  SX1280_LORA_BW_0400 = 0x26,
  SX1280_LORA_BW_0800 = 0x18,
  SX1280_LORA_BW_1600 = 0x0A,
} sx128x_lora_bandwidths_t;

typedef enum {
  SX1280_LORA_SF5 = 0x50,
  SX1280_LORA_SF6 = 0x60,
  SX1280_LORA_SF7 = 0x70,
  SX1280_LORA_SF8 = 0x80,
  SX1280_LORA_SF9 = 0x90,
  SX1280_LORA_SF10 = 0xA0,
  SX1280_LORA_SF11 = 0xB0,
  SX1280_LORA_SF12 = 0xC0,
} sx128x_lora_spreading_factors_t;

typedef enum {
  SX1280_LORA_CR_4_5 = 0x01,
  SX1280_LORA_CR_4_6 = 0x02,
  SX1280_LORA_CR_4_7 = 0x03,
  SX1280_LORA_CR_4_8 = 0x04,
  SX1280_LORA_CR_LI_4_5 = 0x05,
  SX1280_LORA_CR_LI_4_6 = 0x06,
  SX1280_LORA_CR_LI_4_7 = 0x07,
} sx128x_lora_coding_rates_t;

typedef enum {
  SX1280_LORA_PACKET_VARIABLE_LENGTH = 0x00, // The packet is on variable size, header included
  SX1280_LORA_PACKET_FIXED_LENGTH = 0x80,    // The packet is known on both sides, no header included in the packet
  SX1280_LORA_PACKET_EXPLICIT = SX1280_LORA_PACKET_VARIABLE_LENGTH,
  SX1280_LORA_PACKET_IMPLICIT = SX1280_LORA_PACKET_FIXED_LENGTH,
} sx128x_lora_packet_lengths_modes_t;

typedef enum {
  SX1280_LORA_CRC_ON = 0x20,
  SX1280_LORA_CRC_OFF = 0x00,
} sx128x_lora_crc_modes_t;

typedef enum {
  SX1280_LORA_IQ_NORMAL = 0x40,
  SX1280_LORA_IQ_INVERTED = 0x00,
} sx128x_lora_iq_modes_t;

typedef enum {
  SX1280_FLRC_BR_1_300_BW_1_2 = 0x45,
  SX1280_FLRC_BR_1_000_BW_1_2 = 0x69,
  SX1280_FLRC_BR_0_650_BW_0_6 = 0x86,
  SX1280_FLRC_BR_0_520_BW_0_6 = 0xAA,
  SX1280_FLRC_BR_0_325_BW_0_3 = 0xC7,
  SX1280_FLRC_BR_0_260_BW_0_3 = 0xEB,
} sx128x_radio_flrc_bandwidths_t;

typedef enum {
  SX1280_FLRC_CR_1_2 = 0x00,
  SX1280_FLRC_CR_3_4 = 0x02,
  SX1280_FLRC_CR_1_0 = 0x04,
} sx128x_radio_flrc_coding_rates_t;

typedef enum {
  SX1280_FLRC_BT_DIS = 0x00,
  SX1280_FLRC_BT_1 = 0x10,
  SX1280_FLRC_BT_0_5 = 0x20,
} sx128x_radio_flrc_gaussian_filter_t;

typedef enum {
  SX1280_FLRC_SYNC_NOSYNC = 0x00,
  SX1280_FLRC_SYNC_WORD_LEN_P32S = 0x04,
} sx128x_radio_flrc_sync_word_len_t;

typedef enum {
  SX1280_FLRC_RX_DISABLE_SYNC_WORD = 0x00,
  SX1280_FLRC_RX_MATCH_SYNC_WORD_1 = 0x10,
  SX1280_FLRC_RX_MATCH_SYNC_WORD_2 = 0x20,
  SX1280_FLRC_RX_MATCH_SYNC_WORD_1_2 = 0x30,
  SX1280_FLRC_RX_MATCH_SYNC_WORD_3 = 0x40,
  SX1280_FLRC_RX_MATCH_SYNC_WORD_1_3 = 0x50,
  SX1280_FLRC_RX_MATCH_SYNC_WORD_2_3 = 0x60,
  SX1280_FLRC_RX_MATCH_SYNC_WORD_1_2_3 = 0x70,
} sx128x_radio_flrc_sync_word_combination_t;

typedef enum {
  SX1280_FLRC_PACKET_FIXED_LENGTH = 0x00,
  SX1280_FLRC_PACKET_VARIABLE_LENGTH = 0x20,
} sx128x_radio_flrc_packet_type_t;

typedef enum {
  SX1280_FLRC_CRC_OFF = 0x00,
  SX1280_FLRC_CRC_2_BYTE = 0x10,
  SX1280_FLRC_CRC_3_BYTE = 0x20,
  SX1280_FLRC_CRC_4_BYTE = 0x30,
} SX1280_RadioFlrcCrc_t;

typedef enum {
  // Error Packet Status
  SX1280_FLRC_PKT_ERROR_BUSY = 1 << 0,
  SX1280_FLRC_PKT_ERROR_PKT_RCVD = 1 << 1,
  SX1280_FLRC_PKT_ERROR_HDR_RCVD = 1 << 2,
  SX1280_FLRC_PKT_ERROR_ABORT = 1 << 3,
  SX1280_FLRC_PKT_ERROR_CRC = 1 << 4,
  SX1280_FLRC_PKT_ERROR_LENGTH = 1 << 5,
  SX1280_FLRC_PKT_ERROR_SYNC = 1 << 6,
} sx128x_radio_flrc_crc_t;

void sx128x_init();
void sx128x_reset();
void sx128x_wait();

void sx128x_set_busy_timeout(uint32_t timeout);

uint16_t sx128x_read_dio0();

void sx128x_set_mode(const sx128x_modes_t mode);
void sx128x_set_mode_async(const sx128x_modes_t mode);

void sx128x_read_register_burst(const uint16_t reg, uint8_t *data, const uint8_t size);
uint8_t sx128x_read_register(const uint16_t reg);
void sx128x_write_register_burst(const uint16_t reg, const uint8_t *data, const uint8_t size);
void sx128x_write_register(const uint16_t reg, const uint8_t val);

void sx128x_write_command(const sx128x_commands_t cmd, const uint8_t val);
void sx128x_write_command_burst(const sx128x_commands_t cmd, const uint8_t *data, const uint8_t size);

void sx128x_read_command_burst(const sx128x_commands_t cmd, uint8_t *data, const uint8_t size);

void sx128x_write_tx_buffer(const uint8_t offset, const volatile uint8_t *data, const uint8_t size);

void sx128x_set_lora_mod_params(const sx128x_lora_bandwidths_t bw, const sx128x_lora_spreading_factors_t sf, const sx128x_lora_coding_rates_t cr);
void sx128x_set_flrc_mod_params(const uint8_t bw, const uint8_t cr, const uint8_t bt);

void sx128x_set_lora_packet_params(const uint8_t preamble_length, const sx128x_lora_packet_lengths_modes_t header_type, const uint8_t payload_length, const sx128x_lora_crc_modes_t crc, const sx128x_lora_iq_modes_t invert_iq);
void sx128x_set_flrc_packet_params(const uint8_t header_type, const uint8_t preamble_length, const uint8_t payload_length, uint32_t sync_word, uint16_t crc_seed, uint8_t cr);

void sx128x_set_frequency(const uint32_t freq);
void sx128x_set_dio_irq_params(const uint16_t irq_mask, const uint16_t dio1_mask, const uint16_t dio2_mask, const uint16_t dio3_mask);
void sx128x_set_output_power(const int8_t power);

void sx128x_clear_irq_status(const uint16_t irq_mask);