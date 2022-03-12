#include "rx_express_lrs.h"

#include <stdbool.h>

#include "util/util.h"

#if defined(RX_EXPRESS_LRS)

#define SYNC_WORD 0x12 // default LoRa sync word

#define FIFO_TX_BASE_ADDR_MAX 0b00000000
#define FIFO_RX_BASE_ADDR_MAX 0b00000000

static uint32_t current_rate = 0;

#ifdef USE_SX127X

static expresslrs_mod_settings_t air_rate_config[ELRS_RATE_MAX] = {
    {0, RATE_200HZ, SX127x_BW_500_00_KHZ, SX127x_SF_6, SX127x_CR_4_7, 5000, TLM_RATIO_1_64, 4, 8, 8},
    {1, RATE_100HZ, SX127x_BW_500_00_KHZ, SX127x_SF_7, SX127x_CR_4_7, 10000, TLM_RATIO_1_64, 4, 8, 8},
    {2, RATE_50HZ, SX127x_BW_500_00_KHZ, SX127x_SF_8, SX127x_CR_4_7, 20000, TLM_RATIO_NO_TLM, 4, 10, 8},
    {3, RATE_25HZ, SX127x_BW_500_00_KHZ, SX127x_SF_9, SX127x_CR_4_7, 40000, TLM_RATIO_NO_TLM, 2, 10, 8},
};

static expresslrs_rf_pref_params_t rf_pref_params[ELRS_RATE_MAX] = {
    {0, RATE_200HZ, -112, 4380, 3000, 2500, 600, 5000},
    {1, RATE_100HZ, -117, 8770, 3500, 2500, 600, 5000},
    {2, RATE_50HZ, -120, 18560, 4000, 2500, 600, 5000},
    {3, RATE_25HZ, -123, 29950, 6000, 4000, 0, 5000},
};

#endif

#ifdef USE_SX128X
static expresslrs_mod_settings_t air_rate_config[ELRS_RATE_MAX] = {
    {0, RATE_500HZ, SX1280_LORA_BW_0800, SX1280_LORA_SF5, SX1280_LORA_CR_LI_4_6, 2000, TLM_RATIO_1_128, 4, 12, 8},
    {1, RATE_250HZ, SX1280_LORA_BW_0800, SX1280_LORA_SF6, SX1280_LORA_CR_LI_4_7, 4000, TLM_RATIO_1_64, 4, 14, 8},
    {2, RATE_150HZ, SX1280_LORA_BW_0800, SX1280_LORA_SF7, SX1280_LORA_CR_LI_4_7, 6666, TLM_RATIO_1_32, 4, 12, 8},
    {3, RATE_50HZ, SX1280_LORA_BW_0800, SX1280_LORA_SF9, SX1280_LORA_CR_LI_4_6, 20000, TLM_RATIO_NO_TLM, 2, 12, 8},
};

static expresslrs_rf_pref_params_t rf_pref_params[ELRS_RATE_MAX] = {
    {0, RATE_500HZ, -105, 1665, 2500, 2500, 3, 5000},
    {1, RATE_250HZ, -108, 3300, 3000, 2500, 6, 5000},
    {2, RATE_150HZ, -112, 5871, 3500, 2500, 10, 5000},
    {3, RATE_50HZ, -117, 18443, 4000, 2500, 0, 5000},
};
#endif

const expresslrs_mod_settings_t *current_air_rate_config() {
  return &air_rate_config[current_rate];
}

const expresslrs_rf_pref_params_t *current_rf_pref_params() {
  return &rf_pref_params[current_rate];
}

extern int32_t fhss_update_freq_correction(uint8_t value);

#ifdef USE_SX127X

bool elrs_radio_init() {
  sx127x_init();
  return sx127x_detect();
}

void elrs_set_frequency(int32_t freq) {
  sx127x_set_mode(SX127x_OPMODE_STANDBY);

  uint8_t buf[3] = {
      (uint8_t)((freq >> 16) & 0xFF),
      (uint8_t)((freq >> 8) & 0xFF),
      (uint8_t)(freq & 0xFF),
  };
  sx127x_write_reg_burst(SX127x_FRF_MSB, buf, 3);
}

void elrs_set_rate(uint8_t index, int32_t freq, bool invert_iq) {
  sx127x_write_reg(SX127x_OP_MODE, SX127x_OPMODE_SLEEP);
  sx127x_write_reg(SX127x_OP_MODE, SX127x_OPMODE_LORA);
  sx127x_set_mode(SX127x_OPMODE_STANDBY);

  sx127x_write_reg(SX127x_IRQ_FLAGS, 0b11111111);

  sx127x_write_reg(SX127x_PAYLOAD_LENGTH, ELRS_BUFFER_SIZE);
  sx127x_write_reg(SX127x_SYNC_WORD, SYNC_WORD);

  sx127x_write_reg(SX127x_FIFO_RX_BASE_ADDR, FIFO_RX_BASE_ADDR_MAX);
  sx127x_write_reg(SX127x_FIFO_TX_BASE_ADDR, FIFO_TX_BASE_ADDR_MAX);

  sx127x_set_reg(SX127x_DIO_MAPPING_1, 0b11000000, 7, 6);

  sx127x_write_reg(SX127x_LNA, SX127x_LNA_BOOST_ON);
  sx127x_write_reg(SX127x_MODEM_CONFIG_3, SX1278_AGC_AUTO_ON | SX1278_LOW_DATA_RATE_OPT_OFF);
  sx127x_set_reg(SX127x_OCP, SX127X_OCP_ON | SX127X_OCP_150MA, 5, 0);

  sx127x_write_reg(SX127x_PREAMBLE_LSB, air_rate_config[index].preamble_len);

  sx127x_set_reg(SX127x_INVERT_IQ, (uint8_t)(invert_iq), 6, 6);

  sx127x_set_mode(SX127x_OPMODE_STANDBY);
  sx127x_write_reg(SX127x_PA_CONFIG, SX127x_PA_SELECT_BOOST | SX127x_MAX_OUTPUT_POWER | 0b1111);

  sx127x_set_reg(SX127x_MODEM_CONFIG_2, air_rate_config[index].sf | SX127X_TX_MODE_SINGLE | SX1278_RX_CRC_MODE_OFF, 7, 2);
  if (air_rate_config[index].sf == SX127x_SF_6) {
    sx127x_set_reg(SX127x_DETECT_OPTIMIZE, SX127x_DETECT_OPTIMIZE_SF_6, 2, 0);
    sx127x_write_reg(SX127x_DETECTION_THRESHOLD, SX127x_DETECTION_THRESHOLD_SF_6);

    sx127x_write_reg(SX127x_MODEM_CONFIG_1, air_rate_config[index].bw | air_rate_config[index].cr | SX1278_HEADER_IMPL_MODE);
  } else {
    sx127x_set_reg(SX127x_DETECT_OPTIMIZE, SX127x_DETECT_OPTIMIZE_SF_7_12, 2, 0);
    sx127x_write_reg(SX127x_DETECTION_THRESHOLD, SX127x_DETECTION_THRESHOLD_SF_7_12);

    sx127x_write_reg(SX127x_MODEM_CONFIG_1, air_rate_config[index].bw | air_rate_config[index].cr | SX1278_HEADER_IMPL_MODE);
  }

  if (air_rate_config[index].bw == SX127x_BW_500_00_KHZ) {
    // datasheet errata reconmendation http://caxapa.ru/thumbs/972894/SX1276_77_8_ErrataNote_1.1_STD.pdf
    sx127x_write_reg(0x36, 0x02);
    sx127x_write_reg(0x3a, 0x64);
  } else {
    sx127x_write_reg(0x36, 0x03);
  }

  elrs_set_frequency(freq);

  current_rate = index;
}

void elrs_enter_rx(uint8_t *packet) {
  sx127x_set_mode(SX127x_OPMODE_STANDBY);
  sx127x_write_reg(SX127x_FIFO_ADDR_PTR, 0x0);
  sx127x_set_mode(SX127x_OPMODE_RXCONTINUOUS);
}

void elrs_enter_tx(uint8_t *packet) {
  sx127x_set_mode(SX127x_OPMODE_STANDBY);

  sx127x_write_reg(SX127x_FIFO_ADDR_PTR, 0x0);
  sx127x_write_fifo(packet, ELRS_BUFFER_SIZE);

  sx127x_set_mode(SX127x_OPMODE_TX);
}

bool elrs_read_packet(uint8_t *packet) {
  if (!sx127x_read_dio0()) {
    return false;
  }

  sx127x_read_fifo(packet, ELRS_BUFFER_SIZE);
  sx127x_write_reg(SX127x_IRQ_FLAGS, 0b11111111);

  return true;
}

void elrs_freq_correct() {
  const int32_t offset = fhss_update_freq_correction(((sx127x_read_reg(SX127x_FEI_MSB) & 0b1000) >> 3) ? 1 : 0);
  sx127x_write_reg(SX127x_PPMOFFSET, (uint8_t)offset);
}

#endif

#ifdef USE_SX128X

volatile uint8_t packet_status[2] = {0, 0};

bool elrs_radio_init() {
  sx128x_init();

  sx128x_reset();

  for (size_t i = 0; i < 10; i++) {
    uint8_t buf[2];
    sx128x_read_register_burst(SX128x_LR_FIRMWARE_VERSION_MSB, buf, 2);
    sx128x_wait();

    uint16_t version = (((buf[0]) << 8) | (buf[1]));
    if ((version != 0) && (version != 65535)) {
      return true;
    }
  }
  return false;
}

void elrs_set_frequency(int32_t freq) {
  sx128x_set_frequency(freq);
}

void elrs_set_rate(uint8_t index, int32_t freq, bool invert_iq) {
  sx128x_set_busy_timeout(1000);
  sx128x_set_mode(SX1280_MODE_SLEEP);

  sx128x_set_mode(SX1280_MODE_STDBY_RC);
  sx128x_write_command(SX1280_RADIO_SET_PACKETTYPE, SX1280_PACKET_TYPE_LORA);
  sx128x_wait();

  sx128x_config_lora_mod_params(SX1280_LORA_BW_0800, SX1280_LORA_SF6, SX1280_LORA_CR_4_7);
  sx128x_write_command(SX1280_RADIO_SET_AUTOFS, 0x01);
  sx128x_wait();

  sx128x_write_register(0x0891, (sx128x_read_register(0x0891) | 0xC0));
  sx128x_set_packet_params(12, SX1280_LORA_PACKET_IMPLICIT, air_rate_config[index].payload_len, SX1280_LORA_CRC_OFF, SX1280_LORA_IQ_NORMAL);
  sx128x_set_frequency(12098953);
  sx128x_wait();
  sx128x_set_fifo_addr(0x00, 0x00);
  sx128x_set_dio_irq_params(SX1280_IRQ_RADIO_ALL, SX1280_IRQ_TX_DONE | SX1280_IRQ_RX_DONE, SX1280_IRQ_RADIO_NONE, SX1280_IRQ_RADIO_NONE);
  sx128x_set_output_power(13);

  sx128x_set_mode(SX1280_MODE_STDBY_XOSC);
  sx128x_clear_irq_status(SX1280_IRQ_RADIO_ALL);
  sx128x_config_lora_mod_params(air_rate_config[index].bw, air_rate_config[index].sf, air_rate_config[index].cr);
  sx128x_set_packet_params(air_rate_config[index].preamble_len, SX1280_LORA_PACKET_IMPLICIT, air_rate_config[index].payload_len, SX1280_LORA_CRC_OFF, (sx128x_lora_iq_modes_t)((uint8_t)!invert_iq << 6));
  sx128x_set_frequency(freq);

  sx128x_set_busy_timeout(100);

  current_rate = index;
  reset_looptime();
}

void elrs_enter_rx(volatile uint8_t *packet) {
  sx128x_set_mode_async(SX1280_MODE_RX);
}

void elrs_enter_tx(volatile uint8_t *packet) {
  sx128x_write_tx_buffer(0x0, packet, ELRS_BUFFER_SIZE);
  sx128x_set_mode_async(SX1280_MODE_TX);
}

elrs_irq_status_t elrs_get_irq_status() {
  const uint16_t irq = sx128x_read_dio0();
  if ((irq & SX1280_IRQ_TX_DONE)) {
    return IRQ_TX_DONE;
  } else if ((irq & SX1280_IRQ_RX_DONE)) {
    return IRQ_RX_DONE;
  }
  return IRQ_NONE;
}

void elrs_read_packet(volatile uint8_t *packet) {
  // async
}

void elrs_last_packet_stats(int8_t *rssi, int8_t *snr) {
  *rssi = -(int8_t)(packet_status[0] / 2);
  *snr = (int8_t)packet_status[1] / 4;
}

void elrs_freq_correct() {
  // do nothing for 2400ghz
}

#endif

#endif