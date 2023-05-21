#include "rx/express_lrs.h"

#include <stdbool.h>

#include "core/looptime.h"
#include "core/project.h"
#include "util/util.h"

#if defined(RX_EXPRESS_LRS)

#define SYNC_WORD 0x12 // default LoRa sync word

#define FIFO_TX_BASE_ADDR_MAX 0b00000000
#define FIFO_RX_BASE_ADDR_MAX 0b00000000

static uint32_t current_rate = 0;

static const expresslrs_mod_settings_t air_rate_config[ELRS_RATE_MAX] = {
    {0, RADIO_TYPE_SX128x_FLRC, RATE_FLRC_1000HZ, SX1280_FLRC_BR_0_650_BW_0_6, SX1280_FLRC_BT_1, SX1280_FLRC_CR_1_2, TLM_RATIO_1_128, 2, 1000, 32, OTA4_PACKET_SIZE, 1},
    {1, RADIO_TYPE_SX128x_FLRC, RATE_FLRC_500HZ, SX1280_FLRC_BR_0_650_BW_0_6, SX1280_FLRC_BT_1, SX1280_FLRC_CR_1_2, TLM_RATIO_1_128, 2, 2000, 32, OTA4_PACKET_SIZE, 1},
    {2, RADIO_TYPE_SX128x_FLRC, RATE_DVDA_500HZ, SX1280_FLRC_BR_0_650_BW_0_6, SX1280_FLRC_BT_1, SX1280_FLRC_CR_1_2, TLM_RATIO_1_128, 2, 1000, 32, OTA4_PACKET_SIZE, 2},
    {3, RADIO_TYPE_SX128x_FLRC, RATE_DVDA_250HZ, SX1280_FLRC_BR_0_650_BW_0_6, SX1280_FLRC_BT_1, SX1280_FLRC_CR_1_2, TLM_RATIO_1_128, 2, 1000, 32, OTA4_PACKET_SIZE, 4},
    {4, RADIO_TYPE_SX128x_LORA, RATE_LORA_500HZ, SX1280_LORA_BW_0800, SX1280_LORA_SF5, SX1280_LORA_CR_LI_4_6, TLM_RATIO_1_128, 4, 2000, 12, OTA4_PACKET_SIZE, 1},
    {5, RADIO_TYPE_SX128x_LORA, RATE_LORA_333HZ_8CH, SX1280_LORA_BW_0800, SX1280_LORA_SF5, SX1280_LORA_CR_LI_4_7, TLM_RATIO_1_128, 4, 3003, 12, OTA8_PACKET_SIZE, 1},
    {6, RADIO_TYPE_SX128x_LORA, RATE_LORA_250HZ, SX1280_LORA_BW_0800, SX1280_LORA_SF6, SX1280_LORA_CR_LI_4_7, TLM_RATIO_1_64, 4, 4000, 14, OTA4_PACKET_SIZE, 1},
    {7, RADIO_TYPE_SX128x_LORA, RATE_LORA_150HZ, SX1280_LORA_BW_0800, SX1280_LORA_SF7, SX1280_LORA_CR_LI_4_7, TLM_RATIO_1_32, 4, 6666, 12, OTA4_PACKET_SIZE, 1},
    {8, RADIO_TYPE_SX128x_LORA, RATE_LORA_100HZ_8CH, SX1280_LORA_BW_0800, SX1280_LORA_SF7, SX1280_LORA_CR_LI_4_7, TLM_RATIO_1_32, 4, 10000, 12, OTA8_PACKET_SIZE, 1},
    {9, RADIO_TYPE_SX128x_LORA, RATE_LORA_50HZ, SX1280_LORA_BW_0800, SX1280_LORA_SF8, SX1280_LORA_CR_LI_4_7, TLM_RATIO_1_16, 2, 20000, 12, OTA4_PACKET_SIZE, 1},
};

static const expresslrs_rf_pref_params_t rf_pref_params[ELRS_RATE_MAX] = {
    {0, RATE_FLRC_1000HZ, -104, 389, 2500, 2500, 3, 5000, DYNPOWER_SNR_THRESH_NONE, DYNPOWER_SNR_THRESH_NONE},
    {1, RATE_FLRC_500HZ, -104, 389, 2500, 2500, 3, 5000, DYNPOWER_SNR_THRESH_NONE, DYNPOWER_SNR_THRESH_NONE},
    {2, RATE_DVDA_500HZ, -104, 389, 2500, 2500, 3, 5000, DYNPOWER_SNR_THRESH_NONE, DYNPOWER_SNR_THRESH_NONE},
    {3, RATE_DVDA_250HZ, -104, 389, 2500, 2500, 3, 5000, DYNPOWER_SNR_THRESH_NONE, DYNPOWER_SNR_THRESH_NONE},
    {4, RATE_LORA_500HZ, -105, 1507, 2500, 2500, 3, 5000, SNR_SCALE(5), SNR_SCALE(9.5)},
    {5, RATE_LORA_333HZ_8CH, -105, 2374, 2500, 2500, 4, 5000, SNR_SCALE(5), SNR_SCALE(9.5)},
    {6, RATE_LORA_250HZ, -108, 3300, 3000, 2500, 6, 5000, SNR_SCALE(3), SNR_SCALE(9.5)},
    {7, RATE_LORA_150HZ, -112, 5871, 3500, 2500, 10, 5000, SNR_SCALE(0), SNR_SCALE(8.5)},
    {8, RATE_LORA_100HZ_8CH, -112, 7605, 3500, 2500, 11, 5000, SNR_SCALE(0), SNR_SCALE(8.5)},
    {9, RATE_LORA_50HZ, -115, 10798, 4000, 2500, 0, 5000, SNR_SCALE(-1), SNR_SCALE(6.5)},
};

const expresslrs_mod_settings_t *current_air_rate_config() {
  return &air_rate_config[current_rate];
}

const expresslrs_rf_pref_params_t *current_rf_pref_params() {
  return &rf_pref_params[current_rate];
}

extern int32_t fhss_update_freq_correction(bool value);

volatile uint8_t packet_status[2] = {0, 0};

static bool elrs_radio_detect() {
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

bool elrs_radio_init() {
  sx128x_wait();
  if (!sx128x_init()) {
    return false;
  }
  sx128x_reset();

  if (!elrs_radio_detect()) {
    return false;
  }

  sx128x_set_mode(SX1280_MODE_STDBY_RC);
  sx128x_write_register(0x0891, (sx128x_read_register(0x0891) | 0xC0));
  sx128x_write_command(SX1280_RADIO_SET_AUTOFS, 0x01);
  sx128x_wait();

  return true;
}

void elrs_set_frequency(int32_t freq) {
  sx128x_set_frequency(freq);
}

void elrs_set_rate(uint8_t index, int32_t freq, bool invert_iq, uint32_t flrc_sync_word, uint16_t flrc_crc_seed) {
  sx128x_set_busy_timeout(1000);
  sx128x_wait();

  sx128x_set_mode(SX1280_MODE_STDBY_RC);

  if (air_rate_config[index].radio_type == RADIO_TYPE_SX128x_FLRC) {
    sx128x_write_command(SX1280_RADIO_SET_PACKETTYPE, SX1280_PACKET_TYPE_FLRC);
    sx128x_set_flrc_mod_params(air_rate_config[index].bw, air_rate_config[index].cr, air_rate_config[index].sf);
    sx128x_set_flrc_packet_params(SX1280_FLRC_PACKET_FIXED_LENGTH, air_rate_config[index].preamble_len, air_rate_config[index].payload_len, flrc_sync_word, flrc_crc_seed, air_rate_config[index].cr);
    sx128x_wait();
  } else {
    sx128x_write_command(SX1280_RADIO_SET_PACKETTYPE, SX1280_PACKET_TYPE_LORA);
    sx128x_set_lora_mod_params(air_rate_config[index].bw, air_rate_config[index].sf, air_rate_config[index].cr);
    sx128x_set_lora_packet_params(air_rate_config[index].preamble_len, SX1280_LORA_PACKET_IMPLICIT, air_rate_config[index].payload_len, SX1280_LORA_CRC_OFF, (sx128x_lora_iq_modes_t)((uint8_t)!invert_iq << 6));
    sx128x_wait();
  }

  sx128x_set_dio_irq_params(
      SX1280_IRQ_TX_DONE | SX1280_IRQ_RX_DONE | SX1280_IRQ_SYNCWORD_VALID | SX1280_IRQ_SYNCWORD_ERROR | SX1280_IRQ_CRC_ERROR,
      SX1280_IRQ_TX_DONE | SX1280_IRQ_RX_DONE,
      SX1280_IRQ_RADIO_NONE,
      SX1280_IRQ_RADIO_NONE);
  sx128x_wait();

  sx128x_set_output_power(13);
  sx128x_set_frequency(freq);
  sx128x_wait();

  sx128x_clear_irq_status(SX1280_IRQ_RADIO_ALL);
  sx128x_wait();

  sx128x_set_busy_timeout(100);

  current_rate = index;
  looptime_reset();
}

void elrs_enter_rx(volatile uint8_t *packet) {
  sx128x_set_mode_async(SX1280_MODE_RX);
}

void elrs_enter_tx(volatile uint8_t *packet, const uint8_t packet_len) {
  sx128x_write_tx_buffer(0x0, packet, packet_len);
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
  if (current_air_rate_config()->radio_type == RADIO_TYPE_SX128x_FLRC) {
    // No SNR in FLRC mode
    *rssi = -(int8_t)(packet_status[0] / 2);
    *snr = 0;
    return;
  }

  // LoRa mode has both RSSI and SNR
  int8_t new_rssi = -(int8_t)(packet_status[0] / 2);
  int8_t new_snr = (int8_t)packet_status[1];

  // https://www.mouser.com/datasheet/2/761/DS_SX1280-1_V2.2-1511144.pdf p84
  // need to subtract SNR from RSSI when SNR <= 0;
  int8_t neg_offset = (new_snr < 0) ? (new_snr / RADIO_SNR_SCALE) : 0;
  new_rssi += neg_offset;

  *rssi = new_rssi;
  *snr = new_snr;
}

void elrs_freq_correct() {
  // TODO: fhss_update_freq_correction(SX1280_REG_LR_ESTIMATED_FREQUENCY_ERROR_MSB)
}

#endif