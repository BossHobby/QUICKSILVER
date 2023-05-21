#include "rx/express_lrs.h"

#include <stdbool.h>
#include <stdlib.h>
#include <string.h>

#include "core/debug.h"
#include "core/flash.h"
#include "core/project.h"
#include "driver/gpio.h"
#include "driver/time.h"
#include "flight/control.h"
#include "io/msp.h"
#include "io/usb_configurator.h"
#include "rx/crsf.h"
#include "rx/rx_spi.h"
#include "util/util.h"

#if defined(RX_EXPRESS_LRS)

#define ELRS_CRC14_POLY 0x2E57 // 0x372B
#define ELRS_CRC16_POLY 0x3D65 // 0x9eb2

#define MSP_SET_RX_CONFIG 45
#define MSP_BUFFER_SIZE 128

// ELRS specific opcodes
#define MSP_ELRS_RF_MODE 0x06
#define MSP_ELRS_TX_PWR 0x07
#define MSP_ELRS_TLM_RATE 0x08
#define MSP_ELRS_BIND 0x09
#define MSP_ELRS_MODEL_ID 0x0A
#define MSP_ELRS_REQU_VTX_PKT 0x0B
#define MSP_ELRS_SET_TX_BACKPACK_WIFI_MODE 0x0C
#define MSP_ELRS_SET_VRX_BACKPACK_WIFI_MODE 0x0D
#define MSP_ELRS_SET_RX_WIFI_MODE 0x0E

#define TELEMETRY_TYPE_LINK 0x01
#define TELEMETRY_TYPE_DATA 0x02

#define RC_DATA_PACKET 0b00
#define MSP_DATA_PACKET 0b01
#define SYNC_PACKET 0b10
#define TLM_PACKET 0b11

#define MODELMATCH_MASK 0x3f

// Desired buffer time between Packet ISR and Tock ISR
#define PACKET_TO_TOCK_SLACK 250
#define CONSIDER_CONN_GOOD_MILLIS 1000
#define RF_MODE_CYCLE_MULTIPLIER_SLOW 10

#define LOCKUP_TIMEOUT_US 10000
#define SYNC_TIMEOUT_MS 30000

// Maximum ms between LINK_STATISTICS packets for determining burst max
#define TELEM_MIN_LINK_INTERVAL 512U

#define UID bind_storage.elrs.uid

extern volatile elrs_phase_lock_state_t pl_state;

extern void fhss_update_freq_correction(bool value);
extern void fhss_randomize(int32_t seed);
extern uint8_t fhss_get_index();
extern void fhss_set_index(const uint8_t value);
extern uint32_t fhss_get_sync_freq();
extern uint32_t fhss_next_freq();
extern void fhss_reset();
extern uint8_t fhss_min_lq_for_chaos();
extern uint32_t fhss_rf_mode_cycle_interval();

extern void elrs_crc_init(uint8_t bits, uint16_t poly);
extern uint16_t elrs_crc_calc(const volatile uint8_t *data, uint8_t len, uint16_t crc);

extern expresslrs_mod_settings_t *current_air_rate_config();
extern expresslrs_rf_pref_params_t *current_rf_pref_params();

extern uint8_t tlm_ratio_enum_to_value(expresslrs_tlm_ratio_t val);
extern uint16_t rate_enum_to_hz(expresslrs_rf_rates_t val);

volatile uint32_t packet_time = 0;
elrs_timer_state_t elrs_timer_state = TIMER_DISCONNECTED;
volatile uint8_t ota_nonce = 0;

static volatile elrs_state_t elrs_state = DISCONNECTED;

static volatile bool already_hop = false;
static volatile bool already_tlm = false;

static bool radio_is_init = false;
static bool is_full_res = false;
static bool in_binding_mode = false;
static bool has_model_match = false;

static uint32_t connected_millis = 0;
static uint32_t sync_packet_millis = 0;
static uint32_t last_valid_packet_millis = 0;
static uint32_t last_rf_mode_cycle_millis = 0;

static uint32_t next_rate = ELRS_RATE_DEFAULT;
static uint32_t next_switch_mode_pending = 0;
static uint32_t rf_mode_cycle_multiplier = 1;

static int8_t raw_rssi = 0;
static int8_t raw_snr = 0;
static uint8_t uplink_lq = 0;

static elrs_lpf_t rssi_lpf;
static int8_t rssi;

static uint16_t crc_initializer = 0;
static uint8_t bind_uid[6] = {0, 1, 2, 3, 4, 5};

static uint8_t msp_buffer[ELRS_MSP_BUFFER_SIZE];
static uint8_t next_tlm_type = TELEMETRY_TYPE_LINK;
static uint8_t tlm_burst_count = 1;
static uint8_t tlm_burst_max = 1;
static bool tlm_burst_valid = false;
static uint8_t tlm_buffer[CRSF_FRAME_SIZE_MAX];
static bool tlm_device_info_pending = false;
static uint8_t tlm_denom = 1;

static uint8_t msp_tx_buffer[MSP_BUFFER_SIZE];
static uint32_t msp_tx_len = 0;
static uint8_t msp_last_cmd = 0;
static uint8_t msp_origin = 0;
static bool msp_new_data = false;
static bool msp_is_error = false;

static uint32_t elrs_get_uid_mac_seed() {
  return ((uint32_t)UID[2] << 24) + ((uint32_t)UID[3] << 16) +
         ((uint32_t)UID[4] << 8) + (UID[5] ^ ELRS_OTA_VERSION_ID);
}

static void elrs_set_switch_mode(const elrs_switch_mode_t mode, uint8_t packet_size) {
  is_full_res = packet_size == OTA8_PACKET_SIZE;

  if (is_full_res) {
    elrs_crc_init(16, ELRS_CRC16_POLY);
  } else {
    elrs_crc_init(14, ELRS_CRC14_POLY);
  }

  bind_storage.elrs.switch_mode = mode;
}

static uint8_t elrs_get_model_id() {
  // invert value so default (0x0) equals no model match
  return bind_storage.elrs.model_id ^ 0xFF;
}

static bool elrs_hop() {
  if (already_hop || in_binding_mode || elrs_state == DISCONNECTED) {
    return false;
  }

  const uint8_t modresult = (ota_nonce + 1) % current_air_rate_config()->fhss_hop_interval;
  if (modresult != 0) {
    return false;
  }

  elrs_set_frequency(fhss_next_freq());
  already_hop = true;

  const uint8_t tlm_mod = (ota_nonce + 1) % tlm_denom;
  if (tlm_mod != 0 || tlm_denom == 1) {
    elrs_enter_rx(rx_spi_packet);
  }
  return true;
}

static bool elrs_tlm() {
  if (already_tlm || elrs_state == DISCONNECTED) {
    return false;
  }

  const uint8_t tlm_mod = (ota_nonce + 1) % tlm_denom;
  if (tlm_mod != 0 || tlm_denom == 1) {
    return false;
  }

  rx_lqi_got_packet();

  memset((uint8_t *)rx_spi_packet, 0, current_air_rate_config()->payload_len);

  rx_spi_packet[0] = TLM_PACKET;
  already_tlm = true;

  if (next_tlm_type == TELEMETRY_TYPE_LINK || !elrs_tlm_sender_active()) {
    rx_spi_packet[1] = TELEMETRY_TYPE_LINK;
    rx_spi_packet[2] = -rssi;                                                    // rssi
    rx_spi_packet[3] = (has_model_match << 7);                                   // no diversity
    rx_spi_packet[4] = uplink_lq | ((elrs_tlm_receiver_confirm() ? 1 : 0) << 7); // uplink_lq &&  msq confirm
    rx_spi_packet[5] = elrs_snr_mean_get(-16);
    rx_spi_packet[6] = 0;

    next_tlm_type = TELEMETRY_TYPE_DATA;
    tlm_burst_count = 1;
  } else {
    if (tlm_burst_count < tlm_burst_max) {
      tlm_burst_count++;
    } else {
      next_tlm_type = TELEMETRY_TYPE_LINK;
    }

    const uint8_t package_index = elrs_tlm_sender_current_payload((uint8_t *)rx_spi_packet + 2, 5);
    rx_spi_packet[1] = (package_index << ELRS_TELEMETRY_SHIFT) + TELEMETRY_TYPE_DATA;
  }

  const uint16_t crc = elrs_crc_calc(rx_spi_packet, 7, crc_initializer);
  rx_spi_packet[0] |= (crc >> 6) & 0xFC;
  rx_spi_packet[7] = crc & 0xFF;

  elrs_enter_tx(rx_spi_packet, current_air_rate_config()->payload_len);

  return true;
}

static void elrs_update_telemetry_burst() {
  if (tlm_burst_valid) {
    return;
  }
  tlm_burst_valid = true;

  uint32_t hz = rate_enum_to_hz(current_rf_pref_params()->rate);
  tlm_burst_max = TELEM_MIN_LINK_INTERVAL * hz / tlm_denom / 1000U;

  // Reserve one slot for LINK telemetry
  if (tlm_burst_max > 1) {
    --tlm_burst_max;
  } else {
    tlm_burst_max = 1;
  }

  // Notify the sender to adjust its expected throughput
  elrs_tlm_sender_update_rate(hz, tlm_denom, tlm_burst_max);
}

static void elrs_update_telemetry() {
  if (elrs_state != CONNECTED || current_air_rate_config()->tlm_interval == TLM_RATIO_NO_TLM) {
    return;
  }

  if (!elrs_tlm_sender_active()) {
    crsf_tlm_frame_start(tlm_buffer);

    uint32_t payload_size = 0;
    if (msp_new_data) {
      static uint8_t msp_seq = 0;
      static uint16_t msp_tx_sent = 0;

      uint8_t payload[CRSF_MSP_PAYLOAD_SIZE_MAX];

      uint8_t header_size = 0;
      if (msp_tx_sent == 0) { // first chunk
        payload[header_size++] = MSP_STATUS_START_MASK | (msp_seq++ & MSP_STATUS_SEQUENCE_MASK) | (1 << MSP_STATUS_VERSION_SHIFT);
        if (msp_is_error) {
          payload[0] |= MSP_STATUS_ERROR_MASK;
        }

        if (msp_tx_len > 0xFF) {
          payload[header_size++] = 0xFF;
          payload[header_size++] = msp_last_cmd;
          payload[header_size++] = (msp_tx_len >> 0) & 0xFF;
          payload[header_size++] = (msp_tx_len >> 8) & 0xFF;
        } else {
          payload[header_size++] = msp_tx_len;
          payload[header_size++] = msp_last_cmd;
        }

      } else {
        payload[header_size++] = (msp_seq++ & MSP_STATUS_SEQUENCE_MASK) | (1 << MSP_STATUS_VERSION_SHIFT);
      }

      const uint8_t msp_size = min(CRSF_MSP_PAYLOAD_SIZE_MAX - header_size, msp_tx_len - msp_tx_sent);
      memcpy(payload + header_size, msp_tx_buffer + msp_tx_sent, msp_size);
      msp_tx_sent += msp_size;

      if (msp_tx_sent >= msp_tx_len) {
        msp_tx_len = 0;
        msp_tx_sent = 0;
        msp_new_data = false;
      }

      payload_size = crsf_tlm_frame_msp_resp(tlm_buffer, msp_origin, payload, msp_size + header_size);
    } else {
      if (tlm_device_info_pending) {
        payload_size = crsf_tlm_frame_device_info(tlm_buffer);
        tlm_device_info_pending = false;
      } else {
        payload_size = crsf_tlm_frame_battery_sensor(tlm_buffer);
      }
    }

    const uint32_t full_size = crsf_tlm_frame_finish(tlm_buffer, payload_size);
    elrs_tlm_sender_set_data(tlm_buffer, full_size);
  }

  elrs_update_telemetry_burst();
}

static bool elrs_vaild_packet() {
  elrs_read_packet(rx_spi_packet);

  const uint8_t type = rx_spi_packet[0] & 0b11;
  const uint16_t their_crc = (((uint16_t)(rx_spi_packet[0] & 0b11111100)) << 6) | rx_spi_packet[7];

  // For smHybrid the CRC only has the rx_spi_packet type in byte 0
  // For smHybridWide the FHSS slot is added to the CRC in byte 0 on RC_DATA_PACKETs
  if (type == RC_DATA_PACKET && bind_storage.elrs.switch_mode == SWITCH_WIDE_OR_8CH) {
    const uint8_t fhss_result = (ota_nonce % current_air_rate_config()->fhss_hop_interval) + 1;
    rx_spi_packet[0] = type | (fhss_result << 2);
  } else {
    rx_spi_packet[0] = type;
  }

  const uint16_t our_crc = elrs_crc_calc(rx_spi_packet, OTA4_CRC_CALC_LEN, crc_initializer);

  return their_crc == our_crc;
}

static void elrs_connection_lost() {
  elrs_state = DISCONNECTED;
  elrs_timer_state = TIMER_DISCONNECTED;
  elrs_timer_stop();

  flags.failsafe = 1;

  already_hop = false;
  rf_mode_cycle_multiplier = 1;
  uplink_lq = 0;
  state.rx_rssi = 0;

  elrs_lq_reset();
  fhss_reset();
  elrs_phase_reset();

  elrs_set_rate(next_rate, fhss_get_sync_freq(), UID[5] & 0x01, elrs_get_uid_mac_seed(), crc_initializer);
  elrs_enter_rx(rx_spi_packet);
}

static void elrs_connection_tentative(uint32_t now) {
  elrs_state = TENTATIVE;
  elrs_timer_state = TIMER_DISCONNECTED;

  has_model_match = false;

  last_rf_mode_cycle_millis = now;

  fhss_reset();
  elrs_phase_reset();
  elrs_snr_mean_reset();

  if (!in_binding_mode && !elrs_timer_is_running()) {
    elrs_timer_resume(current_air_rate_config()->interval);
  }
}

static void elrs_connected(uint32_t now) {
  if (elrs_state == CONNECTED) {
    return;
  }

  connected_millis = now;

  flags.rx_ready = 1;
  flags.failsafe = 0;

  flags.rx_mode = RXMODE_NORMAL;

  elrs_state = CONNECTED;
  elrs_timer_state = TIMER_TENTATIVE;
}

static void elrs_cycle_rf_mode(uint32_t now) {
  if (elrs_state == CONNECTED || in_binding_mode) {
    return;
  }

  if ((now - last_rf_mode_cycle_millis) <= (fhss_rf_mode_cycle_interval() * rf_mode_cycle_multiplier)) {
    return;
  }

  last_rf_mode_cycle_millis = now;
  sync_packet_millis = now;

  next_rate = (current_air_rate_config()->index + 1) % ELRS_RATE_MAX;

  elrs_lq_reset();
  fhss_reset();
  elrs_phase_reset();

  elrs_set_rate(next_rate, fhss_get_sync_freq(), UID[5] & 0x01, elrs_get_uid_mac_seed(), crc_initializer);
  elrs_enter_rx(rx_spi_packet);

  rf_mode_cycle_multiplier = 1;
}

static void elrs_enter_binding_mode() {
  if ((elrs_state == CONNECTED) || in_binding_mode) {
    return;
  }

  flags.rx_mode = RXMODE_BIND;
  state.rx_status = RX_SPI_STATUS_BINDING;

  UID[0] = bind_uid[0];
  UID[1] = bind_uid[1];
  UID[2] = bind_uid[2];
  UID[3] = bind_uid[3];
  UID[4] = bind_uid[4];
  UID[5] = bind_uid[5];

  crc_initializer = 0;
  in_binding_mode = true;

  next_rate = ERLS_RATE_BIND;

  elrs_set_rate(next_rate, fhss_get_sync_freq(), UID[5] & 0x01, elrs_get_uid_mac_seed(), crc_initializer);
  elrs_enter_rx(rx_spi_packet);
}

static void elrs_setup_bind(const volatile uint8_t *packet) {
  for (uint8_t i = 0; i < 4; i++) {
    UID[i + 2] = packet[i + 3];
  }

  bind_storage.elrs.is_set = 0x1;
  bind_storage.elrs.magic = 0x37;

  crc_initializer = ((UID[4] << 8) | UID[5]) ^ ELRS_OTA_VERSION_ID;
  fhss_randomize(elrs_get_uid_mac_seed());

  last_rf_mode_cycle_millis = 0;
  in_binding_mode = false;
  next_rate = ELRS_RATE_DEFAULT;

  elrs_connection_lost();
}

static void elrs_unpack_channels(uint32_t channels[4], uint8_t *data) {
  const uint32_t num_of_channels = 4;
  const uint32_t src_bits = 10;
  const uint32_t dst_bits = 11;
  const uint32_t input_channel_mask = (1 << src_bits) - 1;
  const uint32_t precision_shift = dst_bits - src_bits;

  uint8_t bits_merged = 0;
  uint32_t read_value = 0;
  uint32_t read_byte_index = 0;
  for (uint8_t n = 0; n < num_of_channels; n++) {
    while (bits_merged < src_bits) {
      uint8_t read_byte = data[read_byte_index++];
      read_value |= ((uint32_t)read_byte) << bits_merged;
      bits_merged += 8;
    }

    channels[n] = (read_value & input_channel_mask) << precision_shift;
    read_value >>= src_bits;
    bits_merged -= src_bits;
  }
}

static uint8_t elrs_unpack_3b_switch(uint16_t val) {
  switch (val) {
  case 6:
  case 7:
  case 0:
    return 0;
  case 5:
    return 1;
  default:
    return val > 3 ? 1 : 0;
  }
}

static uint8_t elrs_unpack_n_switch(uint16_t val, uint16_t max) {
  return val > (max / 2);
}

static void elrs_sample_aux0(bool aux0_value) {
  static bool last_aux0_value = false;
  state.aux[AUX_CHANNEL_0] = (!last_aux0_value && !aux0_value) ? 0 : 1;
  last_aux0_value = aux0_value;
}

static bool elrs_unpack_switches_hybrid(const volatile uint8_t *rx_spi_packet) {
  const uint8_t switch_byte = rx_spi_packet[6];

  elrs_sample_aux0((switch_byte & 0b10000000));

  const uint8_t index = (switch_byte & 0b111000) >> 3;
  const uint16_t value = elrs_unpack_3b_switch(switch_byte & 0b111);
  switch (index) {
  case 0:
    state.aux[AUX_CHANNEL_1] = value;
    break;
  case 1:
    state.aux[AUX_CHANNEL_2] = value;
    break;
  case 2:
    state.aux[AUX_CHANNEL_3] = value;
    break;
  case 3:
    state.aux[AUX_CHANNEL_4] = value;
    break;
  case 4:
    state.aux[AUX_CHANNEL_5] = value;
    break;
  case 5:
    state.aux[AUX_CHANNEL_6] = value;
    break;
  case 6: // Because AUX1 (index 0) is the low latency switch, the low bit
  case 7: // of the switchIndex can be used as data, and arrives as index "6"
    state.aux[AUX_CHANNEL_7] = elrs_unpack_n_switch(switch_byte & 0b1111, 15);
    break;
  }

  state.aux[AUX_CHANNEL_8] = 0;
  state.aux[AUX_CHANNEL_9] = 0;
  state.aux[AUX_CHANNEL_10] = 0;
  state.aux[AUX_CHANNEL_11] = 0;

  // TelemetryStatus bit
  return switch_byte & (1 << 6);
}

static uint8_t elrs_hybrid_wide_nonce_to_switch_index(uint8_t nonce) {
  // Returns the sequence (0 to 7, then 0 to 7 rotated left by 1):
  // 0, 1, 2, 3, 4, 5, 6, 7,
  // 1, 2, 3, 4, 5, 6, 7, 0
  // Because telemetry can occur on every 2, 4, 8, 16, 32, 64, 128th packet
  // this makes sure each of the 8 values is sent at least once every 16 packets
  // regardless of the TLM ratio
  // Index 7 also can never fall on a telemetry slot
  return ((nonce & 0b111) + ((nonce >> 3) & 0b1)) % 8;
}

static bool elrs_unpack_switches_wide(const volatile uint8_t *packet) {
  static bool telemetry_status = false;

  const uint8_t switch_byte = packet[6];

  elrs_sample_aux0((switch_byte & 0b10000000));

  const uint8_t index = elrs_hybrid_wide_nonce_to_switch_index(ota_nonce);

  bool tlm_in_every_packet = (tlm_denom < 8);
  if (tlm_in_every_packet || index == 7) {
    telemetry_status = (switch_byte & 0b01000000) >> 6;
  }

  if (index == 7) {
    // crsf->LinkStatistics.uplink_TX_Power = switch_byte & 0b111111;
    return telemetry_status;
  }

  uint8_t bins = 0;
  uint16_t switch_value = 0;
  if (tlm_in_every_packet) {
    bins = 63;
    switch_value = switch_byte & 0b111111; // 6-bit
  } else {
    bins = 127;
    switch_value = switch_byte & 0b1111111; // 7-bit
  }

  switch_value = elrs_unpack_n_switch(switch_value, bins);
  state.aux[AUX_CHANNEL_1 + index] = switch_value;

  return telemetry_status;
}

static void elrs_msp_send(msp_magic_t magic, uint8_t direction, uint16_t code, const uint8_t *data, uint16_t len) {
  msp_last_cmd = code;

  if (len > 0 && data) {
    memcpy(msp_tx_buffer, data, len);
  }
  msp_tx_len = len;

  msp_is_error = direction == '!';
  msp_new_data = true;
}

static void elrs_msp_process(uint8_t *buf, uint32_t len) {
  static uint8_t msp_rx_buffer[MSP_BUFFER_SIZE];
  static msp_t msp = {
      .buffer = msp_rx_buffer,
      .buffer_size = MSP_BUFFER_SIZE,
      .buffer_offset = 0,
      .send = elrs_msp_send,
      .device = MSP_DEVICE_SPI_RX,
  };

  if (buf[7] == MSP_SET_RX_CONFIG && buf[8] == MSP_ELRS_MODEL_ID) {
    bind_storage.elrs.model_id = buf[9] ^ 0xFF;
    return;
  }

  switch (buf[2]) {
  case CRSF_FRAMETYPE_DEVICE_PING:
    tlm_device_info_pending = true;
    break;

  case CRSF_FRAMETYPE_MSP_WRITE:
  case CRSF_FRAMETYPE_MSP_REQ:
    msp_origin = buf[4];
    msp_process_telemetry(&msp, buf + 5, len - 3);
    break;
  }
}

static bool elrs_process_packet(uint32_t packet_time) {
  if (!elrs_vaild_packet()) {
    return false;
  }

  bool channels_received = false;

  elrs_phase_ext_event(packet_time + PACKET_TO_TOCK_SLACK);
  last_valid_packet_millis = time_millis();

  elrs_last_packet_stats(&raw_rssi, &raw_snr);
  rssi = elrs_lpf_update(&rssi_lpf, raw_rssi);
  elrs_snr_mean_add(raw_snr);

  rx_lqi_got_packet();

  if (profile.receiver.lqi_source == RX_LQI_SOURCE_DIRECT) {
    rx_lqi_update_direct(uplink_lq);
  }
  if (profile.receiver.lqi_source == RX_LQI_SOURCE_CHANNEL) {
    rx_lqi_update_direct(0.f);
  }

  elrs_lq_add();

  rf_mode_cycle_multiplier = RF_MODE_CYCLE_MULTIPLIER_SLOW;

  const uint8_t type = rx_spi_packet[0] & 0b11;
  switch (type) {
  case SYNC_PACKET: {
    if (rx_spi_packet[4] != UID[3] || rx_spi_packet[5] != UID[4]) {
      break;
    }

    if ((rx_spi_packet[6] & ~MODELMATCH_MASK) != (UID[5] & ~MODELMATCH_MASK)) {
      break;
    }

    sync_packet_millis = time_millis();

    next_rate = ((rx_spi_packet[3] & 0b11110000) >> 4);

    // Switch mode can only change when disconnected, and happens on the main thread
    if (elrs_state == DISCONNECTED) {
      // Add one to the mode because next_switch_mode_pending==0 means no switch pending
      // and that's also a valid switch mode. The 1 is removed when this is handled
      next_switch_mode_pending = (rx_spi_packet[3] & 0b1) + 1;
    }

    const uint8_t telemetry_rate_index = TLM_RATIO_NO_TLM + ((rx_spi_packet[3] & 0b00001110) >> 1);
    const uint8_t new_tlm_denom = tlm_ratio_enum_to_value(telemetry_rate_index);
    if (new_tlm_denom != tlm_denom) {
      tlm_denom = new_tlm_denom;
      tlm_burst_valid = false;
    }

    const uint8_t model_xor = (~elrs_get_model_id()) & MODELMATCH_MASK;
    const bool model_match = rx_spi_packet[6] == (UID[5] ^ model_xor);

    if (elrs_state == DISCONNECTED ||
        (ota_nonce != rx_spi_packet[2]) ||
        (fhss_get_index() != rx_spi_packet[1]) ||
        (has_model_match != model_match)) {

      fhss_set_index(rx_spi_packet[1]);
      ota_nonce = rx_spi_packet[2];

      elrs_connection_tentative(time_millis());

      has_model_match = model_match;
    }
    break;
  }
  case RC_DATA_PACKET: {
    if (!has_model_match || next_switch_mode_pending) {
      // dont update channels if we dont have a match
      break;
    }

    uint32_t crsf_channels[4];
    elrs_unpack_channels(crsf_channels, (uint8_t *)rx_spi_packet + 1);

    // AETR channel order
    const float channels[4] = {
        ((float)crsf_channels[0] - 1024.f) * (1.0f / 1024.f),
        ((float)crsf_channels[1] - 1024.f) * (1.0f / 1024.f),
        ((float)crsf_channels[2] - 1024.f) * (1.0f / 1024.f),
        ((float)crsf_channels[3] - 1024.f) * (1.0f / 1024.f),
    };

    rx_map_channels(channels);

    channels_received = true;

    bool tlm_confirm = false;

    switch (bind_storage.elrs.switch_mode) {
    default:
    case SWITCH_WIDE_OR_8CH:
      tlm_confirm = elrs_unpack_switches_wide(rx_spi_packet);
      break;

    case SWITCH_HYBRID_OR_16CH:
      tlm_confirm = elrs_unpack_switches_hybrid(rx_spi_packet);
      break;
    }

    elrs_tlm_sender_confirm_payload(tlm_confirm);
    break;
  }
  case MSP_DATA_PACKET: {
    if (in_binding_mode && rx_spi_packet[1] == 1 && rx_spi_packet[2] == MSP_ELRS_BIND) {
      elrs_setup_bind(rx_spi_packet);
      break;
    }

    if (elrs_state != CONNECTED) {
      break;
    }

    const bool confirm = elrs_tlm_receiver_confirm();
    elrs_tlm_receiver_receive_data(rx_spi_packet[1], (uint8_t *)rx_spi_packet + 2, 5);
    if (confirm != elrs_tlm_receiver_confirm()) {
      next_tlm_type = TELEMETRY_TYPE_LINK;
    }

    if (elrs_tlm_receiver_has_finished_data()) {
      elrs_msp_process(msp_buffer, msp_buffer[1]);
      elrs_tlm_receiver_unlock();
    }
    break;
  }
  default:
    break;
  }

  return channels_received;
}

// this is 180 out of phase with the other callback, occurs mid-packet reception
void elrs_handle_tick() {
  elrs_phase_update(elrs_state);
  ota_nonce++;

  uplink_lq = elrs_lq_get();

  // Only advance the LQI period counter if we didn't send Telemetry this period
  if (!already_tlm) {
    elrs_lq_inc();
  }

  already_hop = false;
  already_tlm = false;
}

void elrs_handle_tock() {
  const uint32_t time = time_micros();
  elrs_phase_int_event(time);

  const bool did_hop = elrs_hop();
  const bool did_tlm = elrs_tlm();
  if (!did_hop && !did_tlm && elrs_lq_current_is_set()) {
    elrs_freq_correct();
  }
}

void rx_expresslrs_init() {
  if (!radio_is_init && !elrs_radio_init()) {
    radio_is_init = false;
    return;
  }

  fhss_randomize(elrs_get_uid_mac_seed());
  elrs_phase_init();
  elrs_lq_reset();
  elrs_lpf_init(&rssi_lpf, 5);

  elrs_tlm_sender_reset();
  elrs_tlm_sender_set_max_package_index(ELRS_TELEMETRY_MAX_PACKAGES);

  elrs_tlm_receiver_reset();
  elrs_tlm_receiver_set_max_package_index(ELRS_MSP_MAX_PACKAGES);
  elrs_tlm_receiver_set_data_to_receive(msp_buffer, ELRS_MSP_BUFFER_SIZE);

  next_rate = ELRS_RATE_DEFAULT;
  crc_initializer = ((UID[4] << 8) | UID[5]) ^ ELRS_OTA_VERSION_ID;
  rf_mode_cycle_multiplier = 1;
  last_rf_mode_cycle_millis = time_millis();

  elrs_set_rate(next_rate, fhss_get_sync_freq(), (UID[5] & 0x01), elrs_get_uid_mac_seed(), crc_initializer);
  elrs_set_switch_mode(SWITCH_WIDE_OR_8CH, current_air_rate_config()->payload_len);
  elrs_timer_init(current_air_rate_config()->interval);
  elrs_enter_rx(rx_spi_packet);

  radio_is_init = true;
  in_binding_mode = false;
}

uint16_t rx_expresslrs_smoothing_cutoff() {
  switch (current_air_rate_config()->rate) {
  case RATE_FLRC_1000HZ:
    return 450;
  case RATE_FLRC_500HZ:
  case RATE_DVDA_500HZ:
  case RATE_LORA_500HZ:
    return 225;
  case RATE_LORA_333HZ_8CH:
    return 150;
  case RATE_DVDA_250HZ:
  case RATE_LORA_250HZ:
    return 112;
  case RATE_LORA_200HZ:
    return 90;
  case RATE_LORA_150HZ:
    return 67;
  case RATE_LORA_100HZ_8CH:
  case RATE_LORA_100HZ:
    return 45;
  case RATE_LORA_50HZ:
    return 22;
  case RATE_LORA_25HZ:
    return 11;
  case RATE_LORA_4HZ:
    return 1;
  }

  return 1;
}

bool rx_expresslrs_check() {
  bool channels_received = false;

  if (!radio_is_init) {
    return channels_received;
  }

  static uint32_t last_time = 0;
  if ((elrs_state == CONNECTED) && (time_micros() - last_time) > LOCKUP_TIMEOUT_US) {
    // we lost 10000us since last visit (flash save, etc)
    // link has become unsustainable
    elrs_connection_lost();
  }
  last_time = time_micros();

  const elrs_irq_status_t irq = elrs_get_irq_status();

  // it is possible we caught a packet during boot, but it will be stale by now
  // read the irq state above, but ignore it during the first run of this function
  static bool has_run_once = false;
  if (!has_run_once) {
    // delay mode cycle a bit
    last_rf_mode_cycle_millis = time_millis();
    has_run_once = true;
    return channels_received;
  }

  if (irq == IRQ_RX_DONE) {
    channels_received = elrs_process_packet(packet_time);
  }
  // TX handled in irq

  const uint32_t time_ms = time_millis();

  if ((elrs_state != DISCONNECTED) && next_rate != current_air_rate_config()->index) {
    elrs_connection_lost();
    sync_packet_millis = time_ms;
    last_rf_mode_cycle_millis = time_ms;
  }

  if ((elrs_state == TENTATIVE) && ((time_ms - sync_packet_millis) > current_rf_pref_params()->rx_lock_timeout_ms)) {
    elrs_connection_lost();
    sync_packet_millis = time_ms;
    last_rf_mode_cycle_millis = time_ms;
  }

  elrs_cycle_rf_mode(time_ms);

  if ((elrs_state == CONNECTED) && ((int32_t)(time_ms - last_valid_packet_millis) > current_rf_pref_params()->disconnect_timeout_ms)) {
    elrs_connection_lost(time_ms);
  }

  if ((elrs_state == CONNECTED) && ((int32_t)(time_ms - sync_packet_millis) > SYNC_TIMEOUT_MS)) {
    elrs_connection_lost(time_ms);
  }

  if ((elrs_state == TENTATIVE) && (abs(pl_state.offset_dx) <= 10) && (pl_state.offset < 100) && (elrs_lq_get_raw() > fhss_min_lq_for_chaos())) {
    elrs_connected(time_ms);
  }

  if ((elrs_timer_state == TIMER_TENTATIVE) && ((time_ms - connected_millis) > CONSIDER_CONN_GOOD_MILLIS) && (abs(pl_state.offset_dx) <= 5)) {
    elrs_timer_state = TIMER_LOCKED;
  }

  if (bind_storage.elrs.is_set == 0x0 && bind_storage.elrs.magic != 0x37) {
    elrs_enter_binding_mode();
  } else {
    state.rx_status = RX_SPI_STATUS_BOUND;
  }

  elrs_update_telemetry();

  if (next_switch_mode_pending) {
    elrs_set_switch_mode(next_switch_mode_pending - 1, current_air_rate_config()->payload_len);
    next_switch_mode_pending = 0;
  }

  rx_lqi_update();

  if (profile.receiver.lqi_source == RX_LQI_SOURCE_PACKET_RATE) {
    rx_lqi_update_from_fps(rate_enum_to_hz(current_rf_pref_params()->rate));
  }

  return channels_received;
}

#endif