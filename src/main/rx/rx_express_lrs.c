#include "rx_express_lrs.h"

#include <stdbool.h>
#include <stdlib.h>
#include <string.h>

#include "control.h"
#include "drv_gpio.h"
#include "drv_time.h"
#include "flash.h"
#include "project.h"
#include "usb_configurator.h"

#if defined(RX_EXPRESS_LRS) && (defined(USE_SX127X) || defined(USE_SX128X))

#define MSP_SET_RX_CONFIG 45
#define MSP_VTX_CONFIG 88     //out message         Get vtx settings - betaflight
#define MSP_SET_VTX_CONFIG 89 //in message          Set vtx settings - betaflight
#define MSP_EEPROM_WRITE 250  //in message          no param

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
#define PACKET_TO_TOCK_SLACK 200
#define CONSIDER_CONN_GOOD_MILLIS 1000
#define RF_MODE_CYCLE_MULTIPLIER_SLOW 10

#define UID bind_storage.elrs.uid

extern volatile uint8_t fhss_index;
extern volatile elrs_phase_lock_state_t pl_state;

extern int32_t fhss_update_freq_correction(uint8_t value);
extern void fhss_randomize(int32_t seed);
extern uint32_t fhss_get_freq(uint16_t index);
extern uint32_t fhss_get_sync_freq();
extern uint32_t fhss_next_freq();
extern void fhss_reset();
extern uint8_t fhss_min_lq_for_chaos();
extern uint32_t fhss_rf_mode_cycle_interval();

extern void crc14_init();
extern uint16_t crc14_calc(uint8_t *data, uint8_t len, uint16_t crc);

extern expresslrs_mod_settings_t *current_air_rate_config();
extern expresslrs_rf_pref_params_t *current_rf_pref_params();

extern uint8_t tlm_ratio_enum_to_value(expresslrs_tlm_ratio_t val);
extern uint16_t rate_enum_to_hz(expresslrs_rf_rates_t val);

uint8_t packet[ELRS_BUFFER_SIZE];
elrs_timer_state_t elrs_timer_state = TIMER_DISCONNECTED;
volatile uint8_t nonce_rx = 0;

static volatile elrs_state_t elrs_state = DISCONNECTED;

static volatile bool already_hop = false;
static volatile bool already_tlm = false;
static volatile bool needs_hop = false;

static bool in_binding_mode = false;
static bool has_model_match = false;

static uint32_t connected_millis = 0;
static uint32_t sync_packet_millis = 0;
static uint32_t last_valid_packet_millis = 0;
static uint32_t last_rf_mode_cycle_millis = 0;

static uint32_t next_rate = ELRS_RATE_DEFAULT;
static uint32_t rf_mode_cycle_multiplier = 1;

static int8_t raw_rssi = 0;
static int8_t raw_snr = 0;
static uint8_t uplink_lq = 0;

static elrs_lpf_t rssi_lpf;
static int8_t rssi;

static uint16_t crc_initializer = 0;
static uint8_t bind_uid[6] = {0, 1, 2, 3, 4, 5};

static uint8_t msp_buffer[ELRS_MSP_BUFFER_SIZE];
static uint8_t next_telemetry_type = TELEMETRY_TYPE_LINK;

static uint8_t elrs_get_model_id() {
  // invert value so default (0x0) equals no model match
  return bind_storage.elrs.model_id ^ 0xFF;
}

static bool elrs_hop() {
  if (already_hop || in_binding_mode) {
    return false;
  }

  const uint8_t modresult = (nonce_rx + 1) % current_air_rate_config()->fhss_hop_interval;
  if (modresult != 0) {
    return false;
  }

  elrs_set_frequency(fhss_next_freq());
  already_hop = true;

  const uint8_t tlm_mod = (nonce_rx + 1) % tlm_ratio_enum_to_value(current_air_rate_config()->tlm_interval);
  if (current_air_rate_config()->tlm_interval == TLM_RATIO_NO_TLM || tlm_mod != 0) {
    elrs_enter_rx(packet);
  }
  return true;
}

static bool elrs_tlm() {
  if (already_tlm) {
    return false;
  }

  const uint8_t tlm_mod = (nonce_rx + 1) % tlm_ratio_enum_to_value(current_air_rate_config()->tlm_interval);
  if (current_air_rate_config()->tlm_interval == TLM_RATIO_NO_TLM || tlm_mod != 0) {
    return false;
  }

  if (profile.receiver.lqi_source == RX_LQI_SOURCE_PACKET_RATE) {
    rx_lqi_update_spi_fps(rate_enum_to_hz(current_rf_pref_params()->rate));
  }

  packet[0] = 0b11;
  packet[1] = TELEMETRY_TYPE_LINK;
  packet[2] = -rssi;                          // rssi
  packet[3] = (has_model_match << 7);         // no diversity
  packet[4] = -raw_snr;                       // snr
  packet[5] = uplink_lq;                      // uplink_lq
  packet[6] = elrs_get_msp_confirm() ? 1 : 0; // msq confirm
  packet[7] = 0;

  const uint16_t crc = crc14_calc(packet, 7, crc_initializer);
  packet[0] |= (crc >> 6) & 0xFC;
  packet[7] = crc & 0xFF;

  already_tlm = true;

  elrs_enter_tx(packet);

  return true;
}

static bool elrs_vaild_packet() {
  elrs_read_packet(packet);

  const uint8_t type = packet[0] & 0b11;
  const uint16_t their_crc = (((uint16_t)(packet[0] & 0b11111100)) << 6) | packet[7];

  // For smHybrid the CRC only has the packet type in byte 0
  // For smHybridWide the FHSS slot is added to the CRC in byte 0 on RC_DATA_PACKETs
  if (type != RC_DATA_PACKET || bind_storage.elrs.switch_mode != SWITCH_HYBRID_WIDE) {
    packet[0] = type;
  } else {
    uint8_t fhss_result = nonce_rx % current_air_rate_config()->fhss_hop_interval;
    packet[0] = type | (fhss_result << 2);
  }

  const uint16_t our_crc = crc14_calc(packet, 7, crc_initializer);

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

  elrs_set_rate(next_rate, fhss_get_sync_freq(), UID[5] & 0x01);
  elrs_enter_rx(packet);
}

static void elrs_connection_tentative(uint32_t now) {
  elrs_state = TENTATIVE;
  elrs_timer_state = TIMER_DISCONNECTED;

  has_model_match = false;

  last_rf_mode_cycle_millis = now;

  fhss_reset();
  elrs_phase_reset();

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

  elrs_set_rate(next_rate, fhss_get_sync_freq(), UID[5] & 0x01);
  elrs_enter_rx(packet);

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

  next_rate = ELRS_RATE_DEFAULT;

  elrs_set_rate(next_rate, fhss_get_sync_freq(), UID[5] & 0x01);
  elrs_enter_rx(packet);
}

static void elrs_setup_bind(uint8_t *packet) {
  for (uint8_t i = 0; i < 4; i++) {
    UID[i + 2] = packet[i + 3];
  }

  bind_storage.elrs.is_set = 0x1;
  bind_storage.elrs.magic = 0x37;

  crc_initializer = (UID[4] << 8) | UID[5];

  const int32_t seed = ((int32_t)UID[2] << 24) + ((int32_t)UID[3] << 16) + ((int32_t)UID[4] << 8) + UID[5];
  fhss_randomize(seed);

  last_rf_mode_cycle_millis = 0;
  in_binding_mode = false;
  next_rate = ELRS_RATE_DEFAULT;

  elrs_connection_lost();
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

static void elrs_sample_aux0(uint8_t aux0_value) {
  static uint8_t last_aux0_value = 0;
  state.aux[AUX_CHANNEL_0] = (!last_aux0_value && !aux0_value) ? 0 : 1;
  last_aux0_value = aux0_value;
}

static bool elrs_unpack_hybrid_switches(uint8_t *packet) {
  const uint8_t switch_byte = packet[6];

  elrs_sample_aux0((switch_byte & 0b01000000) >> 6);

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
  return switch_byte & (1 << 7);
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

static bool elrs_unpack_hybrid_switches_wide(uint8_t *packet) {
  static bool telemetry_status = false;

  const uint8_t switch_byte = packet[6];

  elrs_sample_aux0((switch_byte & 0b10000000) >> 7);

  const uint8_t index = elrs_hybrid_wide_nonce_to_switch_index(nonce_rx);
  const uint8_t tlm_denom = tlm_ratio_enum_to_value(current_air_rate_config()->tlm_interval);

  bool tlm_in_every_packet = (tlm_denom < 8);
  if (tlm_in_every_packet || index == 7) {
    telemetry_status = (switch_byte & 0b01000000) >> 6;
  }

  if (index == 7) {
    //crsf->LinkStatistics.uplink_TX_Power = switch_byte & 0b111111;
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
  switch (index) {
  case 0:
    state.aux[AUX_CHANNEL_5] = switch_value;
    break;
  case 1:
    state.aux[AUX_CHANNEL_6] = switch_value;
    break;
  case 2:
    state.aux[AUX_CHANNEL_7] = switch_value;
    break;
  case 3:
    state.aux[AUX_CHANNEL_8] = switch_value;
    break;
  case 4:
    state.aux[AUX_CHANNEL_9] = switch_value;
    break;
  case 5:
    state.aux[AUX_CHANNEL_10] = switch_value;
    break;
  case 6:
    state.aux[AUX_CHANNEL_11] = switch_value;
    break;
  }

  return telemetry_status;
}

static void elrs_process_packet(uint32_t packet_time) {
  if (!elrs_vaild_packet()) {
    return;
  }

  elrs_phase_ext_event(packet_time + PACKET_TO_TOCK_SLACK);
  last_valid_packet_millis = time_millis();

  elrs_last_packet_stats(&raw_rssi, &raw_snr);
  rssi = elrs_lpf_update(&rssi_lpf, raw_rssi);

  if (profile.receiver.lqi_source == RX_LQI_SOURCE_DIRECT) {
    state.rx_rssi = uplink_lq;
  }
  if (profile.receiver.lqi_source == RX_LQI_SOURCE_PACKET_RATE) {
    rx_lqi_update_spi_fps(rate_enum_to_hz(current_rf_pref_params()->rate));
  }
  if (profile.receiver.lqi_source == RX_LQI_SOURCE_CHANNEL) {
    state.rx_rssi = 0.f;
  }

  elrs_lq_add();

  rf_mode_cycle_multiplier = RF_MODE_CYCLE_MULTIPLIER_SLOW;

  const uint8_t type = packet[0] & 0b11;
  switch (type) {
  case SYNC_PACKET: {
    if (packet[4] != UID[3] || packet[5] != UID[4]) {
      break;
    }

    if ((packet[6] & ~MODELMATCH_MASK) != (UID[5] & ~MODELMATCH_MASK)) {
      break;
    }

    sync_packet_millis = time_millis();

    next_rate = ((packet[3] & 0b11000000) >> 6);
    bind_storage.elrs.switch_mode = ((packet[3] & 0b00000110) >> 1);

    const uint8_t telemetry_rate_index = ((packet[3] & 0b00111000) >> 3);
    if (current_air_rate_config()->tlm_interval != telemetry_rate_index) {
      current_air_rate_config()->tlm_interval = telemetry_rate_index;
    }

    const uint8_t model_xor = (~elrs_get_model_id()) & MODELMATCH_MASK;
    const bool model_match = packet[6] == (UID[5] ^ model_xor);

    if (elrs_state == DISCONNECTED ||
        (nonce_rx != packet[2]) ||
        (fhss_index != packet[1]) ||
        (has_model_match != model_match)) {

      fhss_index = packet[1];
      nonce_rx = packet[2];

      elrs_connection_tentative(time_millis());

      has_model_match = model_match;
    }
    break;
  }
  case RC_DATA_PACKET: {
    if (!has_model_match) {
      // dont update channels if we dont have a match
      break;
    }

    const uint16_t channels[4] = {
        (packet[1] << 3) | ((packet[5] & 0b11000000) >> 5),
        (packet[2] << 3) | ((packet[5] & 0b00110000) >> 3),
        (packet[3] << 3) | ((packet[5] & 0b00001100) >> 1),
        (packet[4] << 3) | ((packet[5] & 0b00000011) << 1),
    };

    state.rx.axis[0] = (channels[0] - 990.5f) * 0.00125707103f;
    state.rx.axis[1] = (channels[1] - 990.5f) * 0.00125707103f;
    state.rx.axis[2] = (channels[3] - 990.5f) * 0.00125707103f;
    state.rx.axis[3] = (channels[2] - 191.0f) * 0.00062853551f;

    rx_apply_stick_calibration_scale();

    switch (bind_storage.elrs.switch_mode) {
    default:
    case SWITCH_1BIT:
    case SWITCH_HYBRID:
      elrs_unpack_hybrid_switches(packet);
      break;

    case SWITCH_HYBRID_WIDE:
      elrs_unpack_hybrid_switches_wide(packet);
      break;
    }
    break;
  }
  case MSP_DATA_PACKET: {
    if (in_binding_mode && packet[1] == 1 && packet[2] == MSP_ELRS_BIND) {
      elrs_setup_bind(packet);
      break;
    }

    if (elrs_state != CONNECTED) {
      break;
    }

    const bool confirm = elrs_get_msp_confirm();
    elrs_receive_msp(packet[1], packet + 2);
    if (confirm != elrs_get_msp_confirm()) {
      next_telemetry_type = TELEMETRY_TYPE_LINK;
    }

    if (elrs_msp_finished_data()) {
      if (msp_buffer[7] == MSP_SET_RX_CONFIG && msp_buffer[8] == MSP_ELRS_MODEL_ID) {
        bind_storage.elrs.model_id = msp_buffer[9] ^ 0xFF;
      }

      elrs_msp_restart();
    }

    break;
  }
  default:
    break;
  }
}

// this is 180 out of phase with the other callback, occurs mid-packet reception
void elrs_handle_tick() {
  elrs_phase_update(elrs_state);
  nonce_rx++;

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
  needs_hop = true;
}

void rx_init() {
  if (!elrs_radio_init()) {
    return;
  }

  const int32_t seed = ((int32_t)UID[2] << 24) + ((int32_t)UID[3] << 16) + ((int32_t)UID[4] << 8) + UID[5];
  fhss_randomize(seed);
  crc14_init();

  elrs_phase_init();
  elrs_lq_reset();
  elrs_lpf_init(&rssi_lpf, 5);
  elrs_setup_msp(ELRS_MSP_BUFFER_SIZE, msp_buffer, ELRS_MSP_BYTES_PER_CALL);

  crc_initializer = (UID[4] << 8) | UID[5];
  rf_mode_cycle_multiplier = 1;

  // only hybrid switches for now
  bind_storage.elrs.switch_mode = 1;

  elrs_set_rate(next_rate, fhss_get_sync_freq(), (UID[5] & 0x01));
  elrs_timer_init(current_air_rate_config()->interval);
  elrs_enter_rx(packet);
}

void rx_check() {
  const uint32_t time = time_micros();

  static uint32_t last_time = 0;
  if ((elrs_state == CONNECTED) && (time - last_time) > 10000) {
    // we lost 10000us since last visit (flash save, etc)
    // link has become unsustainable
    elrs_connection_lost();
  }
  last_time = time;

  const elrs_irq_status_t irq = elrs_get_irq_status();
  if (irq == IRQ_RX_DONE) {
    elrs_process_packet(time);
  } else if (irq == IRQ_TX_DONE) {
    elrs_enter_rx(packet);
  }

  if (needs_hop && irq != IRQ_RX_DONE) {
    needs_hop = false;

    const bool did_hop = elrs_hop();
    const bool did_tlm = elrs_tlm();
    if (!did_hop && !did_tlm && elrs_lq_current_is_set()) {
      elrs_freq_correct();
    }
  }

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

  if ((elrs_state == TENTATIVE) && (abs(pl_state.offset_dx) <= 10) && (pl_state.offset < 100) && (elrs_lq_get() > fhss_min_lq_for_chaos())) {
    elrs_connected(time_ms);
  }

  if ((elrs_timer_state == TIMER_TENTATIVE) && ((time_ms - connected_millis) > CONSIDER_CONN_GOOD_MILLIS) && (abs(pl_state.offset_dx) <= 5)) {
    elrs_timer_state = TIMER_LOCKED;
  }

  if (bind_storage.elrs.is_set == 0x0 && bind_storage.elrs.magic != 0x37 && !in_binding_mode) {
    elrs_enter_binding_mode();
  } else {
    state.rx_status = RX_SPI_STATUS_BOUND;
  }
}

#endif