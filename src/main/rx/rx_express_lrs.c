#include "rx_express_lrs.h"

#include <stdbool.h>
#include <stdlib.h>
#include <string.h>

#include "control.h"
#include "drv_gpio.h"
#include "drv_time.h"
#include "project.h"
#include "usb_configurator.h"

#if defined(RX_EXPRESS_LRS) && (defined(USE_SX127X) || defined(USE_SX128X))

#define TELEMETRY_TYPE_LINK 0x01
#define TELEMETRY_TYPE_DATA 0x02

#define RC_DATA_PACKET 0b00
#define MSP_DATA_PACKET 0b01
#define SYNC_PACKET 0b10
#define TLM_PACKET 0b11

#define DEBUG_PIN PIN_A10

// Desired buffer time between Packet ISR and Tock ISR
#define PACKET_TO_TOCK_SLACK 200

#define CONSIDER_CONN_GOOD_MILLIS 1000

extern volatile uint8_t fhss_index;

extern int32_t fhss_update_freq_correction(uint8_t value);
extern void fhss_randomize(int32_t seed);
extern uint32_t fhss_get_freq(uint16_t index);
extern uint32_t fhss_next_freq();
extern void fhss_reset();
extern uint8_t fhss_min_lq_for_chaos();

extern void crc14_init();
extern uint16_t crc14_calc(uint8_t *data, uint8_t len, uint16_t crc);

extern expresslrs_mod_settings_t *current_air_rate_config();
extern expresslrs_rf_pref_params_t *current_rf_pref_params();

extern volatile elrs_phase_lock_state_t pl_state;

static volatile elrs_state_t elrs_state = DISCONNECTED;

static volatile bool already_hop = false;
static volatile bool already_tlm = false;

static volatile bool needs_hop = false;

uint8_t packet[ELRS_BUFFER_SIZE];
elrs_timer_state_t elrs_timer_state = TIMER_DISCONNECTED;
volatile uint8_t nonce_rx = 0;

static uint32_t sync_packet_millis = 0;
static uint32_t last_valid_packet_millis = 0;
static uint32_t connected_millis = 0;

static uint32_t next_rate = 0;

static int8_t raw_rssi = 0;
static int8_t raw_snr = 0;
static uint8_t uplink_lq = 0;

static uint8_t UID[6] = {221, 251, 226, 34, 222, 25};
// static uint8_t UID[6] = {0, 1, 2, 3, 4, 5};

uint8_t tlm_ratio_enum_to_value(expresslrs_tlm_ratio_t val) {
  switch (val) {
  case TLM_RATIO_NO_TLM:
    return 1;
    break;
  case TLM_RATIO_1_2:
    return 2;
    break;
  case TLM_RATIO_1_4:
    return 4;
    break;
  case TLM_RATIO_1_8:
    return 8;
    break;
  case TLM_RATIO_1_16:
    return 16;
    break;
  case TLM_RATIO_1_32:
    return 32;
    break;
  case TLM_RATIO_1_64:
    return 64;
    break;
  case TLM_RATIO_1_128:
    return 128;
    break;
  default:
    return 0;
  }
}

static bool elrs_hop() {
  if (already_hop) {
    return false;
  }

  const uint8_t modresult = (nonce_rx + 1) % current_air_rate_config()->fhss_hop_interval;
  if (modresult != 0) {
    return false;
  }

  elrs_set_frequency(fhss_next_freq());
  already_hop = true;

  static bool debug_pin_state = false;
  if (debug_pin_state) {
    gpio_pin_reset(DEBUG_PIN);
    debug_pin_state = false;
  } else {
    gpio_pin_set(DEBUG_PIN);
    debug_pin_state = true;
  }

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

  packet[0] = 0b11;
  packet[1] = TELEMETRY_TYPE_LINK;
  packet[2] = -raw_rssi; // rssi
  packet[3] = 0;         // no diversity
  packet[4] = -raw_snr;  // snr
  packet[5] = uplink_lq; // uplink_lq
  packet[6] = 0;         // msq confirm
  packet[7] = 0;

  const uint16_t crc_initializer = (UID[4] << 8) | UID[5];
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
  volatile const uint16_t their_crc = (((uint16_t)(packet[0] & 0b11111100)) << 6) | packet[7];

  // reset first byte to type only so crc passes
  packet[0] = type;

  const uint16_t crc_initializer = (UID[4] << 8) | UID[5];
  volatile const uint16_t our_crc = crc14_calc(packet, 7, crc_initializer);

  return their_crc == our_crc;
}

static void elrs_connection_lost() {
  elrs_state = DISCONNECTED;
  elrs_timer_state = TIMER_DISCONNECTED;
  elrs_timer_stop();

  already_hop = false;

  fhss_index = 0;

  elrs_lq_reset();
  fhss_reset();
  elrs_phase_reset();

  elrs_set_rate(next_rate, fhss_get_freq(0), UID[5] & 0x01);
  elrs_enter_rx(packet);
}

static void elrs_connection_tentative() {
  elrs_state = TENTATIVE;
  elrs_timer_state = TIMER_DISCONNECTED;

  fhss_reset();
  elrs_phase_reset();

  elrs_timer_resume(current_air_rate_config()->interval);
}

static void elrs_connected() {
  if (elrs_state == CONNECTED) {
    return;
  }

  connected_millis = time_millis();

  flags.rx_ready = 1;
  flags.failsafe = 0;

  elrs_state = CONNECTED;
  elrs_timer_state = TIMER_TENTATIVE;
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

void elrs_process_packet(uint32_t packet_time) {
  if (!elrs_vaild_packet()) {
    return;
  }

  elrs_phase_ext_event(packet_time + PACKET_TO_TOCK_SLACK);
  last_valid_packet_millis = time_millis();

  elrs_last_packet_stats(&raw_rssi, &raw_snr);
  elrs_lq_add();

  const uint8_t type = packet[0] & 0b11;
  switch (type) {
  case SYNC_PACKET: {
    if (packet[4] != UID[3] || packet[5] != UID[4] || packet[6] != UID[5]) {
      break;
    }

    sync_packet_millis = time_millis();

    const uint8_t rate_index = ((packet[3] & 0b11000000) >> 6);
    const uint8_t telemetry_rate_index = ((packet[3] & 0b00111000) >> 3);
    // const uint8_t switch_mode = ((packet[3] & 0b00000110) >> 1);

    if (rate_index != current_air_rate_config()->index) {
      next_rate = rate_index;
      elrs_connection_lost();
    }

    if (current_air_rate_config()->tlm_interval != telemetry_rate_index) {
      current_air_rate_config()->tlm_interval = telemetry_rate_index;
    }

    if (elrs_state == DISCONNECTED ||
        (nonce_rx != packet[2]) ||
        (fhss_index != packet[1])) {
      fhss_index = packet[1];
      nonce_rx = packet[2];

      elrs_connection_tentative();
    }
    break;
  }
  case RC_DATA_PACKET: {
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

    state.aux[AUX_CHANNEL_0] = (packet[6] & 0b00001000) ? 1 : 0;
    state.aux[AUX_CHANNEL_1] = (packet[6] & 0b00000100) ? 1 : 0;
    state.aux[AUX_CHANNEL_2] = (packet[6] & 0b00000010) ? 1 : 0;
    state.aux[AUX_CHANNEL_3] = (packet[6] & 0b00000001) ? 1 : 0;
    break;
  }
  default:
    break;
  }
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

  elrs_set_rate(next_rate, fhss_get_freq(0), (UID[5] & 0x01));
  elrs_timer_init(current_air_rate_config()->interval);
  elrs_enter_rx(packet);
}

void rx_check() {
  const uint32_t packet_time = time_micros();

  const elrs_irq_status_t irq = elrs_get_irq_status();
  if (irq == IRQ_RX_DONE) {
    elrs_process_packet(packet_time);
  } else if (irq == IRQ_TX_DONE) {
    elrs_enter_rx(packet);
  }

  if (needs_hop) {
    needs_hop = false;

    const bool did_hop = elrs_hop();
    const bool did_tlm = elrs_tlm();
    if (!did_hop && !did_tlm && elrs_lq_current_is_set()) {
      elrs_freq_correct();
    }
  }

  if ((elrs_state == TENTATIVE) && ((time_millis() - sync_packet_millis) > current_rf_pref_params()->rf_mode_cycle_addtional_time)) {
    sync_packet_millis = time_millis();
    elrs_connection_lost();
  }

  if ((elrs_state == CONNECTED) && ((time_millis() - last_valid_packet_millis) > current_rf_pref_params()->rf_mode_cycle_interval)) {
    elrs_connection_lost();
  }

  if ((elrs_state == TENTATIVE) && (abs(pl_state.offset_dx) <= 10) && (pl_state.offset < 100) && (uplink_lq > fhss_min_lq_for_chaos())) {
    elrs_connected();
  }

  if ((elrs_timer_state == TIMER_TENTATIVE) && ((time_millis() - connected_millis) > CONSIDER_CONN_GOOD_MILLIS) && (abs(pl_state.offset_dx) <= 5)) {
    elrs_timer_state = TIMER_LOCKED;
  }
}

#endif