#include "rx_express_lrs.h"

#include <stdbool.h>
#include <string.h>

#include "control.h"
#include "drv_time.h"
#include "project.h"
#include "usb_configurator.h"

#if defined(RX_EXPRESS_LRS) && (defined(USE_SX127X) || defined(USE_SX128X))

#define DEVICE_ADDR (UID[5] & 0b111111)
#define CRC_CIPHER UID[4]

#define RC_DATA_PACKET 0b00
#define MSP_DATA_PACKET 0b01
#define SYNC_PACKET 0b10
#define TLM_PACKET 0b11

#define DEBUG_PIN PIN_A10

// Desired buffer time between Packet ISR and Tock ISR
#define PACKET_TO_TOCK_SLACK 200

extern uint8_t fhss_index;
extern int32_t fhss_update_freq_correction(uint8_t value);
extern void fhss_randomize(int32_t seed);
extern uint32_t fhss_get_freq(uint16_t index);
extern uint32_t fhss_next_freq();
extern void fhss_reset();

extern void crc14_init();
extern uint16_t crc14_calc(uint8_t *data, uint8_t len, uint16_t crc);

extern expresslrs_mod_settings_t *current_air_rate_config();
extern expresslrs_rf_pref_params_t *current_rf_pref_params();

float rx_rssi;

static volatile elrs_state_t elrs_state = DISCONNECTED;
static volatile bool already_hop = false;
static volatile bool needs_hop = false;

volatile uint8_t packet[ELRS_BUFFER_SIZE];

static uint32_t sync_packet_time = 0;

static uint32_t next_rate = 0;

static uint8_t nonce_rx = 0;
static uint8_t UID[6] = {221, 251, 226, 34, 222, 25};
// static uint8_t UID[6] = {0, 1, 2, 3, 4, 5};

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

  const uint8_t tlm_mod = ((nonce_rx + 1) % current_air_rate_config()->tlm_interval);
  if (tlm_mod != 0 || current_air_rate_config()->tlm_interval == 0) {
    elrs_enter_rx(packet);
  }
  return true;
}

static void elrs_tlm() {
  if (current_air_rate_config()->tlm_interval == 0) {
    return;
  }

  const uint8_t modresult = (nonce_rx + 1) % current_air_rate_config()->tlm_interval;
  if (modresult != 0) {
    return;
  }

  packet[0] = (DEVICE_ADDR << 2) + 0b11;
  packet[1] = 0x14;
  packet[2] = 88;
  packet[3] = 0;
  packet[4] = 0;
  packet[5] = 0;
  packet[6] = 0;
  packet[7] = 0; // TODO: CRC
}

static bool elrs_vaild_packet() {
  if (!elrs_read_packet(packet)) {
    return false;
  }

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
  elrs_timer_stop();

  already_hop = false;

  fhss_reset();
  elrs_phase_reset();

  elrs_set_rate(next_rate, fhss_get_freq(0), UID[5] & 0x01);
  elrs_enter_rx(packet);
}

static void elrs_connection_tentative() {
  elrs_state = TENTATIVE;

  fhss_reset();
  elrs_phase_reset();

  elrs_timer_resume(current_air_rate_config()->interval);
}

// this is 180 out of phase with the other callback, occurs mid-packet reception
void elrs_handle_tick() {
  elrs_phase_update(elrs_state);
  nonce_rx++;
  already_hop = false;
}

void elrs_handle_tock() {
  elrs_phase_int_event(time_micros());
  needs_hop = true;
}

void elrs_process_packet(uint32_t packet_time) {
  elrs_phase_ext_event(packet_time + PACKET_TO_TOCK_SLACK);

  const uint8_t type = packet[0] & 0b11;
  switch (type) {
  case SYNC_PACKET: {
    if (packet[4] != UID[3] || packet[5] != UID[4] || packet[6] != UID[5]) {
      break;
    }

    sync_packet_time = time_millis();

    const uint8_t rate_index = ((packet[3] & 0b11000000) >> 6);
    const uint8_t telemetry_rate_index = ((packet[3] & 0b00111000) >> 3);
    //const uint8_t switch_mode = ((packet[3] & 0b00000110) >> 1);

    if (rate_index != current_air_rate_config()->index) {
      next_rate = rate_index;
      elrs_connection_lost();
    }

    if (current_air_rate_config()->tlm_interval != tlm_ration_map[telemetry_rate_index]) {
      current_air_rate_config()->tlm_interval = tlm_ration_map[telemetry_rate_index];
    }

    if (elrs_state == DISCONNECTED ||
        (nonce_rx != packet[2]) ||
        (fhss_index != packet[1])) {
      fhss_index = packet[1];
      nonce_rx = packet[2];

      elrs_connection_tentative();
    } else {
      quic_debugf("sync???");
    }
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

  elrs_set_rate(next_rate, fhss_get_freq(0), (UID[5] & 0x01));
  elrs_timer_init(current_air_rate_config()->interval);
  elrs_enter_rx(packet);
}

void rx_check() {
  const uint32_t packet_time = time_micros();
  if (elrs_vaild_packet()) {
    elrs_process_packet(packet_time);
  }

  if (needs_hop) {
    needs_hop = false;

    const bool did_hop = elrs_hop();
    if (!did_hop) {
      elrs_freq_correct();
    }
  }

  if (elrs_state == TENTATIVE && ((time_millis() - sync_packet_time) > current_rf_pref_params()->rf_mode_cycle_addtional_time)) {
    sync_packet_time = time_millis();
    elrs_connection_lost();
    return;
  }
}

#endif