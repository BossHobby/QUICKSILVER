#pragma once

#include <stdbool.h>

#include "rx/rx_spi.h"

#include "driver/spi_sx127x.h"
#include "driver/spi_sx128x.h"

#define ELRS_OTA_VERSION_ID 3

#define ELRS_BUFFER_SIZE 16
#define ELRS_RATE_DEFAULT 0

#ifdef USE_SX127X
#define ELRS_RATE_MAX 5
#define ERLS_RATE_BIND 2
#endif

#ifdef USE_SX128X
#define ELRS_RATE_MAX 10
#define ERLS_RATE_BIND 9
#define RADIO_SNR_SCALE 4 // Units for LastPacketSNRRaw
#endif

#define ELRS_TELEMETRY_SHIFT 2
#define ELRS_TELEMETRY_BYTES_PER_CALL 5
#define ELRS_TELEMETRY_MAX_PACKAGES (255 >> ELRS_TELEMETRY_SHIFT)
#define ELRS_TELEMETRY_MAX_MISSED_PACKETS 20

#define ELRS_MSP_BYTES_PER_CALL 5
#define ELRS_MSP_BUFFER_SIZE 65
#define ELRS_MSP_MAX_PACKAGES ((ELRS_MSP_BUFFER_SIZE / ELRS_MSP_BYTES_PER_CALL) + 1)

#define DYNPOWER_SNR_THRESH_NONE -127

#define OTA4_PACKET_SIZE 8U
#define OTA4_CRC_CALC_LEN 7U
#define OTA8_PACKET_SIZE 13U
#define OTA8_CRC_CALC_LEN 11U

#define SNR_SCALE(snr) ((int8_t)((float)snr * RADIO_SNR_SCALE))
#define SNR_DESCALE(snrScaled) (snrScaled / RADIO_SNR_SCALE)

typedef enum {
  DISCONNECTED,
  TENTATIVE,
  CONNECTED
} elrs_state_t;

typedef enum {
  TIMER_DISCONNECTED,
  TIMER_TENTATIVE,
  TIMER_LOCKED
} elrs_timer_state_t;

typedef enum {
  IRQ_NONE,
  IRQ_RX_DONE,
  IRQ_TX_DONE
} elrs_irq_status_t;

typedef enum {
  TLM_RATIO_STD = 0, // Use suggested ratio from ModParams
  TLM_RATIO_NO_TLM,
  TLM_RATIO_1_128,
  TLM_RATIO_1_64,
  TLM_RATIO_1_32,
  TLM_RATIO_1_16,
  TLM_RATIO_1_8,
  TLM_RATIO_1_4,
  TLM_RATIO_1_2,
  TLM_RATIO_DISARMED, // TLM_RATIO_STD when disarmed, TLM_RATIO_NO_TLM when armed
} expresslrs_tlm_ratio_t;

typedef enum {
  RATE_LORA_4HZ = 0,
  RATE_LORA_25HZ,
  RATE_LORA_50HZ,
  RATE_LORA_100HZ,
  RATE_LORA_100HZ_8CH,
  RATE_LORA_150HZ,
  RATE_LORA_200HZ,
  RATE_LORA_250HZ,
  RATE_LORA_333HZ_8CH,
  RATE_LORA_500HZ,
  RATE_DVDA_250HZ,
  RATE_DVDA_500HZ,
  RATE_FLRC_500HZ,
  RATE_FLRC_1000HZ,
} expresslrs_rf_rates_t;

typedef enum {
  RADIO_TYPE_SX127x_LORA,
  RADIO_TYPE_SX128x_LORA,
  RADIO_TYPE_SX128x_FLRC,
} expresslrs_radio_type_t;

typedef enum {
  SWITCH_WIDE_OR_8CH,
  SWITCH_HYBRID_OR_16CH,
} elrs_switch_mode_t;

typedef struct {
  uint8_t is_set;
  uint8_t uid[6];
  uint8_t magic;
  uint8_t switch_mode;
  uint8_t model_id;
} rx_elrs_bind_data_t;

typedef struct {
  int32_t beta;     // Length = 16
  int32_t fp_shift; // Number of fractional bits

  int32_t smooth_data_int;
  int32_t smooth_data_fp;
} elrs_lpf_t;

typedef struct {
  bool int_event_active;
  uint32_t int_event_time_us;

  bool ext_event_active;
  uint32_t ext_event_time_us;

  int32_t raw_offset_us;
  int32_t prev_raw_offset_us;

  int32_t offset;
  elrs_lpf_t offset_lpf;

  int32_t offset_dx;
  elrs_lpf_t offset_dx_lpf;

} elrs_phase_lock_state_t;

typedef struct {
  uint8_t index;
  expresslrs_radio_type_t radio_type;
  expresslrs_rf_rates_t rate;
  uint8_t bw;
  uint8_t sf;
  uint8_t cr;
  expresslrs_tlm_ratio_t tlm_interval; // every X packets is a response TLM packet, should be a power of 2
  uint8_t fhss_hop_interval;           // every X packets we hop to a new frequency. Max value of 16 since only 4 bits have been assigned in the sync package.
  uint32_t interval;                   // interval in us seconds that corresponds to that frequency
  uint8_t preamble_len;
  uint8_t payload_len;  // Number of OTA bytes to be sent.
  uint8_t num_of_sends; // Number of packets to send.
} expresslrs_mod_settings_t;

typedef struct {
  uint8_t index;
  expresslrs_rf_rates_t rate;
  int16_t rx_sensitivity;                  // expected min RF sensitivity
  uint16_t toa;                            // time on air in microseconds
  uint16_t disconnect_timeout_ms;          // Time without a packet before receiver goes to disconnected (ms)
  uint16_t rx_lock_timeout_ms;             // Max time to go from tentative -> connected state on receiver (ms)
  uint16_t sync_pkt_interval_disconnected; // how often to send the PACKET_TYPE_SYNC (ms) when there is no response from RX
  uint16_t sync_pkt_interval_connected;    // how often to send the PACKET_TYPE_SYNC (ms) when there we have a connection
  int8_t dynpower_snr_thresh_up;           // Request a raise in power if the reported (average) SNR is at or below this
                                           // or DYNPOWER_UPTHRESH_SNR_NONE to use RSSI
  int8_t dynpower_snr_thresh_dn;           // Like DynpowerSnrUpThreshold except to lower power

} expresslrs_rf_pref_params_t;

bool elrs_radio_init();

void elrs_set_frequency(int32_t freq);
void elrs_set_rate(uint8_t index, int32_t freq, bool invert_iq, uint32_t flrc_sync_word, uint16_t flrc_crc_seed);

void elrs_enter_rx(volatile uint8_t *packet);
void elrs_enter_tx(volatile uint8_t *packet, const uint8_t packet_len);

elrs_irq_status_t elrs_get_irq_status();
void elrs_read_packet(volatile uint8_t *packet);
void elrs_last_packet_stats(int8_t *rssi, int8_t *snr);

void elrs_freq_correct();

void elrs_timer_init(uint32_t interval_us);
void elrs_timer_resume(uint32_t interval_us);
void elrs_timer_stop();
bool elrs_timer_is_running();

void elrs_phase_init();
void elrs_phase_update(elrs_state_t state);
void elrs_phase_int_event(uint32_t time);
void elrs_phase_ext_event(uint32_t time);
void elrs_phase_reset();

void elrs_lpf_init(elrs_lpf_t *lpf, int32_t beta);
int32_t elrs_lpf_update(elrs_lpf_t *lpf, int32_t data);

void elrs_lq_add();
void elrs_lq_inc();
uint8_t elrs_lq_get();
uint8_t elrs_lq_get_raw();
bool elrs_lq_current_is_set();
void elrs_lq_reset();

void elrs_snr_mean_reset();
void elrs_snr_mean_add(const int8_t val);
int8_t elrs_snr_mean_get(const int8_t def);

void elrs_tlm_receiver_reset();
void elrs_tlm_receiver_set_max_package_index(uint8_t max_package_index);
bool elrs_tlm_receiver_confirm();
void elrs_tlm_receiver_set_data_to_receive(uint8_t *data_to_receive, uint8_t max_length);
void elrs_tlm_receiver_receive_data(uint8_t const package_index, uint8_t const *const receive_data, uint8_t data_len);
bool elrs_tlm_receiver_has_finished_data();
void elrs_tlm_receiver_unlock();

void elrs_tlm_sender_reset();
void elrs_tlm_sender_set_max_package_index(uint8_t max_package_index);
bool elrs_tlm_sender_active();
void elrs_tlm_sender_set_data(uint8_t *data_to_transmit, const uint8_t length_to_transmit);
uint8_t elrs_tlm_sender_current_payload(uint8_t *outData, uint8_t maxLen);
void elrs_tlm_sender_confirm_payload(bool telemetry_confirm_value);
void elrs_tlm_sender_update_rate(const uint16_t air_rate, const uint8_t tlm_ratio, const uint8_t tlm_burst);
