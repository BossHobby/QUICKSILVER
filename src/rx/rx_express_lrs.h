#pragma once

#include <stdbool.h>

#include "rx_spi.h"

#include "drv_spi_sx127x.h"
#include "drv_spi_sx128x.h"

#define ELRS_BUFFER_SIZE 8
#define ELRS_RATE_MAX 4
#define ELRS_RATE_DEFAULT 0

#ifdef USE_SX127X
#define ERLS_RATE_BIND 2
#endif

#ifdef USE_SX128X
#define ERLS_RATE_BIND 3
#endif

#define ELRS_TELEMETRY_SHIFT 2
#define ELRS_TELEMETRY_BYTES_PER_CALL 5
#define ELRS_TELEMETRY_MAX_PACKAGES (255 >> ELRS_TELEMETRY_SHIFT)
#define ELRS_TELEMETRY_MAX_MISSED_PACKETS 20

#define ELRS_MSP_BYTES_PER_CALL 5
#define ELRS_MSP_BUFFER_SIZE 65
#define ELRS_MSP_MAX_PACKAGES ((ELRS_MSP_BUFFER_SIZE / ELRS_MSP_BYTES_PER_CALL) + 1)

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
  TLM_RATIO_NO_TLM = 0,
  TLM_RATIO_1_128 = 1,
  TLM_RATIO_1_64 = 2,
  TLM_RATIO_1_32 = 3,
  TLM_RATIO_1_16 = 4,
  TLM_RATIO_1_8 = 5,
  TLM_RATIO_1_4 = 6,
  TLM_RATIO_1_2 = 7,
} expresslrs_tlm_ratio_t;

typedef enum {
  RATE_500HZ = 0,
  RATE_250HZ = 1,
  RATE_200HZ = 2,
  RATE_150HZ = 3,
  RATE_100HZ = 4,
  RATE_50HZ = 5,
  RATE_25HZ = 6,
  RATE_4HZ = 7
} expresslrs_rf_rates_t;

typedef enum {
  SWITCH_1BIT,
  SWITCH_HYBRID,
  SWITCH_HYBRID_WIDE,
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

#ifdef USE_SX127X
typedef struct {
  int8_t index;
  expresslrs_rf_rates_t rate; // Max value of 16 since only 4 bits have been assigned in the sync package.
  sx127x_bandwidth_t bw;
  sx127x_spreading_factor_t sf;
  sx127x_coding_rate_t cr;
  uint32_t interval;                   // interval in us seconds that corresponds to that frequnecy
  expresslrs_tlm_ratio_t tlm_interval; // every X packets is a response TLM packet, should be a power of 2
  uint8_t fhss_hop_interval;           // every X packets we hope to a new frequnecy. Max value of 16 since only 4 bits have been assigned in the sync package.
  uint8_t preamble_len;
  uint8_t payload_len; // Number of OTA bytes to be sent.
} expresslrs_mod_settings_t;
#endif

#ifdef USE_SX128X
typedef struct {
  int8_t index;
  expresslrs_rf_rates_t rate; // Max value of 16 since only 4 bits have been assigned in the sync package.
  sx128x_lora_bandwidths_t bw;
  sx128x_lora_spreading_factors_t sf;
  sx128x_lora_coding_rates_t cr;
  uint32_t interval;                   // interval in us seconds that corresponds to that frequency
  expresslrs_tlm_ratio_t tlm_interval; // every X packets is a response TLM packet, should be a power of 2
  uint8_t fhss_hop_interval;           // every X packets we hop to a new frequency. Max value of 16 since only 4 bits have been assigned in the sync package.
  uint8_t preamble_len;
  uint8_t payload_len; // Number of OTA bytes to be sent.
} expresslrs_mod_settings_t;
#endif

typedef struct {
  int8_t index;
  expresslrs_rf_rates_t rate;              // Max value of 16 since only 4 bits have been assigned in the sync package.
  int32_t rx_sensitivity;                  // expected RF sensitivity based on
  uint32_t toa;                            // time on air in microseconds
  uint32_t disconnect_timeout_ms;          // Time without a packet before receiver goes to disconnected (ms)
  uint32_t rx_lock_timeout_ms;             // Max time to go from tentative -> connected state on receiver (ms)
  uint32_t sync_pkt_interval_disconnected; // how often to send the SYNC_PACKET packet (ms) when there is no response from RX
  uint32_t sync_pkt_interval_connected;    // how often to send the SYNC_PACKET packet (ms) when there we have a connection
} expresslrs_rf_pref_params_t;

bool elrs_radio_init();

void elrs_set_frequency(int32_t freq);
void elrs_set_rate(uint8_t index, int32_t freq, bool invert_iq);

void elrs_enter_rx(volatile uint8_t *packet);
void elrs_enter_tx(volatile uint8_t *packet);

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
bool elrs_lq_current_is_set();
void elrs_lq_reset();

bool elrs_get_msp_confirm();
void elrs_setup_msp(const uint8_t max_length, uint8_t *buffer, const uint8_t bytes_per_call);
void elrs_receive_msp(const uint8_t package_index, const volatile uint8_t *data);
bool elrs_msp_finished_data();
void elrs_msp_restart();

void elrs_tlm_sender_reset();
bool elrs_tlm_sender_active();
void elrs_tlm_sender_set_data(const uint8_t bpc, uint8_t *data, const uint8_t length);
void elrs_tlm_current_payload(uint8_t *package_index, uint8_t *count, uint8_t **data);
void elrs_tlm_confirm_payload(const bool confirm_value);
void elrs_tlm_update_rate(const uint16_t air_rate, const uint8_t tlm_ratio, const uint8_t tlm_burst);
