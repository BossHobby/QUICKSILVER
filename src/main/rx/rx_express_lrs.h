#pragma once

#include <stdbool.h>

#include "rx.h"

#include "drv_spi_sx127x.h"
#include "drv_spi_sx128x.h"

#define ELRS_BUFFER_SIZE 8

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
} expresslrs_mod_settings_t;
#endif

typedef struct {
  int8_t index;
  expresslrs_rf_rates_t rate; // Max value of 16 since only 4 bits have been assigned in the sync package.
  int32_t rx_sensitivity;     // expected RF sensitivity based on
  uint32_t toa;               // time on air in microseconds
  uint32_t rf_mode_cycle_interval;
  uint32_t rf_mode_cycle_addtional_time;
  uint32_t sync_pkt_interval_disconnected;
  uint32_t sync_pkt_interval_connected;
} expresslrs_rf_pref_params_t;

bool elrs_radio_init();

void elrs_set_frequency(int32_t freq);
void elrs_set_rate(uint8_t index, int32_t freq, bool invert_iq);

void elrs_enter_rx(uint8_t *packet);
void elrs_enter_tx(uint8_t *packet);

elrs_irq_status_t elrs_get_irq_status();
void elrs_read_packet(uint8_t *packet);

void elrs_freq_correct();

void elrs_timer_init(uint32_t interval_us);
void elrs_timer_resume(uint32_t interval_us);
void elrs_timer_stop();

void elrs_phase_init();
void elrs_phase_update(elrs_state_t state);
void elrs_phase_int_event(uint32_t time);
void elrs_phase_ext_event(uint32_t time);
void elrs_phase_reset();

void elrs_lpf_init(elrs_lpf_t *lpf, int32_t beta);
int32_t elrs_lpf_update(elrs_lpf_t *lpf, int32_t data);