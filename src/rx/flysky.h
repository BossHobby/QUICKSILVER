#pragma once

#include "rx/rx.h"

// Bind data saved into flash memory, see rx_bind_storage_t in flash.h
typedef struct {
  uint8_t rx_channel_map[16];
  uint32_t tx_id;
} rx_flysky_bind_data_t;

// State information for managing Flysky (afhds/afhds2a) protocol
typedef struct {
  rx_protocol_t protocol;
  float expected_fps;

  uint8_t bound : 1;

  uint32_t last_bind_time;
  uint32_t last_rx_time;

  uint32_t rx_id;
  uint32_t tx_id;
  uint8_t rx_channel_map[16]; // table of indices to 16 different frequencies we will hop within
  uint8_t channel_index;

  uint16_t channel_data[16]; // only 8 or 14 channels are actually used/available

  uint8_t send_telemetry : 1;
  uint8_t pending_tx : 1;
  uint32_t pending_tx_time;
  uint32_t last_telemetry_time;

  uint32_t tlm_period;
  uint32_t pkt_period;
  uint32_t timeout_period;

  uint32_t timeout;
  uint32_t last_pkt_time;
  uint32_t processed_pkt_count;
  uint32_t num_timeouts;

  uint32_t last_tlm_lqi_time;
  uint8_t tlm_lqi;
} rx_flsky_data_t;

extern rx_flsky_data_t flysky;

bool flysky_detect();
rx_flysky_bind_data_t *flysky_get_bind_data();
uint8_t flysky_get_next_channel(uint8_t step);
void flysky_processed_pkt(uint32_t timestamp);

uint8_t flysky_afhds_process_packet(const uint32_t timestamp);
uint8_t flysky_afhds2a_process_packet(const uint32_t timestamp);
