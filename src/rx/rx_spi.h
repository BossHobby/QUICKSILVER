#pragma once

#include "rx/rx.h"

#define RX_SPI_PACKET_SIZE 128

typedef enum {
  RX_SPI_STATUS_NONE = 0,
  RX_SPI_STATUS_BINDING,
  RX_SPI_STATUS_BOUND,
} rx_spi_status_t;

extern volatile uint8_t rx_spi_packet[RX_SPI_PACKET_SIZE];

void rx_redpine_init();
void rx_frsky_d8_init();
void rx_frsky_d16_init();
void rx_expresslrs_init();
void rx_flysky_afhds_init();
void rx_flysky_afhds2a_init();

bool rx_redpine_check();
bool rx_frsky_d8_check();
bool rx_frsky_d16_check();
bool rx_expresslrs_check();
bool rx_flysky_afhds_check();
bool rx_flysky_afhds2a_check();

void rx_expresslrs_stop();

uint16_t rx_expresslrs_smoothing_cutoff();

void rx_spi_detect();