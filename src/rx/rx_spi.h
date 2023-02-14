#pragma once

#include "rx/rx.h"

typedef enum {
  RX_SPI_STATUS_NONE = 0,
  RX_SPI_STATUS_BINDING,
  RX_SPI_STATUS_BOUND,
} rx_spi_status_t;

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

uint16_t rx_expresslrs_smoothing_cutoff();