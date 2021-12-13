#pragma once

#include "rx.h"

typedef enum {
  RX_SPI_STATUS_NONE = 0,
  RX_SPI_STATUS_BINDING,
  RX_SPI_STATUS_BOUND,
} rx_spi_status_t;