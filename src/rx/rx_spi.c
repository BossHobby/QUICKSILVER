#include "rx/rx_spi.h"

#include "core/profile.h"
#include "core/project.h"
#include "rx/express_lrs.h"
#include "rx/flysky.h"
#include "rx/frsky.h"

volatile uint8_t rx_spi_packet[128];

#ifndef USE_RX_SPI_EXPRESS_LRS
bool elrs_radio_init() {
  return false;
}
#endif

#ifndef USE_RX_SPI_FRSKY
bool frsky_init() {
  return false;
}
#endif

#ifndef USE_RX_SPI_FLYSKY
bool flysky_detect() {
  return false;
}
#endif

void rx_spi_detect() {
  static bool did_detect = false;
  if (did_detect) {
    return;
  }

  if (elrs_radio_init()) {
    target_add_rx_protocol(RX_PROTOCOL_EXPRESS_LRS);
  } else if (frsky_init()) {
    target_add_rx_protocol(RX_PROTOCOL_FRSKY_D8);
    target_add_rx_protocol(RX_PROTOCOL_FRSKY_D16_FCC);
    target_add_rx_protocol(RX_PROTOCOL_FRSKY_D16_LBT);
    target_add_rx_protocol(RX_PROTOCOL_REDPINE);
  } else if (flysky_detect()) {
    target_add_rx_protocol(RX_PROTOCOL_FLYSKY_AFHDS);
    target_add_rx_protocol(RX_PROTOCOL_FLYSKY_AFHDS2A);
  }

  did_detect = true;
}

void rx_spi_handle_busy_exti(bool level) {
  switch (profile.receiver.protocol) {
#ifdef USE_RX_SPI_EXPRESS_LRS
  case RX_PROTOCOL_EXPRESS_LRS: {
    extern void sx128x_handle_busy_exti(bool);
    sx128x_handle_busy_exti(level);
    break;
  }
#endif
  default:
    break;
  }
}

void rx_spi_handle_exti(bool level) {
  switch (profile.receiver.protocol) {
#ifdef USE_RX_SPI_EXPRESS_LRS
  case RX_PROTOCOL_EXPRESS_LRS: {
    extern void sx128x_handle_dio0_exti(bool);
    sx128x_handle_dio0_exti(level);
    break;
  }
#endif
#ifdef USE_RX_SPI_FLYSKY
  case RX_PROTOCOL_FLYSKY_AFHDS:
  case RX_PROTOCOL_FLYSKY_AFHDS2A: {
    extern void a7105_handle_exti(bool);
    a7105_handle_exti(level);
    break;
  }
#endif
  default:
    break;
  }
}