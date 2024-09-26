#include "rx/rx.h"

#include <math.h>

#include "core/flash.h"
#include "core/profile.h"
#include "core/project.h"
#include "driver/serial.h"
#include "driver/time.h"
#include "flight/control.h"
#include "flight/filter.h"
#include "io/simulator.h"
#include "util/util.h"

#define RX_FITER_SAMPLE_TIME (5000)

extern profile_t profile;

uint8_t failsafe_siglost = 0;

uint32_t last_frame_time_us = 0;
static uint32_t frame_missed_time_us = 0;
static uint32_t frames_per_second = 0;
static uint32_t frames_missed = 0;
static uint32_t frames_received = 0;

static filter_lp_pt2 rx_filter;
static filter_state_t rx_filter_state[4];

void rx_lqi_lost_packet() {
  frames_missed++;

  if (frame_missed_time_us == 0) {
    frame_missed_time_us = time_micros();
  }

  if (time_micros() - frame_missed_time_us > FAILSAFE_TIME_US) {
    failsafe_siglost = 1;
  }
}

void rx_lqi_got_packet() {
  frames_received++;
  last_frame_time_us = time_micros();

  frame_missed_time_us = 0;
  failsafe_siglost = 0;
}

void rx_lqi_update() {
  const uint32_t time = time_micros();

  // link quality & rssi
  static uint32_t last_time = 0;
  if (time - last_time < 1000000) {
    // we only run once per second
    return;
  }

  frames_per_second = frames_received;

  frames_received = 0;
  frames_missed = 0;

  last_time = time;
}

void rx_lqi_update_from_fps(float expected_fps) {
  state.rx_rssi = frames_per_second / expected_fps;
  state.rx_rssi = state.rx_rssi * state.rx_rssi * state.rx_rssi * LQ_EXPO + state.rx_rssi * (1 - LQ_EXPO);
  state.rx_rssi *= 100.0f;

  state.rx_rssi = constrain(state.rx_rssi, 0.f, 100.f);
}

void rx_lqi_update_direct(float rssi) {
  state.rx_rssi = constrain(rssi, 0.f, 100.f);
}

static void rx_apply_smoothing() {
  if (state.rx_filter_hz <= 0.1f) {
    state.rx_filtered.roll = state.rx.roll = constrain(state.rx.roll, -1.0, 1.0);
    state.rx_filtered.pitch = state.rx.pitch = constrain(state.rx.pitch, -1.0, 1.0);
    state.rx_filtered.yaw = state.rx.yaw = constrain(state.rx.yaw, -1.0, 1.0);
    state.rx_filtered.throttle = state.rx.throttle = constrain(state.rx.throttle, 0.0, 1.0);
    return;
  }

  filter_lp_pt2_coeff(&rx_filter, state.rx_filter_hz);

  state.rx.roll = constrain(state.rx.roll, -1.0, 1.0);
  state.rx.pitch = constrain(state.rx.pitch, -1.0, 1.0);
  state.rx.yaw = constrain(state.rx.yaw, -1.0, 1.0);
  state.rx.throttle = constrain(state.rx.throttle, 0.0, 1.0);

  state.rx_filtered.roll = constrain(filter_lp_pt2_step(&rx_filter, &rx_filter_state[0], state.rx.roll), -1.0, 1.0);
  state.rx_filtered.pitch = constrain(filter_lp_pt2_step(&rx_filter, &rx_filter_state[1], state.rx.pitch), -1.0, 1.0);
  state.rx_filtered.yaw = constrain(filter_lp_pt2_step(&rx_filter, &rx_filter_state[2], state.rx.yaw), -1.0, 1.0);
  state.rx_filtered.throttle = constrain(filter_lp_pt2_step(&rx_filter, &rx_filter_state[3], state.rx.throttle), 0.0, 1.0);
}

static float rx_apply_deadband(float val) {
  if (profile.rate.sticks_deadband <= 0.0f) {
    return val;
  }

  if (fabsf(val) <= profile.rate.sticks_deadband) {
    return 0.0f;
  }

  if (val >= 0) {
    return mapf(val, profile.rate.sticks_deadband, 1, 0, 1);
  } else {
    return mapf(val, -profile.rate.sticks_deadband, -1, 0, -1);
  }
}

static void rx_init_state() {
  for (uint32_t i = 0; i < AUX_CHANNEL_OFF; i++) {
    state.aux[i] = 0;
  }

  // set always on channel to on
  state.aux[AUX_CHANNEL_ON] = 1;
  state.aux[AUX_CHANNEL_OFF] = 0;

  filter_lp_pt2_init(&rx_filter, rx_filter_state, 4, state.rx_filter_hz);
}

void rx_init() {
  rx_init_state();
  rx_spi_detect();

  serial_rx_detected_protcol = RX_SERIAL_PROTOCOL_INVALID;
  if (!target_has_rx_protocol(profile.receiver.protocol)) {
    profile.receiver.protocol = RX_PROTOCOL_INVALID;
  }

  switch (profile.receiver.protocol) {
  case RX_PROTOCOL_INVALID:
  case RX_PROTOCOL_MAX:
    break;

  case RX_PROTOCOL_UNIFIED_SERIAL:
  case RX_PROTOCOL_SBUS:
  case RX_PROTOCOL_CRSF:
  case RX_PROTOCOL_IBUS:
  case RX_PROTOCOL_FPORT:
  case RX_PROTOCOL_DSM:
#ifdef USE_RX_UNIFIED
    rx_serial_init();
#endif
    break;

  case RX_PROTOCOL_NRF24_BAYANG_TELEMETRY:
  case RX_PROTOCOL_BAYANG_PROTOCOL_BLE_BEACON:
  case RX_PROTOCOL_BAYANG_PROTOCOL_TELEMETRY_AUTOBIND:
    break;

  case RX_PROTOCOL_FRSKY_D8:
#ifdef USE_RX_SPI_FRSKY
    rx_frsky_d8_init();
#endif
    break;

  case RX_PROTOCOL_FRSKY_D16_FCC:
  case RX_PROTOCOL_FRSKY_D16_LBT:
#ifdef USE_RX_SPI_FRSKY
    rx_frsky_d16_init();
#endif
    break;

  case RX_PROTOCOL_REDPINE:
#ifdef USE_RX_SPI_FRSKY
    rx_redpine_init();
#endif
    break;

  case RX_PROTOCOL_EXPRESS_LRS:
#ifdef USE_RX_SPI_EXPRESS_LRS
    rx_expresslrs_init();
#endif
    break;

  case RX_PROTOCOL_FLYSKY_AFHDS:
#ifdef USE_RX_SPI_FLYSKY
    rx_flysky_afhds_init();
#endif
    break;
  case RX_PROTOCOL_FLYSKY_AFHDS2A:
#ifdef USE_RX_SPI_FLYSKY
    rx_flysky_afhds2a_init();
#endif
    break;
  }
}

void rx_map_channels(const float channels[4]) {
  switch (profile.receiver.channel_mapping) {
  case RX_MAPPING_AETR:
    state.rx.roll = channels[0];
    state.rx.pitch = channels[1];
    state.rx.throttle = (channels[2] + 1.0f) * 0.5f;
    state.rx.yaw = channels[3];
    break;

  case RX_MAPPING_TAER:
    state.rx.throttle = (channels[0] + 1.0f) * 0.5f;
    state.rx.roll = channels[1];
    state.rx.pitch = channels[2];
    state.rx.yaw = channels[3];
    break;
  }
}

bool rx_check() {
#ifdef SIMULATOR
  return simulator_rx_check();
#else
  switch (profile.receiver.protocol) {
  case RX_PROTOCOL_INVALID:
  case RX_PROTOCOL_MAX:
    return false;

  case RX_PROTOCOL_UNIFIED_SERIAL:
  case RX_PROTOCOL_SBUS:
  case RX_PROTOCOL_CRSF:
  case RX_PROTOCOL_IBUS:
  case RX_PROTOCOL_FPORT:
  case RX_PROTOCOL_DSM:
#ifdef USE_RX_UNIFIED
    return rx_serial_check();
#else
    return false;
#endif

  case RX_PROTOCOL_NRF24_BAYANG_TELEMETRY:
  case RX_PROTOCOL_BAYANG_PROTOCOL_BLE_BEACON:
  case RX_PROTOCOL_BAYANG_PROTOCOL_TELEMETRY_AUTOBIND:
    return false;

  case RX_PROTOCOL_FRSKY_D8:
#ifdef USE_RX_SPI_FRSKY
    return rx_frsky_d8_check();
#else
    return false;
#endif

  case RX_PROTOCOL_FRSKY_D16_FCC:
  case RX_PROTOCOL_FRSKY_D16_LBT:
#ifdef USE_RX_SPI_FRSKY
    return rx_frsky_d16_check();
#else
    return false;
#endif

  case RX_PROTOCOL_REDPINE:
#ifdef USE_RX_SPI_FRSKY
    return rx_redpine_check();
#else
    return false;
#endif

  case RX_PROTOCOL_EXPRESS_LRS:
#ifdef USE_RX_SPI_EXPRESS_LRS
    return rx_expresslrs_check();
#else
    return false;
#endif

  case RX_PROTOCOL_FLYSKY_AFHDS:
#ifdef USE_RX_SPI_FLYSKY
    return rx_flysky_afhds_check();
#else
    return false;
#endif
  case RX_PROTOCOL_FLYSKY_AFHDS2A:
#ifdef USE_RX_SPI_FLYSKY
    return rx_flysky_afhds2a_check();
#else
    return false;
#endif
  }

  return false;
#endif
}

void rx_update() {
  static uint32_t rx_filter_start = 0;
  static uint32_t rx_filter_counter = 0;

  if (rx_check()) {
    rx_apply_stick_scale();

    state.rx.roll = rx_apply_deadband(state.rx.roll);
    state.rx.pitch = rx_apply_deadband(state.rx.pitch);
    state.rx.yaw = rx_apply_deadband(state.rx.yaw);

    rx_filter_counter += 1;
  }

  const uint32_t rx_filter_delta = (time_millis() - rx_filter_start);
  if (rx_filter_delta > RX_FITER_SAMPLE_TIME) {
    const float sample_hz = (float)rx_filter_counter / ((float)rx_filter_delta / 1000.0f);

    state.rx_filter_hz = rintf(sample_hz * 0.45f);
    rx_filter_start = time_millis();
    rx_filter_counter = 0;
  }

  rx_apply_smoothing();
}

void rx_stop() {
  switch (profile.receiver.protocol) {
#ifdef USE_RX_SPI_EXPRESS_LRS
  case RX_PROTOCOL_EXPRESS_LRS:
    rx_expresslrs_stop();
    break;
#endif

  default:
    break;
  }
}