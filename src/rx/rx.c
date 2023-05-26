#include "rx/rx.h"

#include <math.h>

#include "core/flash.h"
#include "core/profile.h"
#include "core/project.h"
#include "driver/serial.h"
#include "driver/time.h"
#include "flight/control.h"
#include "flight/filter.h"
#include "util/util.h"

extern profile_t profile;

uint8_t failsafe_siglost = 0;

uint32_t last_frame_time_us = 0;
static uint32_t frame_missed_time_us = 0;
static uint32_t frames_per_second = 0;
static uint32_t frames_missed = 0;
static uint32_t frames_received = 0;

static filter_lp_pt1 rx_filter;
static filter_state_t rx_filter_state[4];

// Select filter cut, Formula is [(1/rx framerate)/2] * 0.9
// 0 will trigger selection via rx_smoothing_cutoff
static const uint16_t RX_SMOOTHING_HZ[RX_PROTOCOL_MAX] = {
    0,   // RX_PROTOCOL_INVALID, wont happen
    0,   // RX_PROTOCOL_UNIFIED_SERIAL, will autodetect following
    25,  // RX_PROTOCOL_SBUS,
    67,  // RX_PROTOCOL_CRSF,
    50,  // RX_PROTOCOL_IBUS, check these
    50,  // RX_PROTOCOL_FPORT, check these
    0,   // RX_PROTOCOL_DSM,
    90,  // RX_PROTOCOL_NRF24_BAYANG_TELEMETRY,
    90,  // RX_PROTOCOL_BAYANG_PROTOCOL_BLE_BEACON,
    90,  // RX_PROTOCOL_BAYANG_PROTOCOL_TELEMETRY_AUTOBIND,
    50,  // RX_PROTOCOL_FRSKY_D8,
    50,  // RX_PROTOCOL_FRSKY_D16_FCC,
    50,  // RX_PROTOCOL_FRSKY_D16_LBT,
    225, // RX_PROTOCOL_FRSKY_REDPINE,
    0,   // RX_PROTOCOL_EXPRESS_LRS
    225, // RX_PROTOCOL_FLYSKY_AFHDS    225 = (666pps * 0.75)/2 * 0.9   Note: mul by 0.75 for expected packet loss
    88   // RX_PROTOCOL_FLYSKY_AFHDS2A  88 = (260pps * 0.75)/2 * 0.9
};

static const uint16_t SERIAL_PROTO_MAP[] = {
    RX_PROTOCOL_INVALID, // RX_SERIAL_PROTOCOL_INVALID
    RX_PROTOCOL_DSM,     // RX_SERIAL_PROTOCOL_DSM
    RX_PROTOCOL_SBUS,    // RX_SERIAL_PROTOCOL_SBUS
    RX_PROTOCOL_IBUS,    // RX_SERIAL_PROTOCOL_IBUS
    RX_PROTOCOL_FPORT,   // RX_SERIAL_PROTOCOL_FPORT
    RX_PROTOCOL_CRSF,    // RX_SERIAL_PROTOCOL_CRSF
    RX_PROTOCOL_REDPINE, // RX_SERIAL_PROTOCOL_REDPINE
    // No need to filter differently for inverted.
    RX_PROTOCOL_SBUS,    // RX_SERIAL_PROTOCOL_SBUS_INVERTED
    RX_PROTOCOL_FPORT,   // RX_SERIAL_PROTOCOL_FPORT_INVERTED
    RX_PROTOCOL_REDPINE, // RX_SERIAL_PROTOCOL_REDPINE_INVERTED
};

uint16_t rx_serial_smoothing_cutoff() {
  const uint16_t serial_proto = SERIAL_PROTO_MAP[bind_storage.unified.protocol];
  if (serial_proto == RX_PROTOCOL_CRSF) {
    return rx_serial_crsf_smoothing_cutoff();
  }
  if (serial_proto == RX_PROTOCOL_DSM) {
    return rx_serial_dsm_smoothing_cutoff();
  }
  return RX_SMOOTHING_HZ[serial_proto];
}

float rx_smoothing_hz() {
  switch (profile.receiver.protocol) {
  case RX_PROTOCOL_UNIFIED_SERIAL:
    return rx_serial_smoothing_cutoff();

#ifdef RX_EXPRESS_LRS
  case RX_PROTOCOL_EXPRESS_LRS:
    return rx_expresslrs_smoothing_cutoff();
#endif

  default:
    return RX_SMOOTHING_HZ[profile.receiver.protocol];
  }
}

uint8_t rx_aux_on(aux_function_t function) {
  return state.aux[profile.receiver.aux[function]];
}

void rx_lqi_lost_packet() {
  frames_missed++;

  if (frame_missed_time_us == 0) {
    frame_missed_time_us = time_micros();
  }

  if (time_micros() - frame_missed_time_us > FAILSAFETIME) {
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

  state.rx_rssi = constrainf(state.rx_rssi, 0.f, 100.f);
}

void rx_lqi_update_direct(float rssi) {
  state.rx_rssi = constrainf(rssi, 0.f, 100.f);
}

static void rx_apply_smoothing() {
  filter_lp_pt1_coeff(&rx_filter, rx_smoothing_hz());

  for (int i = 0; i < 4; ++i) {
    if (i == 3) {
      // throttle is 0 - 1.0
      state.rx.axis[i] = constrainf(state.rx.axis[i], 0.0, 1.0);
    } else {
      // other channels are -1.0 - 1.0
      state.rx.axis[i] = constrainf(state.rx.axis[i], -1.0, 1.0);
    }
#ifdef RX_SMOOTHING
    state.rx_filtered.axis[i] = filter_lp_pt1_step(&rx_filter, &rx_filter_state[i], state.rx.axis[i]);
#endif
    if (i == 3) {
      // throttle is 0 - 1.0
      state.rx_filtered.axis[i] = constrainf(state.rx_filtered.axis[i], 0.0, 1.0);
    } else {
      // other channels are -1.0 - 1.0
      state.rx_filtered.axis[i] = constrainf(state.rx_filtered.axis[i], -1.0, 1.0);
    }
  }
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
#ifdef GESTURE_AUX_START_ON
  state.aux[AUX_CHANNEL_GESTURE] = 1;
#endif
  filter_lp_pt1_init(&rx_filter, rx_filter_state, 4, rx_smoothing_hz());
}

void rx_init() {
  rx_init_state();
  rx_spi_detect();

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
    rx_serial_init();
    break;

  case RX_PROTOCOL_NRF24_BAYANG_TELEMETRY:
  case RX_PROTOCOL_BAYANG_PROTOCOL_BLE_BEACON:
  case RX_PROTOCOL_BAYANG_PROTOCOL_TELEMETRY_AUTOBIND:
    break;

  case RX_PROTOCOL_FRSKY_D8:
#ifdef RX_FRSKY
    rx_frsky_d8_init();
#endif
    break;

  case RX_PROTOCOL_FRSKY_D16_FCC:
  case RX_PROTOCOL_FRSKY_D16_LBT:
#ifdef RX_FRSKY
    rx_frsky_d16_init();
#endif
    break;

  case RX_PROTOCOL_REDPINE:
#ifdef RX_FRSKY
    rx_redpine_init();
#endif
    break;

  case RX_PROTOCOL_EXPRESS_LRS:
#ifdef RX_EXPRESS_LRS
    rx_expresslrs_init();
#endif
    break;

  case RX_PROTOCOL_FLYSKY_AFHDS:
#ifdef RX_FLYSKY
    rx_flysky_afhds_init();
#endif
    break;
  case RX_PROTOCOL_FLYSKY_AFHDS2A:
#ifdef RX_FLYSKY
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
    return rx_serial_check();

  case RX_PROTOCOL_NRF24_BAYANG_TELEMETRY:
  case RX_PROTOCOL_BAYANG_PROTOCOL_BLE_BEACON:
  case RX_PROTOCOL_BAYANG_PROTOCOL_TELEMETRY_AUTOBIND:
    return false;

  case RX_PROTOCOL_FRSKY_D8:
#ifdef RX_FRSKY
    return rx_frsky_d8_check();
#else
    return false;
#endif

  case RX_PROTOCOL_FRSKY_D16_FCC:
  case RX_PROTOCOL_FRSKY_D16_LBT:
#ifdef RX_FRSKY
    return rx_frsky_d16_check();
#else
    return false;
#endif

  case RX_PROTOCOL_REDPINE:
#ifdef RX_FRSKY
    return rx_redpine_check();
#else
    return false;
#endif

  case RX_PROTOCOL_EXPRESS_LRS:
#ifdef RX_EXPRESS_LRS
    return rx_expresslrs_check();
#else
    return false;
#endif

  case RX_PROTOCOL_FLYSKY_AFHDS:
#ifdef RX_FLYSKY
    return rx_flysky_afhds_check();
#else
    return false;
#endif
  case RX_PROTOCOL_FLYSKY_AFHDS2A:
#ifdef RX_FLYSKY
    return rx_flysky_afhds2a_check();
#else
    return false;
#endif
  }

  return false;
}

void rx_update() {
  if (rx_check()) {
    rx_apply_stick_scale();

    state.rx.roll = rx_apply_deadband(state.rx.roll);
    state.rx.pitch = rx_apply_deadband(state.rx.pitch);
    state.rx.yaw = rx_apply_deadband(state.rx.yaw);
  }

  rx_apply_smoothing();
}
