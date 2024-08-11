#include "io/blackbox.h"

#include "driver/time.h"
#include "flight/control.h"
#include "io/blackbox_device.h"
#include "io/usb_configurator.h"
#include "util/cbor_helper.h"
#include "util/util.h"

#ifdef USE_BLACKBOX

static blackbox_t blackbox;
static uint8_t blackbox_enabled = 0;
static uint8_t blackbox_rate = 0;

cbor_result_t cbor_encode_blackbox_t(cbor_value_t *enc, const blackbox_t *b, const uint32_t field_flags) {
  CBOR_CHECK_ERROR(cbor_result_t res = cbor_encode_array_indefinite(enc));

  // loop and time are always emitted regardless of what field_flags indicates
  CBOR_CHECK_ERROR(res = cbor_encode_uint32_t(enc, &b->loop));
  CBOR_CHECK_ERROR(res = cbor_encode_uint32_t(enc, &b->time));

  if (field_flags & (1 << BBOX_FIELD_PID_P_TERM)) {
    CBOR_CHECK_ERROR(res = cbor_encode_compact_vec3_t(enc, &b->pid_p_term));
  }
  if (field_flags & (1 << BBOX_FIELD_PID_I_TERM)) {
    CBOR_CHECK_ERROR(res = cbor_encode_compact_vec3_t(enc, &b->pid_i_term));
  }
  if (field_flags & (1 << BBOX_FIELD_PID_D_TERM)) {
    CBOR_CHECK_ERROR(res = cbor_encode_compact_vec3_t(enc, &b->pid_d_term));
  }

  if (field_flags & (1 << BBOX_FIELD_RX)) {
    CBOR_CHECK_ERROR(res = cbor_encode_compact_vec4_t(enc, &b->rx));
  }

  if (field_flags & (1 << BBOX_FIELD_SETPOINT)) {
    CBOR_CHECK_ERROR(res = cbor_encode_compact_vec4_t(enc, &b->setpoint));
  }

  if (field_flags & (1 << BBOX_FIELD_ACCEL_RAW)) {
    CBOR_CHECK_ERROR(res = cbor_encode_compact_vec3_t(enc, &b->accel_raw));
  }

  if (field_flags & (1 << BBOX_FIELD_ACCEL_FILTER)) {
    CBOR_CHECK_ERROR(res = cbor_encode_compact_vec3_t(enc, &b->accel_filter));
  }

  if (field_flags & (1 << BBOX_FIELD_GYRO_RAW)) {
    CBOR_CHECK_ERROR(res = cbor_encode_compact_vec3_t(enc, &b->gyro_raw));
  }

  if (field_flags & (1 << BBOX_FIELD_GYRO_FILTER)) {
    CBOR_CHECK_ERROR(res = cbor_encode_compact_vec3_t(enc, &b->gyro_filter));
  }

  if (field_flags & (1 << BBOX_FIELD_MOTOR)) {
    CBOR_CHECK_ERROR(res = cbor_encode_compact_vec4_t(enc, &b->motor));
  }

  if (field_flags & (1 << BBOX_FIELD_CPU_LOAD)) {
    CBOR_CHECK_ERROR(res = cbor_encode_uint16_t(enc, &b->cpu_load));
  }

  if (field_flags & (1 << BBOX_FIELD_DEBUG)) {
    CBOR_CHECK_ERROR(res = cbor_encode_array(enc, BLACKBOX_DEBUG_SIZE));
    for (uint32_t i = 0; i < BLACKBOX_DEBUG_SIZE; i++) {
      CBOR_CHECK_ERROR(res = cbor_encode_int16_t(enc, &b->debug[i]));
    }
  }

  CBOR_CHECK_ERROR(res = cbor_encode_end_indefinite(enc));

  return res;
}

void blackbox_init() {
  blackbox_device_init();
}

void blackbox_set_debug(blackbox_debug_flag_t flag, uint8_t index, int16_t data) {
  if (index >= BLACKBOX_DEBUG_SIZE) {
    return;
  }
  if ((profile.blackbox.field_flags & flag) != flag) {
    return;
  }

  blackbox.debug[index] = data;
}

static uint32_t blackbox_rate_div() {
  return (1000000 / state.looptime_autodetect) / profile.blackbox.sample_rate_hz;
}

void blackbox_update() {
  if (!blackbox_device_update()) {
    // flash is still detecting, dont do anything
    return;
  }

  // flash is either idle or writing, do blackbox
  if ((!flags.arm_state || !rx_aux_on(AUX_BLACKBOX)) && blackbox_enabled == 1) {
    blackbox_device_finish();
    blackbox_enabled = 0;
    return;
  } else if ((flags.arm_state && flags.turtle_ready == 0 && rx_aux_on(AUX_BLACKBOX)) && blackbox_enabled == 0) {
    if (blackbox_device_restart(profile.blackbox.field_flags, blackbox_rate_div(), state.looptime_autodetect)) {
      blackbox_rate = blackbox_rate_div();
      blackbox_enabled = 1;
      blackbox.loop = 0;
    }
    return;
  }

  if (blackbox_enabled == 0) {
    return;
  }

  if ((state.loop_counter % blackbox_rate) != 0) {
    return;
  }

  blackbox.loop++;
  blackbox.time = time_micros();

  vec3_compress(&blackbox.pid_p_term, &state.pid_p_term, BLACKBOX_SCALE);
  vec3_compress(&blackbox.pid_i_term, &state.pid_i_term, BLACKBOX_SCALE);
  vec3_compress(&blackbox.pid_d_term, &state.pid_d_term, BLACKBOX_SCALE);

  vec4_compress(&blackbox.rx, &state.rx, BLACKBOX_SCALE);

  blackbox.setpoint.roll = state.setpoint.roll * BLACKBOX_SCALE;
  blackbox.setpoint.pitch = state.setpoint.pitch * BLACKBOX_SCALE;
  blackbox.setpoint.yaw = state.setpoint.yaw * BLACKBOX_SCALE;
  blackbox.setpoint.throttle = state.throttle * BLACKBOX_SCALE;

  vec3_compress(&blackbox.gyro_filter, &state.gyro, BLACKBOX_SCALE);
  vec3_compress(&blackbox.gyro_raw, &state.gyro_raw, BLACKBOX_SCALE);

  vec3_compress(&blackbox.accel_filter, &state.accel, BLACKBOX_SCALE);
  vec3_compress(&blackbox.accel_raw, &state.accel_raw, BLACKBOX_SCALE);

  vec4_compress(&blackbox.motor, &state.motor_mix, BLACKBOX_SCALE);

  blackbox.cpu_load = state.cpu_load;

  blackbox_device_write(profile.blackbox.field_flags, &blackbox);
}
#else
void blackbox_init() {}
void blackbox_set_debug(blackbox_debug_flag_t flag, uint8_t index, int16_t data) {}
void blackbox_update() {}
#endif