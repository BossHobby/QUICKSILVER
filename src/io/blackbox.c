#include "io/blackbox.h"

#include "driver/time.h"
#include "flight/control.h"
#include "io/data_flash.h"
#include "io/usb_configurator.h"
#include "util/cbor_helper.h"
#include "util/util.h"

#ifdef ENABLE_BLACKBOX

static blackbox_t blackbox;

static uint8_t blackbox_enabled = 0;

cbor_result_t cbor_encode_blackbox_t(cbor_value_t *enc, const blackbox_t *b, const uint32_t blackbox_fieldflags) {
  CBOR_CHECK_ERROR(cbor_result_t res = cbor_encode_array_indefinite(enc));

  // loop and time are always emitted regardless of what blackbox_fieldflags indicates
  CBOR_CHECK_ERROR(res = cbor_encode_uint32(enc, &b->loop));
  CBOR_CHECK_ERROR(res = cbor_encode_uint32(enc, &b->time));

  if ((1 << BBOX_FIELD_PID_P_TERM) & blackbox_fieldflags) {
    CBOR_CHECK_ERROR(res = cbor_encode_compact_vec3_t(enc, &b->pid_p_term));
  }
  if ((1 << BBOX_FIELD_PID_I_TERM) & blackbox_fieldflags) {
    CBOR_CHECK_ERROR(res = cbor_encode_compact_vec3_t(enc, &b->pid_i_term));
  }
  if ((1 << BBOX_FIELD_PID_D_TERM) & blackbox_fieldflags) {
    CBOR_CHECK_ERROR(res = cbor_encode_compact_vec3_t(enc, &b->pid_d_term));
  }

  if ((1 << BBOX_FIELD_RX) & blackbox_fieldflags) {
    CBOR_CHECK_ERROR(res = cbor_encode_compact_vec4_t(enc, &b->rx));
  }

  if ((1 << BBOX_FIELD_SETPOINT) & blackbox_fieldflags) {
    CBOR_CHECK_ERROR(res = cbor_encode_compact_vec4_t(enc, &b->setpoint));
  }

  if ((1 << BBOX_FIELD_ACCEL_RAW) & blackbox_fieldflags) {
    CBOR_CHECK_ERROR(res = cbor_encode_compact_vec3_t(enc, &b->accel_raw));
  }

  if ((1 << BBOX_FIELD_ACCEL_FILTER) & blackbox_fieldflags) {
    CBOR_CHECK_ERROR(res = cbor_encode_compact_vec3_t(enc, &b->accel_filter));
  }

  if ((1 << BBOX_FIELD_GYRO_RAW) & blackbox_fieldflags) {
    CBOR_CHECK_ERROR(res = cbor_encode_compact_vec3_t(enc, &b->gyro_raw));
  }

  if ((1 << BBOX_FIELD_GYRO_FILTER) & blackbox_fieldflags) {
    CBOR_CHECK_ERROR(res = cbor_encode_compact_vec3_t(enc, &b->gyro_filter));
  }

  if ((1 << BBOX_FIELD_MOTOR) & blackbox_fieldflags) {
    CBOR_CHECK_ERROR(res = cbor_encode_compact_vec4_t(enc, &b->motor));
  }

  if ((1 << BBOX_FIELD_CPU_LOAD) & blackbox_fieldflags) {
    CBOR_CHECK_ERROR(res = cbor_encode_uint16(enc, &b->cpu_load));
  }

  if ((1 << BBOX_FIELD_DEBUG) & blackbox_fieldflags) {
    CBOR_CHECK_ERROR(res = cbor_encode_array(enc, 4));
    CBOR_CHECK_ERROR(res = cbor_encode_int16(enc, &b->debug[0]));
    CBOR_CHECK_ERROR(res = cbor_encode_int16(enc, &b->debug[1]));
    CBOR_CHECK_ERROR(res = cbor_encode_int16(enc, &b->debug[2]));
    CBOR_CHECK_ERROR(res = cbor_encode_int16(enc, &b->debug[3]));
  }

  CBOR_CHECK_ERROR(res = cbor_encode_end_indefinite(enc));

  return res;
}

void blackbox_init() {
  data_flash_init();
}

void blackbox_set_debug(uint8_t index, int16_t data) {
  if (index >= 4) {
    return;
  }

  blackbox.debug[index] = data;
}

uint8_t blackbox_update() {
  static uint32_t loop_counter = 0;

  data_flash_result_t flash_result = data_flash_update();

  if (flash_result == DATA_FLASH_DETECT || flash_result == DATA_FLASH_STARTING) {
    // flash is still detecting, dont do anything
    return 0;
  }

  // flash is either idle or writing, do blackbox

  if ((!flags.arm_state || !rx_aux_on(AUX_BLACKBOX)) && blackbox_enabled == 1) {
    data_flash_finish();
    blackbox_enabled = 0;
    return 0;
  } else if ((flags.arm_state && flags.turtle_ready == 0 && rx_aux_on(AUX_BLACKBOX)) && blackbox_enabled == 0) {
    if (data_flash_restart(profile.blackbox.blackbox_fieldflags, profile.blackbox.rate_divisor, state.looptime_autodetect)) {
      blackbox_enabled = 1;
    }
    return 0;
  }

  if (blackbox_enabled == 0) {
    return 0;
  }

  blackbox.loop = loop_counter / profile.blackbox.rate_divisor;
  blackbox.time = time_micros();

  vec3_compress(&blackbox.pid_p_term, &state.pid_p_term, BLACKBOX_SCALE);
  vec3_compress(&blackbox.pid_i_term, &state.pid_i_term, BLACKBOX_SCALE);
  vec3_compress(&blackbox.pid_d_term, &state.pid_d_term, BLACKBOX_SCALE);

  vec4_compress(&blackbox.rx, &state.rx, BLACKBOX_SCALE);

  blackbox.setpoint.axis[0] = state.setpoint.axis[0] * BLACKBOX_SCALE;
  blackbox.setpoint.axis[1] = state.setpoint.axis[1] * BLACKBOX_SCALE;
  blackbox.setpoint.axis[2] = state.setpoint.axis[2] * BLACKBOX_SCALE;
  blackbox.setpoint.axis[3] = state.throttle * BLACKBOX_SCALE;

  vec3_compress(&blackbox.gyro_filter, &state.gyro, BLACKBOX_SCALE);
  vec3_compress(&blackbox.gyro_raw, &state.gyro_raw, BLACKBOX_SCALE);

  vec3_compress(&blackbox.accel_filter, &state.accel, BLACKBOX_SCALE);
  vec3_compress(&blackbox.accel_raw, &state.accel_raw, BLACKBOX_SCALE);

  vec4_compress(&blackbox.motor, &state.motor_mix, BLACKBOX_SCALE);

  blackbox.cpu_load = state.cpu_load;

  if (blackbox_enabled != 0 && (loop_counter % profile.blackbox.rate_divisor) == 0) {
    data_flash_write_backbox(profile.blackbox.blackbox_fieldflags, &blackbox);
  }

  loop_counter++;

  // tell the rest of the code that flash is occuping the spi bus
  return flash_result == DATA_FLASH_WRITE;
}
#else
void blackbox_set_debug(uint8_t index, int16_t data) {}
#endif