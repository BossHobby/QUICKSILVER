#include "blackbox.h"

#include "control.h"
#include "data_flash.h"
#include "drv_time.h"
#include "usb_configurator.h"
#include "util.h"
#include "util/cbor_helper.h"

#ifdef ENABLE_BLACKBOX

#define BLACKBOX_SCALE 1000

uint32_t blackbox_rate = 4;
blackbox_t blackbox;

static uint8_t blackbox_enabled = 0;

cbor_result_t cbor_encode_blackbox_t(cbor_value_t *enc, const blackbox_t *b) {
  CBOR_CHECK_ERROR(cbor_result_t res = cbor_encode_array_indefinite(enc));

  CBOR_CHECK_ERROR(res = cbor_encode_uint32(enc, &b->loop));
  CBOR_CHECK_ERROR(res = cbor_encode_uint32(enc, &b->time));

  CBOR_CHECK_ERROR(res = cbor_encode_compact_vec3_t(enc, &b->pid_p_term));
  CBOR_CHECK_ERROR(res = cbor_encode_compact_vec3_t(enc, &b->pid_i_term));
  CBOR_CHECK_ERROR(res = cbor_encode_compact_vec3_t(enc, &b->pid_d_term));

  CBOR_CHECK_ERROR(res = cbor_encode_compact_vec4_t(enc, &b->rx));
  CBOR_CHECK_ERROR(res = cbor_encode_compact_vec4_t(enc, &b->setpoint));

  CBOR_CHECK_ERROR(res = cbor_encode_compact_vec3_t(enc, &b->gyro_raw));
  CBOR_CHECK_ERROR(res = cbor_encode_compact_vec3_t(enc, &b->gyro_filter));

  CBOR_CHECK_ERROR(res = cbor_encode_compact_vec3_t(enc, &b->accel_raw));
  CBOR_CHECK_ERROR(res = cbor_encode_compact_vec3_t(enc, &b->accel_filter));

  CBOR_CHECK_ERROR(res = cbor_encode_compact_vec4_t(enc, &b->motor));
  CBOR_CHECK_ERROR(res = cbor_encode_uint16(enc, &b->cpu_load));

  CBOR_CHECK_ERROR(res = cbor_encode_end_indefinite(enc));

  return res;
}

void blackbox_init() {
  data_flash_init();
}

uint8_t blackbox_update() {
  static uint32_t loop_counter = 0;

  if ((!rx_aux_on(AUX_ARMING) || !rx_aux_on(AUX_BLACKBOX)) && blackbox_enabled == 1) {
    data_flash_finish();
    blackbox_enabled = 0;
    return 0;
  } else if ((rx_aux_on(AUX_ARMING) && rx_aux_on(AUX_BLACKBOX)) && blackbox_enabled == 0) {
    data_flash_restart();
    blackbox_enabled = 1;
    return 0;
  }

  uint8_t write_in_progress = data_flash_update(loop_counter);

  if (blackbox_enabled == 0)
    return 0;

  blackbox.loop = loop_counter / blackbox_rate;
  blackbox.time = timer_micros();

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

  if (blackbox_enabled != 0 && (loop_counter % blackbox_rate) == 0) {
    data_flash_write_backbox(&blackbox);
  }

  loop_counter++;

  return write_in_progress;
}
#endif