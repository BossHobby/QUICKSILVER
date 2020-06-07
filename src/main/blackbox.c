#include "blackbox.h"

#include "control.h"
#include "data_flash.h"
#include "drv_time.h"
#include "usb_configurator.h"
#include "util.h"
#include "util/cbor_helper.h"

uint32_t blackbox_rate = 4;
blackbox_t blackbox;

static uint8_t blackbox_enabled = 0;

cbor_result_t cbor_encode_blackbox_t(cbor_value_t *enc, const blackbox_t *b) {
  CBOR_CHECK_ERROR(cbor_result_t res = cbor_encode_array_indefinite(enc));

  CBOR_CHECK_ERROR(res = cbor_encode_uint32(enc, &b->time));
  CBOR_CHECK_ERROR(res = cbor_encode_uint16(enc, &b->cpu_load));
  CBOR_CHECK_ERROR(res = cbor_encode_uint16(enc, &b->vbat_filter));

  CBOR_CHECK_ERROR(res = cbor_encode_compact_vec4_t(enc, &b->rx_raw));
  CBOR_CHECK_ERROR(res = cbor_encode_compact_vec4_t(enc, &b->rx_filter));
  CBOR_CHECK_ERROR(res = cbor_encode_uint32(enc, &b->rx_aux));

  CBOR_CHECK_ERROR(res = cbor_encode_compact_vec3_t(enc, &b->gyro_raw));
  CBOR_CHECK_ERROR(res = cbor_encode_compact_vec3_t(enc, &b->gyro_filter));

  CBOR_CHECK_ERROR(res = cbor_encode_compact_vec3_t(enc, &b->accel_raw));
  CBOR_CHECK_ERROR(res = cbor_encode_compact_vec3_t(enc, &b->accel_filter));

  CBOR_CHECK_ERROR(res = cbor_encode_compact_vec3_t(enc, &b->gyro_vector));
  CBOR_CHECK_ERROR(res = cbor_encode_compact_vec3_t(enc, &b->pid_output));

  CBOR_CHECK_ERROR(res = cbor_encode_end_indefinite(enc));

  return res;
}

void blackbox_init() {
  data_flash_init();
}

void blackbox_update() {
  static uint32_t loop_counter = 0;

  if (!rx_aux_on(AUX_ARMING) && blackbox_enabled == 1) {
    data_flash_finish();
    blackbox_enabled = 0;
    return;
  } else if (rx_aux_on(AUX_ARMING) && blackbox_enabled == 0) {
    data_flash_restart();
    blackbox_enabled = 1;
    return;
  }

  if (blackbox_enabled == 0)
    return;

  blackbox.time = timer_millis();
  blackbox.cpu_load = state.cpu_load;
  blackbox.vbat_filter = state.vbattfilt * 10;

  vec4_compress(&blackbox.rx_raw, &state.rx, 1024);
  vec4_compress(&blackbox.rx_filter, &state.rx_filtered, 1024);

  blackbox.rx_aux = 0;
  for (uint32_t i = 0; i < AUX_CHANNEL_MAX; i++) {
    if (state.aux[i]) {
      blackbox.rx_aux = blackbox.rx_aux | (0x1 << i);
    }
  }

  vec3_compress(&blackbox.gyro_filter, &state.gyro, 1024);
  vec3_compress(&blackbox.gyro_raw, &state.gyro_raw, 1024);

  vec3_compress(&blackbox.accel_filter, &state.accel, 1024);
  vec3_compress(&blackbox.accel_raw, &state.accel_raw, 1024);

  vec3_compress(&blackbox.gyro_vector, &state.GEstG, 1024);
  vec3_compress(&blackbox.pid_output, &state.pidoutput, 1024);

  if (blackbox_enabled != 0 && (loop_counter % blackbox_rate) == 0) {
    data_flash_write_backbox(&blackbox);
  }

  loop_counter++;
}