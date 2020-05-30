#include "blackbox.h"

#include "control.h"
#include "data_flash.h"
#include "drv_time.h"
#include "usb_configurator.h"
#include "util.h"
#include "util/cbor_helper.h"

uint32_t blackbox_rate = 2;
uint8_t blackbox_override = 0;
blackbox_t blackbox;

static uint8_t blackbox_enabled = 0;

extern float cpu_load;
extern float vbattfilt;

extern float rx_filtered[4];
extern uint8_t aux[AUX_CHANNEL_MAX];

extern float GEstG[3];
extern float pidoutput[3];

cbor_result_t cbor_encode_blackbox_t(cbor_value_t *enc, const blackbox_t *b) {
  CBOR_CHECK_ERROR(cbor_result_t res = cbor_encode_map_indefinite(enc));

  CBOR_CHECK_ERROR(res = cbor_encode_str(enc, "time"));
  CBOR_CHECK_ERROR(res = cbor_encode_uint32(enc, &b->time));

  CBOR_CHECK_ERROR(res = cbor_encode_str(enc, "cpu_load"));
  CBOR_CHECK_ERROR(res = cbor_encode_uint16(enc, &b->cpu_load));

  CBOR_CHECK_ERROR(res = cbor_encode_str(enc, "vbat_filter"));
  CBOR_CHECK_ERROR(res = cbor_encode_uint16(enc, &b->vbat_filter));

  CBOR_CHECK_ERROR(res = cbor_encode_str(enc, "gyro_raw"));
  CBOR_CHECK_ERROR(res = cbor_encode_compact_vec3_t(enc, &b->gyro_raw));
  CBOR_CHECK_ERROR(res = cbor_encode_str(enc, "gyro_filter"));
  CBOR_CHECK_ERROR(res = cbor_encode_compact_vec3_t(enc, &b->gyro_filter));
  CBOR_CHECK_ERROR(res = cbor_encode_str(enc, "gyro_vector"));
  CBOR_CHECK_ERROR(res = cbor_encode_float_array(enc, b->gyro_vector, 3));

  CBOR_CHECK_ERROR(res = cbor_encode_str(enc, "rx_raw"));
  CBOR_CHECK_ERROR(res = cbor_encode_vec4_t(enc, &b->rx_raw));
  CBOR_CHECK_ERROR(res = cbor_encode_str(enc, "rx_filter"));
  CBOR_CHECK_ERROR(res = cbor_encode_float_array(enc, b->rx_filter, 4));
  CBOR_CHECK_ERROR(res = cbor_encode_str(enc, "rx_aux"));
  CBOR_CHECK_ERROR(res = cbor_encode_uint32(enc, &b->rx_aux));

  CBOR_CHECK_ERROR(res = cbor_encode_str(enc, "accel_raw"));
  CBOR_CHECK_ERROR(res = cbor_encode_compact_vec3_t(enc, &b->accel_raw));
  CBOR_CHECK_ERROR(res = cbor_encode_str(enc, "accel_filter"));
  CBOR_CHECK_ERROR(res = cbor_encode_compact_vec3_t(enc, &b->accel_filter));

  CBOR_CHECK_ERROR(res = cbor_encode_str(enc, "pid_output"));
  CBOR_CHECK_ERROR(res = cbor_encode_float_array(enc, b->pid_output, 3));

  CBOR_CHECK_ERROR(res = cbor_encode_end_indefinite(enc));

  return res;
}

cbor_result_t cbor_encode_compact_blackbox_t(cbor_value_t *enc, const blackbox_t *b) {
  CBOR_CHECK_ERROR(cbor_result_t res = cbor_encode_array_indefinite(enc));

  CBOR_CHECK_ERROR(res = cbor_encode_uint32(enc, &b->time));

  CBOR_CHECK_ERROR(res = cbor_encode_uint16(enc, &b->cpu_load));

  CBOR_CHECK_ERROR(res = cbor_encode_uint16(enc, &b->vbat_filter));

  CBOR_CHECK_ERROR(res = cbor_encode_compact_vec3_t(enc, &b->gyro_raw));
  CBOR_CHECK_ERROR(res = cbor_encode_compact_vec3_t(enc, &b->gyro_filter));

  CBOR_CHECK_ERROR(res = cbor_encode_vec4_t(enc, &b->rx_raw));
  CBOR_CHECK_ERROR(res = cbor_encode_float_array(enc, b->rx_filter, 4));
  CBOR_CHECK_ERROR(res = cbor_encode_uint32(enc, &b->rx_aux));

  CBOR_CHECK_ERROR(res = cbor_encode_compact_vec3_t(enc, &b->accel_raw));
  CBOR_CHECK_ERROR(res = cbor_encode_compact_vec3_t(enc, &b->accel_filter));

  CBOR_CHECK_ERROR(res = cbor_encode_float_array(enc, b->gyro_vector, 3));
  CBOR_CHECK_ERROR(res = cbor_encode_float_array(enc, b->pid_output, 3));

  CBOR_CHECK_ERROR(res = cbor_encode_end_indefinite(enc));

  return res;
}

cbor_result_t cbor_decode_compact_blackbox_t(cbor_value_t *dec, blackbox_t *b) {
  cbor_container_t array;
  CBOR_CHECK_ERROR(cbor_result_t res = cbor_decode_array(dec, &array));

  CBOR_CHECK_ERROR(res = cbor_decode_uint32(dec, &b->time));

  CBOR_CHECK_ERROR(res = cbor_decode_uint16(dec, &b->cpu_load));

  CBOR_CHECK_ERROR(res = cbor_decode_uint16(dec, &b->vbat_filter));

  CBOR_CHECK_ERROR(res = cbor_encode_compact_vec3_t(dec, &b->gyro_raw));
  CBOR_CHECK_ERROR(res = cbor_encode_compact_vec3_t(dec, &b->gyro_filter));
  CBOR_CHECK_ERROR(res = cbor_decode_float_array(dec, b->gyro_vector, 3));

  CBOR_CHECK_ERROR(res = cbor_encode_vec4_t(dec, &b->rx_raw));
  CBOR_CHECK_ERROR(res = cbor_decode_float_array(dec, b->rx_filter, 4));
  CBOR_CHECK_ERROR(res = cbor_decode_uint32(dec, &b->rx_aux));

  CBOR_CHECK_ERROR(res = cbor_encode_compact_vec3_t(dec, &b->accel_raw));
  CBOR_CHECK_ERROR(res = cbor_encode_compact_vec3_t(dec, &b->accel_filter));

  CBOR_CHECK_ERROR(res = cbor_decode_float_array(dec, b->pid_output, 3));

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

  if (blackbox_enabled == 0 && blackbox_override == 0)
    return;

  blackbox.time = timer_millis();

  blackbox.cpu_load = cpu_load;

  blackbox.vbat_filter = vbattfilt * 10;

  blackbox.rx_raw = state.rx;

  blackbox.rx_filter[0] = rx_filtered[0];
  blackbox.rx_filter[1] = rx_filtered[1];
  blackbox.rx_filter[2] = rx_filtered[2];
  blackbox.rx_filter[3] = rx_filtered[3];

  blackbox.rx_aux = 0;
  for (uint32_t i = 0; i < AUX_CHANNEL_MAX; i++) {
    if (aux[i]) {
      blackbox.rx_aux = blackbox.rx_aux | (0x1 << i);
    }
  }

  vec3_compress(&blackbox.gyro_filter, &state.gyro, 1024);
  vec3_compress(&blackbox.gyro_raw, &state.gyro_raw, 1024);

  vec3_compress(&blackbox.accel_filter, &state.accel, 1024);
  vec3_compress(&blackbox.accel_raw, &state.accel_raw, 1024);

  blackbox.gyro_vector[0] = GEstG[0];
  blackbox.gyro_vector[1] = GEstG[1];
  blackbox.gyro_vector[2] = GEstG[2];

  blackbox.pid_output[0] = pidoutput[0];
  blackbox.pid_output[1] = pidoutput[1];
  blackbox.pid_output[2] = pidoutput[2];

  if ((loop_counter % (uint32_t)((1000000.0f / (float)blackbox_rate) / LOOPTIME)) == 0) {
    if (flags.usb_active != 0) {
      quic_blackbox(&blackbox);
    }
  }

  if (rx_aux_on(AUX_ARMING) && (loop_counter % 4 == 0)) {
    data_flash_write_backbox(&blackbox);
  }

  loop_counter++;
}