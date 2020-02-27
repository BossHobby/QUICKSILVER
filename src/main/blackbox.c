#include "blackbox.h"

#include "data_flash.h"
#include "drv_time.h"
#include "usb_configurator.h"

uint32_t blackbox_rate = 2;
uint8_t blackbox_override = 0;
blackbox_t state;

static uint8_t blackbox_enabled = 0;

extern uint8_t usb_is_active;

extern float cpu_load;
extern float vbattfilt;

extern float rx[4];
extern float rx_filtered[4];
extern uint8_t aux[AUX_CHANNEL_MAX];

extern float gyro[3];
extern float gyro_raw[3];
extern float GEstG[3];

extern float accel[3];
extern float accel_filter[3];

extern float pidoutput[3];

#define CHECK_CBOR_ERROR(expr) \
  expr;                        \
  if (res < CBOR_OK) {         \
    return res;                \
  }

cbor_result_t cbor_encode_float_array(cbor_value_t *enc, const float *array, uint32_t size) {
  CHECK_CBOR_ERROR(cbor_result_t res = cbor_encode_array(enc, size))

  for (uint32_t i = 0; i < size; i++) {
    CHECK_CBOR_ERROR(res = cbor_encode_float(enc, &array[i]));
  }

  return res;
}

cbor_result_t cbor_encode_uint8_array(cbor_value_t *enc, const uint8_t *array, uint32_t size) {
  CHECK_CBOR_ERROR(cbor_result_t res = cbor_encode_array(enc, size));

  for (uint32_t i = 0; i < size; i++) {
    CHECK_CBOR_ERROR(res = cbor_encode_uint8(enc, &array[i]));
  }

  return res;
}

cbor_result_t cbor_decode_float_array(cbor_value_t *enc, float *array, uint32_t size) {
  cbor_container_t container;
  CHECK_CBOR_ERROR(cbor_result_t res = cbor_decode_array(enc, &container))

  for (uint32_t i = 0; i < size; i++) {
    CHECK_CBOR_ERROR(res = cbor_decode_float(enc, &array[i]));
  }

  return res;
}

cbor_result_t cbor_decode_uint8_array(cbor_value_t *enc, uint8_t *array, uint32_t size) {
  cbor_container_t container;
  CHECK_CBOR_ERROR(cbor_result_t res = cbor_decode_array(enc, &container));

  for (uint32_t i = 0; i < size; i++) {
    CHECK_CBOR_ERROR(res = cbor_decode_uint8(enc, &array[i]));
  }

  return res;
}

cbor_result_t cbor_encode_blackbox_t(cbor_value_t *enc, const blackbox_t *b) {
  CHECK_CBOR_ERROR(cbor_result_t res = cbor_encode_map_indefinite(enc));

  CHECK_CBOR_ERROR(res = cbor_encode_str(enc, "time"));
  CHECK_CBOR_ERROR(res = cbor_encode_uint32(enc, &b->time));

  CHECK_CBOR_ERROR(res = cbor_encode_str(enc, "cpu_load"));
  CHECK_CBOR_ERROR(res = cbor_encode_uint16(enc, &b->cpu_load));

  CHECK_CBOR_ERROR(res = cbor_encode_str(enc, "vbat_filter"));
  CHECK_CBOR_ERROR(res = cbor_encode_uint16(enc, &b->vbat_filter));

  CHECK_CBOR_ERROR(res = cbor_encode_str(enc, "gyro_raw"));
  CHECK_CBOR_ERROR(res = cbor_encode_float_array(enc, b->gyro_raw, 3));
  CHECK_CBOR_ERROR(res = cbor_encode_str(enc, "gyro_filter"));
  CHECK_CBOR_ERROR(res = cbor_encode_float_array(enc, b->gyro_filter, 3));
  CHECK_CBOR_ERROR(res = cbor_encode_str(enc, "gyro_vector"));
  CHECK_CBOR_ERROR(res = cbor_encode_float_array(enc, b->gyro_vector, 3));

  CHECK_CBOR_ERROR(res = cbor_encode_str(enc, "rx_raw"));
  CHECK_CBOR_ERROR(res = cbor_encode_float_array(enc, b->rx_raw, 4));
  CHECK_CBOR_ERROR(res = cbor_encode_str(enc, "rx_filter"));
  CHECK_CBOR_ERROR(res = cbor_encode_float_array(enc, b->rx_filter, 4));
  CHECK_CBOR_ERROR(res = cbor_encode_str(enc, "rx_aux"));
  CHECK_CBOR_ERROR(res = cbor_encode_uint32(enc, &b->rx_aux));

  CHECK_CBOR_ERROR(res = cbor_encode_str(enc, "accel_raw"));
  CHECK_CBOR_ERROR(res = cbor_encode_float_array(enc, b->accel_raw, 3));
  CHECK_CBOR_ERROR(res = cbor_encode_str(enc, "accel_filter"));
  CHECK_CBOR_ERROR(res = cbor_encode_float_array(enc, b->accel_filter, 3));

  CHECK_CBOR_ERROR(res = cbor_encode_str(enc, "pid_output"));
  CHECK_CBOR_ERROR(res = cbor_encode_float_array(enc, b->pid_output, 3));

  CHECK_CBOR_ERROR(res = cbor_encode_end_indefinite(enc));

  return res;
}

cbor_result_t cbor_encode_compact_blackbox_t(cbor_value_t *enc, const blackbox_t *b) {
  CHECK_CBOR_ERROR(cbor_result_t res = cbor_encode_array_indefinite(enc));

  CHECK_CBOR_ERROR(res = cbor_encode_uint32(enc, &b->time));

  CHECK_CBOR_ERROR(res = cbor_encode_uint16(enc, &b->cpu_load));

  CHECK_CBOR_ERROR(res = cbor_encode_uint16(enc, &b->vbat_filter));

  CHECK_CBOR_ERROR(res = cbor_encode_float_array(enc, b->gyro_raw, 3));
  CHECK_CBOR_ERROR(res = cbor_encode_float_array(enc, b->gyro_filter, 3));
  CHECK_CBOR_ERROR(res = cbor_encode_float_array(enc, b->gyro_vector, 3));

  CHECK_CBOR_ERROR(res = cbor_encode_float_array(enc, b->rx_raw, 4));
  CHECK_CBOR_ERROR(res = cbor_encode_float_array(enc, b->rx_filter, 4));
  CHECK_CBOR_ERROR(res = cbor_encode_uint32(enc, &b->rx_aux));

  CHECK_CBOR_ERROR(res = cbor_encode_float_array(enc, b->accel_raw, 3));
  CHECK_CBOR_ERROR(res = cbor_encode_float_array(enc, b->accel_filter, 3));

  CHECK_CBOR_ERROR(res = cbor_encode_float_array(enc, b->pid_output, 3));

  CHECK_CBOR_ERROR(res = cbor_encode_end_indefinite(enc));

  return res;
}

cbor_result_t cbor_decode_compact_blackbox_t(cbor_value_t *dec, blackbox_t *b) {
  cbor_container_t array;
  CHECK_CBOR_ERROR(cbor_result_t res = cbor_decode_array(dec, &array));

  CHECK_CBOR_ERROR(res = cbor_decode_uint32(dec, &b->time));

  CHECK_CBOR_ERROR(res = cbor_decode_uint16(dec, &b->cpu_load));

  CHECK_CBOR_ERROR(res = cbor_decode_uint16(dec, &b->vbat_filter));

  CHECK_CBOR_ERROR(res = cbor_decode_float_array(dec, b->gyro_raw, 3));
  CHECK_CBOR_ERROR(res = cbor_decode_float_array(dec, b->gyro_filter, 3));
  CHECK_CBOR_ERROR(res = cbor_decode_float_array(dec, b->gyro_vector, 3));

  CHECK_CBOR_ERROR(res = cbor_decode_float_array(dec, b->rx_raw, 4));
  CHECK_CBOR_ERROR(res = cbor_decode_float_array(dec, b->rx_filter, 4));
  CHECK_CBOR_ERROR(res = cbor_decode_uint32(dec, &b->rx_aux));

  CHECK_CBOR_ERROR(res = cbor_decode_float_array(dec, b->accel_raw, 3));
  CHECK_CBOR_ERROR(res = cbor_decode_float_array(dec, b->accel_filter, 3));

  CHECK_CBOR_ERROR(res = cbor_decode_float_array(dec, b->pid_output, 3));

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

  state.time = timer_millis();

  state.cpu_load = cpu_load;

  state.vbat_filter = vbattfilt * 10;

  state.rx_raw[0] = rx[0];
  state.rx_raw[1] = rx[1];
  state.rx_raw[2] = rx[2];
  state.rx_raw[3] = rx[3];

  state.rx_filter[0] = rx_filtered[0];
  state.rx_filter[1] = rx_filtered[1];
  state.rx_filter[2] = rx_filtered[2];
  state.rx_filter[3] = rx_filtered[3];

  state.rx_aux = 0;
  for (uint32_t i = 0; i < AUX_CHANNEL_MAX; i++) {
    if (aux[i]) {
      state.rx_aux = state.rx_aux | (0x1 << i);
    }
  }

  state.gyro_raw[0] = gyro_raw[0];
  state.gyro_raw[1] = gyro_raw[1];
  state.gyro_raw[2] = gyro_raw[2];

  state.gyro_filter[0] = gyro[0];
  state.gyro_filter[1] = gyro[1];
  state.gyro_filter[2] = gyro[2];

  state.gyro_vector[0] = GEstG[0];
  state.gyro_vector[1] = GEstG[1];
  state.gyro_vector[2] = GEstG[2];

  state.accel_raw[0] = accel[0];
  state.accel_raw[1] = accel[1];
  state.accel_raw[2] = accel[2];

  state.accel_filter[0] = accel_filter[0];
  state.accel_filter[1] = accel_filter[1];
  state.accel_filter[2] = accel_filter[2];

  state.pid_output[0] = pidoutput[0];
  state.pid_output[1] = pidoutput[1];
  state.pid_output[2] = pidoutput[2];

  if ((loop_counter % (uint32_t)((1000000.0f / (float)blackbox_rate) / LOOPTIME)) == 0) {
    if (usb_is_active != 0) {
      quic_blackbox(&state);
    }
  }

  if (rx_aux_on(AUX_ARMING) && (loop_counter % 4 == 0)) {
    data_flash_write_backbox(&state);
  }

  loop_counter++;
}