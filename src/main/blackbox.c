#include "blackbox.h"

#include <string.h>

#include "drv_spi_m25p16.h"
#include "drv_time.h"
#include "usb_configurator.h"

uint32_t blackbox_rate = 2;
uint8_t blackbox_enabled = 0;
uint8_t blackbox_flash_enabled = 1;
uint32_t blackbox_flash_offset = 0;

blackbox_t state;

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
  CHECK_CBOR_ERROR(res = cbor_encode_float(enc, &b->cpu_load));

  CHECK_CBOR_ERROR(res = cbor_encode_str(enc, "vbat_filter"));
  CHECK_CBOR_ERROR(res = cbor_encode_float(enc, &b->vbat_filter));

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
  CHECK_CBOR_ERROR(res = cbor_encode_uint8_array(enc, b->rx_aux, AUX_CHANNEL_MAX));

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

  CHECK_CBOR_ERROR(res = cbor_encode_float(enc, &b->cpu_load));

  CHECK_CBOR_ERROR(res = cbor_encode_float(enc, &b->vbat_filter));

  CHECK_CBOR_ERROR(res = cbor_encode_float_array(enc, b->gyro_raw, 3));
  CHECK_CBOR_ERROR(res = cbor_encode_float_array(enc, b->gyro_filter, 3));
  CHECK_CBOR_ERROR(res = cbor_encode_float_array(enc, b->gyro_vector, 3));

  CHECK_CBOR_ERROR(res = cbor_encode_float_array(enc, b->rx_raw, 4));
  CHECK_CBOR_ERROR(res = cbor_encode_float_array(enc, b->rx_filter, 4));
  CHECK_CBOR_ERROR(res = cbor_encode_uint8_array(enc, b->rx_aux, AUX_CHANNEL_MAX));

  CHECK_CBOR_ERROR(res = cbor_encode_float_array(enc, b->accel_raw, 3));
  CHECK_CBOR_ERROR(res = cbor_encode_float_array(enc, b->accel_filter, 3));

  CHECK_CBOR_ERROR(res = cbor_encode_float_array(enc, b->pid_output, 3));

  CHECK_CBOR_ERROR(res = cbor_encode_end_indefinite(enc));

  return res;
}

cbor_result_t cbor_decode_compact_blackbox_t(cbor_value_t *dec, blackbox_t *b) {
  cbor_container_t array;
  cbor_decode_array_size(dec, &array);

  CHECK_CBOR_ERROR(cbor_result_t res = cbor_decode_uint32(dec, &b->time));

  CHECK_CBOR_ERROR(res = cbor_decode_float(dec, &b->cpu_load));

  CHECK_CBOR_ERROR(res = cbor_decode_float(dec, &b->vbat_filter));

  CHECK_CBOR_ERROR(res = cbor_decode_float_array(dec, b->gyro_raw, 3));
  CHECK_CBOR_ERROR(res = cbor_decode_float_array(dec, b->gyro_filter, 3));
  CHECK_CBOR_ERROR(res = cbor_decode_float_array(dec, b->gyro_vector, 3));

  CHECK_CBOR_ERROR(res = cbor_decode_float_array(dec, b->rx_raw, 4));
  CHECK_CBOR_ERROR(res = cbor_decode_float_array(dec, b->rx_filter, 4));
  CHECK_CBOR_ERROR(res = cbor_decode_uint8_array(dec, b->rx_aux, AUX_CHANNEL_MAX));

  CHECK_CBOR_ERROR(res = cbor_decode_float_array(dec, b->accel_raw, 3));
  CHECK_CBOR_ERROR(res = cbor_decode_float_array(dec, b->accel_filter, 3));

  CHECK_CBOR_ERROR(res = cbor_decode_float_array(dec, b->pid_output, 3));

  return res;
}

void blackbox_init() {
#ifdef USE_M25P16
  m25p16_init();

  m25p16_command(M25P16_WRITE_ENABLE);
  m25p16_command(M25P16_BULK_ERASE);
#endif
}

cbor_result_t blackbox_read_flash(const uint32_t addr, blackbox_t *b) {

  while ((m25p16_command(M25P16_READ_STATUS_REGISTER) & 0x01))
    ;

  uint8_t data[255];
  m25p16_read_addr(M25P16_READ_DATA_BYTES, addr, data, 255);

  cbor_value_t dec;
  cbor_decoder_init(&dec, data, 255);
  return cbor_decode_compact_blackbox_t(&dec, b);
}

void blackbox_update() {
  static uint32_t loop_counter = 0;

  if (blackbox_enabled == 0)
    return;

  state.time = timer_millis();

  state.cpu_load = cpu_load;

  state.vbat_filter = vbattfilt;

  state.rx_raw[0] = rx[0];
  state.rx_raw[1] = rx[1];
  state.rx_raw[2] = rx[2];
  state.rx_raw[3] = rx[3];

  state.rx_filter[0] = rx_filtered[0];
  state.rx_filter[1] = rx_filtered[1];
  state.rx_filter[2] = rx_filtered[2];
  state.rx_filter[3] = rx_filtered[3];

  for (uint32_t i = 0; i < AUX_CHANNEL_MAX; i++) {
    state.rx_aux[i] = aux[i];
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
    if (blackbox_flash_enabled) {
      static uint8_t data[255];
      memset(data, 0, 255);

      cbor_value_t enc;
      cbor_encoder_init(&enc, data, 255);

      cbor_result_t res = cbor_encode_compact_blackbox_t(&enc, &state);
      if (res < CBOR_OK) {
        quic_debugf("CBOR ERROR %d", res);
        return;
      }

      while ((m25p16_command(M25P16_READ_STATUS_REGISTER) & 0x01) != 0)
        ;
      m25p16_command(M25P16_WRITE_ENABLE);
      m25p16_write_addr(M25P16_PAGE_PROGRAM, blackbox_flash_offset, data, 255);
      blackbox_flash_offset += 256;
    }
  }

  loop_counter++;
}