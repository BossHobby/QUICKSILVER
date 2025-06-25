#include "io/blackbox.h"

#include "driver/time.h"
#include "flight/control.h"
#include "io/blackbox_device.h"
#include "io/usb_configurator.h"
#include "util/cbor_helper.h"
#include "util/util.h"

#ifdef USE_BLACKBOX

#define BLACKBOX_I_FRAME_INTERVAL 32  // Every 32nd frame is an I-frame
#define BLACKBOX_FIELD_ENABLED(flags, field) ((flags) & (1 << (field)))

static blackbox_t blackbox;
static blackbox_t blackbox_previous;  // Store previous frame for delta encoding
static uint8_t blackbox_enabled = 0;
static uint8_t blackbox_has_previous = 0;
static uint32_t blackbox_rate = 1;

static int16_t blackbox_compress_float(float value) {
  const float scaled = value * BLACKBOX_SCALE;
  if (scaled > 32767.0f) {
    return 32767;
  }
  if (scaled < -32768.0f) {
    return -32768;
  }
  return scaled;
}

static void blackbox_compress_vec3(compact_vec3_t *out, const vec3_t *in) {
  out->axis[0] = blackbox_compress_float(in->axis[0]);
  out->axis[1] = blackbox_compress_float(in->axis[1]);
  out->axis[2] = blackbox_compress_float(in->axis[2]);
}

static void blackbox_compress_vec4(compact_vec4_t *out, const vec4_t *in) {
  out->axis[0] = blackbox_compress_float(in->axis[0]);
  out->axis[1] = blackbox_compress_float(in->axis[1]);
  out->axis[2] = blackbox_compress_float(in->axis[2]);
  out->axis[3] = blackbox_compress_float(in->axis[3]);
}

// Helper functions for delta encoding
static inline int16_t delta_int16(int16_t current, int16_t previous) {
  return current - previous;
}

static void compact_vec3_delta(compact_vec3_t *delta, const compact_vec3_t *current, const compact_vec3_t *previous) {
  for (int i = 0; i < 3; i++) {
    delta->axis[i] = delta_int16(current->axis[i], previous->axis[i]);
  }
}

static void compact_vec4_delta(compact_vec4_t *delta, const compact_vec4_t *current, const compact_vec4_t *previous) {
  for (int i = 0; i < 4; i++) {
    delta->axis[i] = delta_int16(current->axis[i], previous->axis[i]);
  }
}

// Check if vec3 is zero
static bool compact_vec3_is_zero(const compact_vec3_t *vec) {
  return vec->axis[0] == 0 && vec->axis[1] == 0 && vec->axis[2] == 0;
}

// Check if vec4 is zero
static bool compact_vec4_is_zero(const compact_vec4_t *vec) {
  return vec->axis[0] == 0 && vec->axis[1] == 0 && vec->axis[2] == 0 && vec->axis[3] == 0;
}

// Check if debug array has changed
static bool debug_has_changed(const int16_t *current, const int16_t *previous, uint32_t size) {
  for (uint32_t i = 0; i < size; i++) {
    if (current[i] != previous[i]) {
      return true;
    }
  }
  return false;
}


// Helper function to compute and check if delta should be encoded
static inline bool compute_and_check_vec3_delta(compact_vec3_t *delta, const compact_vec3_t *current, const compact_vec3_t *previous) {
  compact_vec3_delta(delta, current, previous);
  return !compact_vec3_is_zero(delta);
}

static inline bool compute_and_check_vec4_delta(compact_vec4_t *delta, const compact_vec4_t *current, const compact_vec4_t *previous) {
  compact_vec4_delta(delta, current, previous);
  return !compact_vec4_is_zero(delta);
}

// Helper function to encode vec3 field if present in flags
static inline cbor_result_t encode_vec3_field_if_present(cbor_value_t *enc, uint32_t flags, blackbox_field_t field_id, const compact_vec3_t *data) {
  if (flags & (1 << field_id)) {
    return cbor_encode_compact_vec3_t(enc, data);
  }
  return CBOR_OK;
}

// Helper function to encode vec4 field if present in flags
static inline cbor_result_t encode_vec4_field_if_present(cbor_value_t *enc, uint32_t flags, blackbox_field_t field_id, const compact_vec4_t *data) {
  if (flags & (1 << field_id)) {
    return cbor_encode_compact_vec4_t(enc, data);
  }
  return CBOR_OK;
}

// Encode a frame with I-frame/P-frame compression
cbor_result_t cbor_encode_blackbox_frame(cbor_value_t *enc, const blackbox_t *current, const blackbox_t *previous, blackbox_frame_type_t frame_type, const uint32_t field_flags) {
  CBOR_CHECK_ERROR(cbor_result_t res = cbor_encode_array_indefinite(enc));

  blackbox_t deltas = {0};
  uint32_t encoded_field_flags = field_flags;
  
  // For P-frames, compute deltas and determine which fields to include
  if (frame_type == BLACKBOX_FRAME_P) {
    uint32_t active_fields = 0;
    
    // Check each field and compute deltas only for non-zero changes
    if ((field_flags & (1 << BBOX_FIELD_PID_P_TERM)) && compute_and_check_vec3_delta(&deltas.pid_p_term, &current->pid_p_term, &previous->pid_p_term)) {
      active_fields |= (1 << BBOX_FIELD_PID_P_TERM);
    }
    if ((field_flags & (1 << BBOX_FIELD_PID_I_TERM)) && compute_and_check_vec3_delta(&deltas.pid_i_term, &current->pid_i_term, &previous->pid_i_term)) {
      active_fields |= (1 << BBOX_FIELD_PID_I_TERM);
    }
    if ((field_flags & (1 << BBOX_FIELD_PID_D_TERM)) && compute_and_check_vec3_delta(&deltas.pid_d_term, &current->pid_d_term, &previous->pid_d_term)) {
      active_fields |= (1 << BBOX_FIELD_PID_D_TERM);
    }
    if ((field_flags & (1 << BBOX_FIELD_RX)) && compute_and_check_vec4_delta(&deltas.rx, &current->rx, &previous->rx)) {
      active_fields |= (1 << BBOX_FIELD_RX);
    }
    if ((field_flags & (1 << BBOX_FIELD_SETPOINT)) && compute_and_check_vec4_delta(&deltas.setpoint, &current->setpoint, &previous->setpoint)) {
      active_fields |= (1 << BBOX_FIELD_SETPOINT);
    }
    if ((field_flags & (1 << BBOX_FIELD_ACCEL_RAW)) && compute_and_check_vec3_delta(&deltas.accel_raw, &current->accel_raw, &previous->accel_raw)) {
      active_fields |= (1 << BBOX_FIELD_ACCEL_RAW);
    }
    if ((field_flags & (1 << BBOX_FIELD_ACCEL_FILTER)) && compute_and_check_vec3_delta(&deltas.accel_filter, &current->accel_filter, &previous->accel_filter)) {
      active_fields |= (1 << BBOX_FIELD_ACCEL_FILTER);
    }
    if ((field_flags & (1 << BBOX_FIELD_GYRO_RAW)) && compute_and_check_vec3_delta(&deltas.gyro_raw, &current->gyro_raw, &previous->gyro_raw)) {
      active_fields |= (1 << BBOX_FIELD_GYRO_RAW);
    }
    if ((field_flags & (1 << BBOX_FIELD_GYRO_FILTER)) && compute_and_check_vec3_delta(&deltas.gyro_filter, &current->gyro_filter, &previous->gyro_filter)) {
      active_fields |= (1 << BBOX_FIELD_GYRO_FILTER);
    }
    if ((field_flags & (1 << BBOX_FIELD_MOTOR)) && compute_and_check_vec4_delta(&deltas.motor, &current->motor, &previous->motor)) {
      active_fields |= (1 << BBOX_FIELD_MOTOR);
    }
    
    // Check CPU load
    if (field_flags & (1 << BBOX_FIELD_CPU_LOAD)) {
      deltas.cpu_load = current->cpu_load - previous->cpu_load;
      if (deltas.cpu_load != 0) {
        active_fields |= (1 << BBOX_FIELD_CPU_LOAD);
      }
    }
    
    // Check debug array
    if (field_flags & (1 << BBOX_FIELD_DEBUG)) {
      for (uint32_t i = 0; i < BLACKBOX_DEBUG_SIZE; i++) {
        deltas.debug[i] = current->debug[i] - previous->debug[i];
      }
      if (debug_has_changed(current->debug, previous->debug, BLACKBOX_DEBUG_SIZE)) {
        active_fields |= (1 << BBOX_FIELD_DEBUG);
      }
    }
    
    encoded_field_flags = active_fields | BLACKBOX_FRAME_TYPE_BIT;
  }
  
  // Encode field flags
  CBOR_CHECK_ERROR(res = cbor_encode_uint32_t(enc, &encoded_field_flags));
  
  // Encode loop and time
  if (frame_type == BLACKBOX_FRAME_I) {
    CBOR_CHECK_ERROR(res = cbor_encode_uint32_t(enc, &current->loop));
    CBOR_CHECK_ERROR(res = cbor_encode_uint32_t(enc, &current->time));
  } else {
    uint32_t loop_delta = current->loop - previous->loop;
    int32_t time_delta = (int32_t)(current->time - previous->time);
    CBOR_CHECK_ERROR(res = cbor_encode_uint32_t(enc, &loop_delta));
    CBOR_CHECK_ERROR(res = cbor_encode_int32_t(enc, &time_delta));
  }

  // Encode all active fields
  const uint32_t active_flags = encoded_field_flags & ~BLACKBOX_FRAME_TYPE_BIT;
  const blackbox_t *source = (frame_type == BLACKBOX_FRAME_I) ? current : &deltas;
  
  // Encode fields if present in active flags
  CBOR_CHECK_ERROR(res = encode_vec3_field_if_present(enc, active_flags, BBOX_FIELD_PID_P_TERM, &source->pid_p_term));
  CBOR_CHECK_ERROR(res = encode_vec3_field_if_present(enc, active_flags, BBOX_FIELD_PID_I_TERM, &source->pid_i_term));
  CBOR_CHECK_ERROR(res = encode_vec3_field_if_present(enc, active_flags, BBOX_FIELD_PID_D_TERM, &source->pid_d_term));
  CBOR_CHECK_ERROR(res = encode_vec4_field_if_present(enc, active_flags, BBOX_FIELD_RX, &source->rx));
  CBOR_CHECK_ERROR(res = encode_vec4_field_if_present(enc, active_flags, BBOX_FIELD_SETPOINT, &source->setpoint));
  CBOR_CHECK_ERROR(res = encode_vec3_field_if_present(enc, active_flags, BBOX_FIELD_ACCEL_RAW, &source->accel_raw));
  CBOR_CHECK_ERROR(res = encode_vec3_field_if_present(enc, active_flags, BBOX_FIELD_ACCEL_FILTER, &source->accel_filter));
  CBOR_CHECK_ERROR(res = encode_vec3_field_if_present(enc, active_flags, BBOX_FIELD_GYRO_RAW, &source->gyro_raw));
  CBOR_CHECK_ERROR(res = encode_vec3_field_if_present(enc, active_flags, BBOX_FIELD_GYRO_FILTER, &source->gyro_filter));
  CBOR_CHECK_ERROR(res = encode_vec4_field_if_present(enc, active_flags, BBOX_FIELD_MOTOR, &source->motor));
  
  // CPU load needs special handling for type differences
  if (active_flags & (1 << BBOX_FIELD_CPU_LOAD)) {
    if (frame_type == BLACKBOX_FRAME_I) {
      CBOR_CHECK_ERROR(res = cbor_encode_uint16_t(enc, &current->cpu_load));
    } else {
      int16_t cpu_delta = (int16_t)deltas.cpu_load;
      CBOR_CHECK_ERROR(res = cbor_encode_int16_t(enc, &cpu_delta));
    }
  }
  
  // Debug array
  if (active_flags & (1 << BBOX_FIELD_DEBUG)) {
    CBOR_CHECK_ERROR(res = cbor_encode_array(enc, BLACKBOX_DEBUG_SIZE));
    for (uint32_t i = 0; i < BLACKBOX_DEBUG_SIZE; i++) {
      CBOR_CHECK_ERROR(res = cbor_encode_int16_t(enc, &source->debug[i]));
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
  if ((profile.blackbox.debug_flags & flag) != flag) {
    return;
  }

  blackbox.debug[index] = data;
}

static uint32_t blackbox_rate_div() {
  if (state.looptime_autodetect <= 0.0f || profile.blackbox.sample_rate_hz == 0) {
    return 1;
  }

  const uint32_t loop_hz = 1000000.0f / state.looptime_autodetect;
  const uint32_t rate = loop_hz / profile.blackbox.sample_rate_hz;
  return max(1U, rate);
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
      blackbox_has_previous = 0;
    }
    return;
  }

  if (blackbox_enabled == 0) {
    return;
  }

  if ((state.loop_counter % blackbox_rate) != 0) {
    return;
  }

  const uint32_t field_flags = profile.blackbox.field_flags;

  blackbox.loop++;
  blackbox.time = time_micros();

  if (BLACKBOX_FIELD_ENABLED(field_flags, BBOX_FIELD_PID_P_TERM)) {
    blackbox_compress_vec3(&blackbox.pid_p_term, &state.pid_p_term);
  }
  if (BLACKBOX_FIELD_ENABLED(field_flags, BBOX_FIELD_PID_I_TERM)) {
    blackbox_compress_vec3(&blackbox.pid_i_term, &state.pid_i_term);
  }
  if (BLACKBOX_FIELD_ENABLED(field_flags, BBOX_FIELD_PID_D_TERM)) {
    blackbox_compress_vec3(&blackbox.pid_d_term, &state.pid_d_term);
  }

  if (BLACKBOX_FIELD_ENABLED(field_flags, BBOX_FIELD_RX)) {
    blackbox_compress_vec4(&blackbox.rx, &state.rx);
  }

  if (BLACKBOX_FIELD_ENABLED(field_flags, BBOX_FIELD_SETPOINT)) {
    blackbox.setpoint.roll = blackbox_compress_float(state.setpoint.roll);
    blackbox.setpoint.pitch = blackbox_compress_float(state.setpoint.pitch);
    blackbox.setpoint.yaw = blackbox_compress_float(state.setpoint.yaw);
    blackbox.setpoint.throttle = blackbox_compress_float(state.throttle);
  }

  if (BLACKBOX_FIELD_ENABLED(field_flags, BBOX_FIELD_GYRO_FILTER)) {
    blackbox_compress_vec3(&blackbox.gyro_filter, &state.gyro);
  }
  if (BLACKBOX_FIELD_ENABLED(field_flags, BBOX_FIELD_GYRO_RAW)) {
    blackbox_compress_vec3(&blackbox.gyro_raw, &state.gyro_raw);
  }

  if (BLACKBOX_FIELD_ENABLED(field_flags, BBOX_FIELD_ACCEL_FILTER)) {
    blackbox_compress_vec3(&blackbox.accel_filter, &state.accel);
  }
  if (BLACKBOX_FIELD_ENABLED(field_flags, BBOX_FIELD_ACCEL_RAW)) {
    blackbox_compress_vec3(&blackbox.accel_raw, &state.accel_raw);
  }

  if (BLACKBOX_FIELD_ENABLED(field_flags, BBOX_FIELD_MOTOR)) {
    blackbox_compress_vec4(&blackbox.motor, &state.motor_mix);
  }

  if (BLACKBOX_FIELD_ENABLED(field_flags, BBOX_FIELD_CPU_LOAD)) {
    blackbox.cpu_load = state.cpu_load;
  }

  // Determine frame type based on blackbox.loop counter
  // First frame (loop == 1) is always an I-frame, then every BLACKBOX_I_FRAME_INTERVAL frames
  blackbox_frame_type_t frame_type = (!blackbox_has_previous || blackbox.loop % BLACKBOX_I_FRAME_INTERVAL == 0) ?
                                     BLACKBOX_FRAME_I : BLACKBOX_FRAME_P;
  
  // Write the frame using I-frame/P-frame encoding
  if (blackbox_device_write_frame(field_flags, &blackbox, &blackbox_previous, frame_type)) {
    // Store only frames that were queued so P-frame deltas stay decodable after drops.
    blackbox_previous = blackbox;
    blackbox_has_previous = 1;
  }
}
#else
void blackbox_init() {}
void blackbox_set_debug(blackbox_debug_flag_t flag, uint8_t index, int16_t data) {}
void blackbox_update() {}
#endif
