#include "io/blackbox.h"

#include "driver/time.h"
#include "flight/control.h"
#include "io/blackbox_device.h"
#include "io/usb_configurator.h"
#include "util/cbor_helper.h"
#include "util/util.h"

#ifdef USE_BLACKBOX

#define BLACKBOX_I_FRAME_INTERVAL 32  // Every 32nd frame is an I-frame

static blackbox_t blackbox;
static blackbox_t blackbox_previous;  // Store previous frame for delta encoding
static uint8_t blackbox_enabled = 0;
static uint8_t blackbox_rate = 0;

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

  // Determine frame type based on blackbox.loop counter
  // First frame (loop == 1) is always an I-frame, then every BLACKBOX_I_FRAME_INTERVAL frames
  blackbox_frame_type_t frame_type = (blackbox.loop == 1 || blackbox.loop % BLACKBOX_I_FRAME_INTERVAL == 0) ? 
                                     BLACKBOX_FRAME_I : BLACKBOX_FRAME_P;
  
  // Write the frame using I-frame/P-frame encoding
  blackbox_device_write_frame(profile.blackbox.field_flags, &blackbox, &blackbox_previous, frame_type);
  
  // Store current frame as previous for next P-frame
  blackbox_previous = blackbox;
}
#else
void blackbox_init() {}
void blackbox_set_debug(blackbox_debug_flag_t flag, uint8_t index, int16_t data) {}
void blackbox_update() {}
#endif