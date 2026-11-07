#include "driver/motor_dshot.h"

#include "control/control.h"
#include "core/profile.h"
#include "core/project.h"
#include "driver/time.h"
#include "util/util.h"

#ifdef USE_MOTOR_DSHOT

typedef enum {
  DIR_CHANGE_START,
  DIR_CHANGE_DELAY,
  DIR_CHANGE_CMD,
  DIR_CHANGE_STOP,
} dir_change_state_t;

static uint16_t dshot_packet[MOTOR_PIN_MAX];
// Outputs whose pins motor_dshot_init() configured.
static uint32_t dshot_motor_mask = 0;

static motor_direction_t motor_dir = MOTOR_FORWARD;
static bool dir_change_done = true;

static bool dshot_motor_slot_active(uint32_t slot) {
  if (slot >= MOTOR_PIN_MAX) {
    return false;
  }
  return target.outputs[slot].pin != PIN_NONE && (target.outputs[slot].caps & OUTPUT_CAP_DSHOT) != 0 &&
         profile_output_slot_uses_protocol(slot, OUTPUT_PROTOCOL_DSHOT);
}

// Configured outputs the profile still assigns to DShot. A USB profile change
// can move an output to another protocol without re-initializing the motors.
static uint32_t dshot_active_mask() {
  return dshot_motor_mask & profile_output_protocol_slots(OUTPUT_PROTOCOL_DSHOT);
}

static void dshot_make_packet(uint8_t number, uint16_t value, bool telemetry) {
  const uint16_t packet = (value << 1) | (telemetry ? 1 : 0);

  uint16_t csum = 0;
  uint16_t csum_data = packet;
  for (uint8_t i = 0; i < 3; ++i) {
    csum ^= csum_data; // xor data by nibbles
    csum_data >>= 4;
  }

  if (profile.motor.dshot_telemetry) {
    csum = ~csum;
  }
  csum &= 0xf;

  dshot_packet[number] = (packet << 4) | csum;
}

static void dshot_make_packet_all(uint16_t value, bool telemetry) {
  const uint32_t active = dshot_active_mask();
  for (uint32_t i = 0; i < MOTOR_PIN_MAX; i++) {
    if (active & (1U << i))
      dshot_make_packet(i, value, telemetry);
  }
}

// Sends dshot_packet to the active outputs. dshot_packet is left intact;
// every caller builds the packets for a frame before starting it.
static void dshot_start() {
  dshot_send_frame(dshot_packet, dshot_active_mask());
}

void motor_dshot_wait_for_ready() {
  while (!dshot_frame_done())
    __NOP();
}

static void dshot_send_packet(uint8_t index, uint16_t value, bool telemetry) {
  motor_dshot_wait_for_ready();

  const uint32_t active = dshot_active_mask();
  for (uint32_t i = 0; i < MOTOR_PIN_MAX; i++) {
    if (active & (1U << i))
      dshot_make_packet(i, i == index ? value : DSHOT_CMD_MOTOR_STOP, telemetry && i == index);
  }

  dshot_start();
}

COLD_CODE static void dshot_handle_dir_change() {
  static uint8_t counter = 0;
  static uint32_t dir_change_time = 0;
  static dir_change_state_t state = DIR_CHANGE_START;

  switch (state) {
  case DIR_CHANGE_START: {
    if (counter < 100) {
      dshot_make_packet_all(0, false);
      dshot_start();
      counter++;
    } else {
      state = DIR_CHANGE_DELAY;
      counter = 0;
    }
    dir_change_time = time_micros();
    break;
  }

  case DIR_CHANGE_DELAY:
    if ((time_micros() - dir_change_time) < DSHOT_DIR_CHANGE_IDLE_TIME_US) {
      break;
    }
    state = DIR_CHANGE_CMD;
    dir_change_time = time_micros();
    break;

  case DIR_CHANGE_CMD: {
    const uint16_t value = motor_dir == MOTOR_REVERSE ? DSHOT_CMD_SPIN_DIRECTION_REVERSED : DSHOT_CMD_SPIN_DIRECTION_NORMAL;
    dshot_make_packet_all(value, true);
    dshot_start();
    if ((time_micros() - dir_change_time) >= DSHOT_DIR_CHANGE_CMD_TIME_US) {
      state = DIR_CHANGE_STOP;
    }
    break;
  }

  case DIR_CHANGE_STOP:
    dir_change_done = true;
    state = DIR_CHANGE_START;
    break;
  }
}

static uint16_t dshot_encode_motor_value(float value) {
#ifdef VEHICLE_ROVER
  if (profile.rover.reversible) {
    if (value == MOTOR_OFF) {
      return DSHOT_CMD_MOTOR_STOP;
    }
    if (value > 0.0f) {
      const float pwm = constrain(value, 0.0f, 1.0f);
      return mapf(pwm, 0.0f, 1.0f, DSHOT_3D_FORWARD_MIN, DSHOT_3D_FORWARD_MAX);
    }
    if (value < 0.0f) {
      const float pwm = constrain(value, -1.0f, 0.0f);
      return mapf(pwm, -1.0f, 0.0f, DSHOT_3D_REVERSE_MIN, DSHOT_3D_REVERSE_MAX);
    }
    return DSHOT_CMD_MOTOR_STOP;
  }
#endif

  if (value >= 0.0f) {
    const float pwm = constrain(value, 0.0f, 1.0f);
    return mapf(pwm, 0.0f, 1.0f, DSHOT_THROTTLE_MIN, DSHOT_THROTTLE_MAX);
  }

  return DSHOT_CMD_MOTOR_STOP;
}

void motor_dshot_init() {
  dshot_motor_mask = 0;
  motor_dir = MOTOR_FORWARD;
  for (uint32_t i = 0; i < MOTOR_PIN_MAX; i++) {
    if (dshot_motor_slot_active(i))
      dshot_motor_mask |= 1U << i;
  }
  dshot_init_outputs(dshot_motor_mask);
}

void motor_dshot_write(float *values) {
  if (!dir_change_done) {
    return dshot_handle_dir_change();
  }

  const uint32_t active = dshot_active_mask();
  for (uint32_t i = 0; i < MOTOR_PIN_MAX; i++) {
    if (active & (1U << i))
      dshot_make_packet(i, dshot_encode_motor_value(values[i]), false);
  }

  dshot_start();
}

void motor_dshot_set_direction(motor_direction_t dir) {
  if (dir_change_done) {
    motor_dir = dir;
    dir_change_done = false;
  }
}

bool motor_dshot_direction_change_done() {
  return dir_change_done;
}

bool motor_dshot_configure_direction(uint8_t index, motor_direction_t dir) {
  if (!dir_change_done || index >= MOTOR_PIN_MAX || !(dshot_active_mask() & (1U << index))) {
    return false;
  }

  for (uint8_t i = 0; i < 100; i++) {
    dshot_send_packet(index, DSHOT_CMD_MOTOR_STOP, false);
  }

  time_delay_us(DSHOT_DIR_CHANGE_IDLE_TIME_US);

  // 7/8 configure the base direction; 20/21 temporarily reverse it for turtle mode.
  const uint16_t command = dir == MOTOR_REVERSE ? DSHOT_CMD_SPIN_DIRECTION_2 : DSHOT_CMD_SPIN_DIRECTION_1;
  for (uint8_t i = 0; i < 10; i++) {
    dshot_send_packet(index, command, true);
    time_delay_us(DSHOT_DIR_CHANGE_CMD_TIME_US);
  }

  for (uint8_t i = 0; i < 10; i++) {
    dshot_send_packet(index, DSHOT_CMD_SAVE_SETTINGS, true);
    time_delay_us(DSHOT_DIR_CHANGE_CMD_TIME_US);
  }

  // Allow the ESC to finish its nonvolatile write before sending more packets.
  time_delay_ms(35);

  dshot_send_packet(index, DSHOT_CMD_MOTOR_STOP, false);
  motor_dshot_wait_for_ready();

  return true;
}

void motor_dshot_beep() {
  static uint32_t last_time = 0;
  const uint32_t time = time_millis() - last_time;

  static uint8_t beep_command = DSHOT_CMD_BEEP1;
  dshot_make_packet_all(beep_command, true);
  dshot_start();

  if (time >= 500) {
    beep_command++;
    if (beep_command > DSHOT_CMD_BEEP5) {
      beep_command = DSHOT_CMD_BEEP1;
    }

    last_time = time_millis();
  }
}
#endif
