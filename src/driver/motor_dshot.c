#include "driver/motor_dshot.h"

#include "core/profile.h"
#include "core/project.h"
#include "driver/time.h"
#include "flight/control.h"
#include "util/util.h"

#ifdef USE_MOTOR_DSHOT

typedef enum {
  DIR_CHANGE_START,
  DIR_CHANGE_DELAY,
  DIR_CHANGE_CMD,
  DIR_CHANGE_STOP,
} dir_change_state_t;

uint16_t dshot_packet[MOTOR_PIN_MAX]; // 16bits dshot data for 4 motors
uint32_t pwm_failsafe_time = 1;
motor_direction_t motor_dir = MOTOR_FORWARD;

static bool dir_change_done = true;

extern void dshot_dma_start();

void dshot_make_packet(uint8_t number, uint16_t value, bool telemetry) {
  const uint16_t packet = (value << 1) | (telemetry ? 1 : 0);

  uint16_t csum = 0;
  uint16_t csum_data = packet;
  for (uint8_t i = 0; i < 3; ++i) {
    csum ^= csum_data; // xor data by nibbles
    csum_data >>= 4;
  }

  dshot_packet[number] = (packet << 4) | (csum & 0xf);
}

void dshot_make_packet_all(uint16_t value, bool telemetry) {
  for (uint32_t i = 0; i < MOTOR_PIN_MAX; i++) {
    dshot_make_packet(profile.motor.motor_pins[i], value, telemetry);
  }
}

void motor_dshot_write(float *values) {
  if (dir_change_done) {
    for (uint32_t i = 0; i < MOTOR_PIN_MAX; i++) {
      uint16_t value = 0;

      if (values[i] >= 0.0f) {
        const float pwm = constrainf(values[i], 0.0f, 1.0f);
        value = mapf(pwm, 0.0f, 1.0f, 48, 2047);
      } else {
        value = 0;
      }

      if (flags.failsafe && !flags.motortest_override) {
        if (!pwm_failsafe_time) {
          pwm_failsafe_time = time_micros();
        } else if (time_micros() - pwm_failsafe_time > 4000000) {
          // 1s after failsafe we turn off the signal for safety
          // this means the escs won't rearm correctly after 2 secs of signal lost
          // usually the quad should be gone by then
          value = 0;
        }
      } else {
        pwm_failsafe_time = 0;
      }

      dshot_make_packet(profile.motor.motor_pins[i], value, false);
    }

    dshot_dma_start();
  } else {
    static uint8_t counter = 0;
    static uint32_t dir_change_time = 0;
    static dir_change_state_t state = DIR_CHANGE_START;

    switch (state) {
    case DIR_CHANGE_START: {
      if (counter < 100) {
        dshot_make_packet_all(0, false);
        dshot_dma_start();
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
      if ((time_micros() - dir_change_time) < DSHOT_DIR_CHANGE_CMD_TIME_US) {
        break;
      }

      const uint16_t value = motor_dir == MOTOR_REVERSE ? DSHOT_CMD_ROTATE_REVERSE : DSHOT_CMD_ROTATE_NORMAL;
      if (counter < 10) {
        dshot_make_packet_all(value, true);
        dshot_dma_start();
        counter++;
      } else {
        state = DIR_CHANGE_STOP;
        counter = 0;
      }
      dir_change_time = time_micros();
      break;
    }

    case DIR_CHANGE_STOP:
      dir_change_done = true;
      state = DIR_CHANGE_START;
      break;
    }
  }
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

void motor_dshot_beep() {
  static uint32_t last_time = 0;
  const uint32_t time = time_millis() - last_time;

  static uint8_t beep_command = DSHOT_CMD_BEEP1;
  dshot_make_packet_all(beep_command, true);
  dshot_dma_start();

  if (time >= 500) {
    beep_command++;
    if (beep_command > DSHOT_CMD_BEEP5) {
      beep_command = DSHOT_CMD_BEEP1;
    }

    last_time = time_millis();
  }
}
#endif