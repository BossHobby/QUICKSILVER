#include "driver/servo.h"
#include "core/profile.h"
#include "core/project.h"

#ifdef USE_SERVO

static float servo_values[SERVO_MAX];

void servo_set(servo_channel_t index, float value) {
  if (index >= SERVO_MAX) {
    return;
  }
  servo_values[index] = value;
}

void servo_write() {
  servo_pwm_write(servo_values);
}

void servo_init() {
  gpio_pins_t pins[SERVO_MAX] = {PIN_NONE};
  pins[SERVO_STEERING] = target.motor_pins[profile.rover.servo_index];
  pins[SERVO_DRIVE] = target.motor_pins[profile.rover.motor_index];
  servo_pwm_init(pins, profile.rover.servo_pwm_hz);
}

void servo_stop() {
  servo_pwm_stop();
}

#endif
