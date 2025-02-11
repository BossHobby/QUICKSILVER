#include "serial_esc.h"

#include "core/profile.h"
#include "core/target.h"
#include "driver/motor.h"
#include "driver/usb.h"

#define START_BIT_TIMEOUT_MS 2

#define BIT_TIME(baud) (SYS_CLOCK_FREQ_HZ / (baud))
#define BIT_TIME_HALF(baud) (BIT_TIME(baud) / 2)
#define BIT_TIME_3_4(baud) (BIT_TIME_HALF(baud) + (BIT_TIME_HALF(baud) / 2))

#define START_BIT_TIME(baud) (BIT_TIME_3_4(baud))
#define STOP_BIT_TIME(baud) ((BIT_TIME(baud) * 9) + BIT_TIME_HALF(baud))

void serial_esc_set_input(gpio_pins_t pin) {
  gpio_config_t gpio_init = gpio_config_default();
  gpio_init.mode = GPIO_INPUT;
  gpio_init.output = GPIO_OPENDRAIN;
  gpio_init.pull = GPIO_UP_PULL;
  gpio_init.drive = GPIO_DRIVE_NORMAL;
  gpio_pin_init(pin, gpio_init);
}

void serial_esc_set_output(gpio_pins_t pin) {
  gpio_config_t gpio_init = gpio_config_default();
  gpio_init.mode = GPIO_OUTPUT;
  gpio_init.output = GPIO_PUSHPULL;
  gpio_init.pull = GPIO_NO_PULL;
  gpio_init.drive = GPIO_DRIVE_NORMAL;
  gpio_pin_init(pin, gpio_init);
}

bool serial_esc_read(gpio_pins_t pin, uint32_t baud, uint8_t *bt) {
  uint32_t start_time = time_millis();

  while (serial_esc_is_high(pin)) {
    // check for startbit begin
    if (time_millis() - start_time > START_BIT_TIMEOUT_MS) {
      return false;
    }
  }

  // start bit
  const uint32_t bit_time = BIT_TIME(baud);
  uint32_t btime = START_BIT_TIME(baud);
  uint16_t bitmask = 0;
  uint8_t bit = 0;

  start_time = time_cycles();
  while (time_cycles() - start_time < btime)
    ;

  while (1) {
    if (serial_esc_is_high(pin)) {
      bitmask |= (1 << bit);
    }
    btime = btime + bit_time;
    bit++;

    if (bit == 10)
      break;

    while (time_cycles() - start_time < btime)
      ;
  }

  // check start bit and stop bit
  if ((bitmask & 1) || (!(bitmask & (1 << 9)))) {
    return false;
  }

  *bt = bitmask >> 1;

  return true;
}

void serial_esc_write(gpio_pins_t pin, uint32_t baud, uint8_t data) {
  const uint32_t bit_time = BIT_TIME(baud);

  // shift out stopbit first
  uint16_t bitmask = (data << 2) | 1 | (1 << 10);
  while (1) {
    const uint32_t start_time = time_cycles();
    if (bitmask & 1) {
      serial_esc_set_high(pin);
    } else {
      serial_esc_set_low(pin);
    }

    bitmask = (bitmask >> 1);

    if (bitmask == 0)
      break; // stopbit shifted out - but don't wait

    while (time_cycles() - start_time < bit_time)
      ;
  }
}

void serial_esc_process(uint8_t index, uint32_t baud) {
  const gpio_pins_t pin = target.motor_pins[profile.motor.motor_pins[index]];

  motor_set_all(MOTOR_OFF);
  motor_wait_for_ready();

  time_delay_ms(10);

  bool is_output = true;
  serial_esc_set_output(pin);
  serial_esc_set_high(pin);

  while (true) {
    uint8_t byte = 0;

    is_output = false;
    serial_esc_set_input(pin);
    while (serial_esc_read(pin, baud, &byte))
      usb_serial_write(&byte, 1);

    while (usb_serial_read(&byte, 1)) {
      if (!is_output) {
        serial_esc_set_output(pin);
        is_output = true;
      }
      serial_esc_write(pin, baud, byte);
    }
  }
}