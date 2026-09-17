#include "driver/gyro/gyro.h"

#include "core/project.h"
#include "driver/exti.h"
#include "driver/interrupt.h"
#include "driver/spi.h"
#include "driver/time.h"

#include "driver/gyro/bmi270.h"
#include "driver/gyro/bmi323.h"
#include "driver/gyro/icm42605.h"
#include "driver/gyro/lsm6dso.h"
#include "driver/gyro/lsm6dsv16x.h"
#include "driver/gyro/mpu6xxx.h"

#ifdef USE_GYRO
gyro_types_t gyro_type = GYRO_TYPE_INVALID;
spi_bus_device_t gyro_bus = {};
#endif

// EXTI owns this clock except initialization and the masked calibration reset.
// SPI reads remain in Flight.
static struct {
  gyro_clock_t timing;
  uint32_t nominal_period;
  uint32_t rate_start;
  uint8_t count;
  bool mpu6000;
  bool phase_valid;
} gyro_clock;

gyro_clock_t gyro_clock_snapshot() {
  gyro_clock_t timing;
  ATOMIC_BLOCK_ALL {
    timing = gyro_clock.timing;
  }
  return timing;
}

#if defined(USE_GYRO) || defined(PIO_UNIT_TESTING)
static void gyro_clock_reset(uint32_t period, bool mpu6000) {
  gyro_clock = {};
  gyro_clock.nominal_period = period;
  gyro_clock.mpu6000 = mpu6000;
}

static void gyro_clock_update(uint32_t sample) {
  const uint32_t interval = sample - gyro_clock.timing.sample;
  const uint32_t nominal = gyro_clock.nominal_period;
  if (gyro_clock.count == 0 || interval < nominal / 2 || interval > nominal * 3 / 2) {
    gyro_clock_reset(nominal, gyro_clock.mpu6000);
    gyro_clock.rate_start = sample;
    gyro_clock.count = 1;
    gyro_clock.timing.phase = sample;
  } else {
    // MPU6000's delayed eighth edge precedes the short interval. Lock to that
    // late phase, not the alternating intervals, so every read sees new data.
    if (!gyro_clock.mpu6000 || interval < US_TO_CYCLES(85)) {
      gyro_clock.timing.phase = gyro_clock.timing.sample;
      gyro_clock.phase_valid = true;
    }
    if (++gyro_clock.count == 65) {
      if (gyro_clock.phase_valid)
        gyro_clock.timing.period = (sample - gyro_clock.rate_start + 32) / 64;
      gyro_clock.rate_start = sample;
      gyro_clock.count = 1;
    }
  }
  gyro_clock.timing.sample = sample;
}

#ifdef PIO_UNIT_TESTING
void gyro_test_clock_reset(uint32_t period, bool mpu6000) { gyro_clock_reset(period, mpu6000); }
void gyro_test_clock_update(uint32_t sample) { gyro_clock_update(sample); }
#endif
#endif

#ifdef USE_GYRO

static bool gyro_exti_conflicts(gpio_pins_t pin) {
  return pin != PIN_NONE && gpio_pin_defs[pin].pin_index == gpio_pin_defs[target.gyro.exti].pin_index;
}

static gyro_types_t gyro_spi_detect() {
  gyro_types_t type = GYRO_TYPE_INVALID;

  switch (type) {
  case GYRO_TYPE_INVALID:
    // FALLTHROUGH

  case GYRO_TYPE_MPU6000:
  case GYRO_TYPE_MPU6500:
  case GYRO_TYPE_ICM20601:
  case GYRO_TYPE_ICM20602:
  case GYRO_TYPE_ICM20608:
  case GYRO_TYPE_ICM20689:
    type = mpu6xxx_detect();
    if (type != GYRO_TYPE_INVALID) {
      break;
    }
    // FALLTHROUGH

  case GYRO_TYPE_ICM42605:
  case GYRO_TYPE_ICM42688P:
  case GYRO_TYPE_ICM42622P:
  case GYRO_TYPE_ICM42686P:
    type = icm42605_detect();
    if (type != GYRO_TYPE_INVALID) {
      break;
    }
    // FALLTHROUGH

  case GYRO_TYPE_LSM6DSO:
    type = lsm6dso_detect();
    if (type != GYRO_TYPE_INVALID) {
      break;
    }
    // FALLTHROUGH

  case GYRO_TYPE_LSM6DSV16X:
  case GYRO_TYPE_LSM6DSK320X:
    type = lsm6dsv16x_detect();
    if (type != GYRO_TYPE_INVALID) {
      break;
    }
    // FALLTHROUGH

  case GYRO_TYPE_BMI270:
    type = bmi270_detect();
    if (type != GYRO_TYPE_INVALID) {
      break;
    }
    // FALLTHROUGH
  case GYRO_TYPE_BMI323:
    type = bmi323_detect();
    if (type != GYRO_TYPE_INVALID) {
      break;
    }
    // FALLTHROUGH
  default:
    break;
  }

  return type;
}

gyro_types_t gyro_init() {
  if (!target_gyro_spi_device_valid(&target.gyro)) {
    return GYRO_TYPE_INVALID;
  }

  if (target.gyro.exti != PIN_NONE) {
    gpio_config_t gpio_init;
    gpio_init.mode = GPIO_INPUT;
    gpio_init.output = GPIO_OPENDRAIN;
    gpio_init.pull = GPIO_NO_PULL;
    gpio_init.drive = GPIO_DRIVE_HIGH;
    gpio_pin_init(target.gyro.exti, gpio_init);
  }

  gyro_bus.port = target.gyro.port;
  gyro_bus.nss = target.gyro.nss;
  spi_bus_device_init(&gyro_bus);

  gyro_type = gyro_spi_detect();

  switch (gyro_type) {
  case GYRO_TYPE_MPU6000:
  case GYRO_TYPE_MPU6500:
  case GYRO_TYPE_ICM20601:
  case GYRO_TYPE_ICM20602:
  case GYRO_TYPE_ICM20608:
  case GYRO_TYPE_ICM20689:
    mpu6xxx_configure();
    break;

  case GYRO_TYPE_ICM42605:
  case GYRO_TYPE_ICM42688P:
  case GYRO_TYPE_ICM42622P:
  case GYRO_TYPE_ICM42686P:
    icm42605_configure();
    break;

  case GYRO_TYPE_LSM6DSO:
    lsm6dso_configure();
    break;

  case GYRO_TYPE_LSM6DSV16X:
  case GYRO_TYPE_LSM6DSK320X:
    lsm6dsv16x_configure();
    break;

  case GYRO_TYPE_BMI270:
    bmi270_configure();
    break;
  case GYRO_TYPE_BMI323:
    bmi323_configure();
    break;

  default:
    break;
  }

  gyro_clock_reset(US_TO_CYCLES(gyro_update_period()), gyro_type == GYRO_TYPE_MPU6000);
  if (gyro_type != GYRO_TYPE_INVALID && target.gyro.exti != PIN_NONE &&
      !gyro_exti_conflicts(target.rx_spi.exti) &&
      !(target.rx_spi.busy_exti && gyro_exti_conflicts(target.rx_spi.busy))) {
    exti_enable(target.gyro.exti, EXTI_TRIG_RISING);
  }

  return gyro_type;
}

void gyro_handle_exti() {
  gyro_clock_update(time_cycles());
}

bool gyro_exti_state() {
  if (target.gyro.exti == PIN_NONE) {
    return true;
  }
  return gpio_pin_read(target.gyro.exti);
}

float gyro_update_period() {
  switch (gyro_type) {
  case GYRO_TYPE_MPU6000:
  case GYRO_TYPE_MPU6500:
  case GYRO_TYPE_ICM20601:
  case GYRO_TYPE_ICM20602:
  case GYRO_TYPE_ICM20608:
  case GYRO_TYPE_ICM20689:
    return 125.0f;

  case GYRO_TYPE_ICM42605:
  case GYRO_TYPE_ICM42688P:
  case GYRO_TYPE_ICM42622P:
  case GYRO_TYPE_ICM42686P:
    return 125.0f;

  case GYRO_TYPE_LSM6DSO:
    return 150.06f;

  case GYRO_TYPE_LSM6DSV16X:
  case GYRO_TYPE_LSM6DSK320X:
    return 125.0f;

  case GYRO_TYPE_BMI270:
  case GYRO_TYPE_BMI323:
    return 312.5f;

  default:
    return 250.0f;
  }
}

gyro_data_t gyro_read() {
  gyro_data_t data = {};

  switch (gyro_type) {
  case GYRO_TYPE_MPU6000:
  case GYRO_TYPE_MPU6500:
  case GYRO_TYPE_ICM20601:
  case GYRO_TYPE_ICM20602:
  case GYRO_TYPE_ICM20608:
  case GYRO_TYPE_ICM20689: {
    mpu6xxx_read_gyro_data(&data);
    break;
  }

  case GYRO_TYPE_ICM42605:
  case GYRO_TYPE_ICM42688P:
  case GYRO_TYPE_ICM42622P:
  case GYRO_TYPE_ICM42686P: {
    icm42605_read_gyro_data(&data);
    break;
  }

  case GYRO_TYPE_LSM6DSO: {
    lsm6dso_read_gyro_data(&data);
    break;
  }

  case GYRO_TYPE_LSM6DSV16X:
  case GYRO_TYPE_LSM6DSK320X: {
    lsm6dsv16x_read_gyro_data(&data);
    break;
  }

  case GYRO_TYPE_BMI270: {
    bmi270_read_gyro_data(&data);
    break;
  }
  case GYRO_TYPE_BMI323: {
    bmi323_read_gyro_data(&data);
    break;
  }

  default:
    break;
  }

  return data;
}

void gyro_calibrate() {
  switch (gyro_type) {
  case GYRO_TYPE_BMI270: {
    bmi270_calibrate();
    ATOMIC_BLOCK(EXTI_PRIORITY) {
      gyro_clock_reset(US_TO_CYCLES(gyro_update_period()), false);
    }
    break;
  }

  default:
    break;
  }
}
#else
float gyro_update_period() {
  return 250.0f;
}
#endif
