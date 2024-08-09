#include "driver/gyro/gyro.h"

#include "control/control.h"
#include "core/failloop.h"
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

// Written only by the DMA completion ISR, except while acquisition is stopped.
static uint32_t sample_period;
static struct {
  uint32_t rate_start;
  uint32_t read_duration;
  uint8_t count;
} gyro_clock;

static void gyro_timing_reset(uint32_t period) {
  sample_period = period;
  gyro_clock = {};
  state.gyro_period_cycles = 0;
  state.gyro_phase_cycles = 0;
  state.gyro_sample_cycles = 0;
}

static void gyro_timing_update(uint32_t sample, uint32_t completed, bool mpu6000) {
  const uint32_t interval = sample - state.gyro_sample_cycles;
  const uint32_t duration = completed - sample;
  if (gyro_clock.count == 0 || interval < sample_period / 2 ||
      interval > sample_period * 3 / 2 || duration > sample_period / 2) {
    // Reacquire after missing samples instead of measuring the gap as a lower ODR.
    gyro_timing_reset(sample_period);
    gyro_clock.rate_start = sample;
    gyro_clock.count = 1;
    state.gyro_phase_cycles = sample;
  } else {
    // The edge before MPU6000's short interval is its delayed eighth sample.
    // Other gyros can use every edge as the phase reference.
    if (!mpu6000 || interval < US_TO_CYCLES(85)) {
      state.gyro_phase_cycles = state.gyro_sample_cycles + gyro_clock.read_duration;
    }
    if (++gyro_clock.count == 65) {
      state.gyro_period_cycles = (sample - gyro_clock.rate_start + 32) / 64;
      gyro_clock.rate_start = sample;
      gyro_clock.count = 1;
    }
  }
  if (duration <= sample_period / 2 && duration > gyro_clock.read_duration) {
    state.gyro_phase_cycles += duration - gyro_clock.read_duration;
    gyro_clock.read_duration = duration;
  }
  state.gyro_sample_cycles = sample;
}

#ifdef PIO_UNIT_TESTING
void gyro_test_timing_reset(uint32_t period) { gyro_timing_reset(period); }
void gyro_test_timing_update(uint32_t sample, uint32_t completed, bool mpu6000) {
  gyro_timing_update(sample, completed, mpu6000);
}
#endif

#ifdef USE_GYRO

gyro_types_t gyro_type = GYRO_TYPE_INVALID;
spi_bus_device_t gyro_bus = {};
uint8_t gyro_buf[32];

// Probe in order; each driver owns its operations and timing requirements.
static constexpr const gyro_device_t *GYRO_DEVICES[] = {
    &gyro_device_mpu6xxx,
    &gyro_device_icm42605,
    &gyro_device_lsm6dso,
    &gyro_device_lsm6dsv16x,
    &gyro_device_bmi270,
    &gyro_device_bmi323,
};

static const gyro_device_t *device;
static volatile bool exti_enabled;
static volatile bool read_pending;
static uint32_t read_started;
static uint32_t last_completed_us;
static gyro_data_t completed_sample;

static bool gyro_exti_conflicts(gpio_pins_t pin) {
  return pin != PIN_NONE && gpio_pin_defs[pin].pin_index == gpio_pin_defs[target.gyro.exti].pin_index;
}

gyro_types_t gyro_init() {
  if (!target_gyro_spi_device_valid(&target.gyro)) {
    return GYRO_TYPE_INVALID;
  }

  gyro_bus.port = target.gyro.port;
  gyro_bus.nss = target.gyro.nss;
  spi_bus_device_init(&gyro_bus);

  for (const auto *candidate : GYRO_DEVICES) {
    gyro_type = candidate->detect();
    if (gyro_type != GYRO_TYPE_INVALID) {
      device = candidate;
      break;
    }
  }
  if (!device)
    return GYRO_TYPE_INVALID;

  device->configure();
  gyro_timing_reset(US_TO_CYCLES(device->period_us));
  gyro_read(); // Prime the existing read pipeline before enabling interrupts.
  spi_txn_wait(&gyro_bus);
  if (device->decode)
    device->decode(&completed_sample);
  last_completed_us = time_micros();

  if (device->start_read && target.gyro.exti != PIN_NONE &&
      !gyro_exti_conflicts(target.rx_spi.exti) &&
      !(target.rx_spi.busy_exti && gyro_exti_conflicts(target.rx_spi.busy))) {
    gpio_config_t gpio_init;
    gpio_init.mode = GPIO_INPUT;
    gpio_init.output = GPIO_OPENDRAIN;
    gpio_init.pull = GPIO_NO_PULL;
    gpio_init.drive = GPIO_DRIVE_HIGH;
    gpio_pin_init(target.gyro.exti, gpio_init);
    exti_enabled = true;
    exti_enable(target.gyro.exti, EXTI_TRIG_RISING);
  }

  return gyro_type;
}

float gyro_update_period() {
  return device ? device->period_us : 250.0f;
}

static void gyro_set_ready(void *arg) {
  const uint32_t completed = time_cycles();
  device->decode(&completed_sample);
  gyro_timing_update(read_started, completed, gyro_type == GYRO_TYPE_MPU6000);
  last_completed_us = time_micros();
  read_pending = false;
}

gyro_data_t gyro_read() {
  static gyro_data_t data;

  if (exti_enabled) {
    // A queued transfer may have been deferred by a shared bus or F4 DMA2 use.
    spi_txn_continue(&gyro_bus);
    bool stalled;
    // DMA publishes the sample; EXTI may start another read during fallback.
    ATOMIC_BLOCK(DMA_PRIORITY) {
      data = completed_sample;
      if (time_micros() - last_completed_us <= 2000) {
        return data;
      }
      stalled = read_pending;
      exti_enabled = false;
      exti_disable(target.gyro.exti);
      state.gyro_period_cycles = 0;
    }
    if (stalled)
      failloop(FAILLOOP_GYRO);
  }

  if (device)
    device->read(&data);
  return data;
}

void gyro_handle_exti() {
  if (!exti_enabled || read_pending)
    return;

  // Keep the pool check and submission together.
  // Mask DMA and lower priorities that can submit to the shared SPI queue.
  ATOMIC_BLOCK(DMA_PRIORITY) {
    if (!spi_txn_ready(&gyro_bus) || !spi_txn_has_free()) {
      return;
    }
    read_started = time_cycles();
    read_pending = true;
    device->start_read(gyro_set_ready);
  }
}

void gyro_calibrate() {
  if (gyro_type != GYRO_TYPE_BMI270)
    return;

  const bool resume_exti = exti_enabled;
  // Stop new reads, then drain DMA before resetting the sensor or timing state.
  exti_enabled = false;
  if (resume_exti)
    exti_disable(target.gyro.exti);
  spi_txn_wait(&gyro_bus);
  bmi270_calibrate();
  device->read(&completed_sample);
  spi_txn_wait(&gyro_bus);
  device->decode(&completed_sample);
  gyro_timing_reset(US_TO_CYCLES(device->period_us));
  last_completed_us = time_micros();
  if (resume_exti) {
    exti_enabled = true;
    exti_enable(target.gyro.exti, EXTI_TRIG_RISING);
  }
}
#else
float gyro_update_period() {
  return 250.0f;
}
#endif
