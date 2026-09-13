#include <math.h>
#include <stdint.h>
#include <stdio.h>
#include <string.h>

#include "config/feature.h"

#include "control/control.h"
#ifdef VEHICLE_ROVER
#include "control/rover/control.h"
#else
#include "control/pid.h"
#endif
#include "control/imu.h"
#include "control/sixaxis.h"
#include "core/debug.h"
#include "core/failloop.h"
#include "core/flash.h"
#include "core/profile.h"
#include "core/project.h"
#include "core/scheduler.h"
#include "core/tasks.h"
#include "core/target.h"
#include "driver/adc.h"
#include "driver/baro/baro.h"
#include "driver/gpio.h"
#include "driver/interrupt.h"
#include "driver/motor.h"
#include "driver/rgb_led.h"
#include "driver/servo.h"
#include "driver/time.h"
#include "driver/timer.h"
#include "driver/usb.h"
#include "io/blackbox.h"
#include "io/buzzer.h"
#include "io/gps.h"
#include "io/led.h"
#include "io/simulator.h"
#include "io/usb_configurator.h"
#include "io/vbat.h"
#include "io/vtx.h"
#include "osd/render.h"
#include "util/filter.h"

static StaticSemaphore_t usb_mutex_storage;
SemaphoreHandle_t usb_configurator_mutex;

extern "C" __attribute__((__used__)) void
memory_section_init() {
#ifdef USE_FAST_RAM
  extern uint8_t _fast_ram_start;
  extern uint8_t _fast_ram_end;
  extern uint8_t _fast_ram_data;
  memcpy(&_fast_ram_start, &_fast_ram_data, (size_t)(&_fast_ram_end - &_fast_ram_start));
#endif
#ifdef USE_DMA_RAM
  extern uint8_t _dma_ram_start;
  extern uint8_t _dma_ram_end;
  extern uint8_t _dma_ram_data;

#ifdef STM32H7
  HAL_MPU_Disable();

  MPU_Region_InitTypeDef mpu_init;
  mpu_init.Enable = MPU_REGION_ENABLE;
  mpu_init.BaseAddress = (uint32_t)&_dma_ram_start;
  mpu_init.Size = MPU_REGION_SIZE_256KB;
  mpu_init.AccessPermission = MPU_REGION_FULL_ACCESS;
  mpu_init.IsBufferable = MPU_ACCESS_NOT_BUFFERABLE;
  mpu_init.IsCacheable = MPU_ACCESS_CACHEABLE;
  mpu_init.IsShareable = MPU_ACCESS_SHAREABLE;
  mpu_init.Number = MPU_REGION_NUMBER0;
  mpu_init.TypeExtField = MPU_TEX_LEVEL1;
  mpu_init.SubRegionDisable = 0x00;
  mpu_init.DisableExec = MPU_INSTRUCTION_ACCESS_ENABLE;
  HAL_MPU_ConfigRegion(&mpu_init);

  HAL_MPU_Enable(MPU_PRIVILEGED_DEFAULT);
#endif

  memcpy(&_dma_ram_start, &_dma_ram_data, (size_t)(&_dma_ram_end - &_dma_ram_start));
#endif
}

static timer_index_t flight_timer;

static uint32_t flight_timer_period() {
  return (uint32_t)(state.looptime_autodetect * 2.0f + 0.5f) - 1;
}

bool flight_timer_irq_handler() {
  if (flight_timer == TIMER_INVALID || !timer_up_pending(flight_timer))
    return false;

  BaseType_t wake = pdFALSE;
  vTaskNotifyGiveFromISR(threads[THREAD_FLIGHT].handle, &wake);
  return wake == pdTRUE;
}

void flight_thread(void *) {
  // load settings from flash
  flash_load();

  // wait for flash to stabilze
  time_delay_us(100);

  // setup filters early
  filter_global_init();
  timer_alloc_init();

  // Turn on LED during boot so that if a delay is used as part of using programming pins for other functions,
  // the FC does not appear inactive while programming times out
  led_init();
  led_on(LEDALL);

  debug_pin_init();
  buzzer_init();
  usb_init();
  target_init();
  simulator_init();

  rgb_led_init();
  motor_init();
  motor_set_all(MOTOR_OFF);
  servo_init();

  // wait for devices to wake up
  time_delay_ms(100);

  baro_init();
  rx_spektrum_bind();

  osd_init();
  sixaxis_init();
  // needs to happen after gyro is detected so we know its update period
  scheduler_init();

  pid_init();
  control_filter_update(false);

  time_delay_ms(50);
  sixaxis_gyro_cal();

  adc_init();
  vbat_init();

  rx_init();
  gps_init();
  vtx_init();

  blackbox_init();
  imu_init();
  usb_configurator_mutex = xSemaphoreCreateMutexStatic(&usb_mutex_storage);
  configASSERT(usb_configurator_mutex);
  thread_start(THREAD_USB);

  task_reset_runtime();

  uint32_t last_loop_cycles = time_cycles();

  flight_timer = TIMER_TAG_TIM(timer_alloc(TIMER_USE_SCHEDULER));
  configASSERT(flight_timer != TIMER_INVALID);

  timer_up_init(flight_timer, PWM_CLOCK_FREQ_HZ / 2000000, flight_timer_period());
  interrupt_enable(timer_defs[flight_timer].irq, TIMER_PRIORITY);
  timer_up_start(flight_timer);

  while (1) {
    xTaskNotifyGive(threads[THREAD_BLACKBOX].handle);

    const uint32_t elapsed_cycles = time_cycles() - last_loop_cycles;
    ulTaskNotifyTake(pdTRUE, portMAX_DELAY); // Coalesce overruns; never replay old cycles.

    // USB blocks arming in control_update_arming(). This lock protects live
    // settings, including a command still finishing after USB disconnects.
    // Armed Flight never locks: the ground-only USB thread is suspended.
    mutex_guard_t configuration(usb_configurator_mutex, !flags.arm_state);

    const float previous_period = state.looptime_autodetect;
    const uint32_t cycles = scheduler_update_loop(elapsed_cycles);
    last_loop_cycles = cycles;
    if (state.looptime_autodetect != previous_period) {
      timer_up_set_period(flight_timer, flight_timer_period());
    }

    simulator_update();
    scheduler_run(cycles);

    // Apply masks after control resolves arming, before releasing configuration
    // ownership: a ground worker must never be suspended holding its mutex.
    threads_update();
  }
}

#ifndef PIO_UNIT_TESTING
__attribute__((__used__)) int main() {
  gpio_ports_init();
  interrupt_init();
  time_init();
  thread_start(THREAD_FLIGHT);
  vTaskStartScheduler();
  thread_assert_failed();
}
#endif
