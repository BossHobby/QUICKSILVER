#include <math.h>
#include <stdint.h>
#include <stdio.h>
#include <string.h>

#include "core/debug.h"
#include "core/failloop.h"
#include "core/flash.h"
#include "core/profile.h"
#include "core/project.h"
#include "core/scheduler.h"
#include "driver/adc.h"
#include "driver/gpio.h"
#include "driver/interrupt.h"
#include "driver/motor.h"
#include "driver/rgb_led.h"
#include "driver/time.h"
#include "driver/timer.h"
#include "driver/usb.h"
#include "flight/filter.h"
#include "flight/imu.h"
#include "flight/pid.h"
#include "flight/sixaxis.h"
#include "io/blackbox.h"
#include "io/buzzer.h"
#include "io/led.h"
#include "io/vbat.h"
#include "io/vtx.h"
#include "osd/render.h"

__attribute__((__used__)) void
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

__attribute__((__used__)) int main() {
  interrupt_init();

  // init timer so we can use delays etc
  time_init();

  // load settings from flash
  flash_load();

  // wait for flash to stabilze
  time_delay_us(100);

  // setup filters early
  filter_global_init();
  timer_alloc_init();
  pid_init();

  // init some hardware things
  gpio_ports_init();

  // Turn on LED during boot so that if a delay is used as part of using programming pins for other functions,
  // the FC does not appear inactive while programming times out
  led_init();
  led_on(LEDALL);

  debug_pin_init();
  buzzer_init();

  usb_init();

  rx_spektrum_bind();

  // init motors
  motor_init();

  // wait for devices to wake up
  time_delay_ms(300);

  if (!sixaxis_detect()) {
    // gyro not found
    failloop(FAILLOOP_GYRO);
  }

  scheduler_init();
  sixaxis_init();

  // give the gyro some time to settle
  time_delay_ms(100);

  // display bootlogo while calibrating
  osd_init();
  sixaxis_gyro_cal();

  // wait for adc and vtx to wake up
  time_delay_ms(100);

  // send first value to esc
  motor_set_all(MOTOR_OFF);
  motor_update();

  adc_init();
  vbat_init();

  rx_init();
  vtx_init();
  rgb_init();

  blackbox_init();
  imu_init();
  osd_clear();

  scheduler_run();
}
