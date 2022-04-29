#include <math.h>
#include <stdint.h>
#include <stdio.h>
#include <string.h>

#include "debug.h"
#include "drv_adc.h"
#include "drv_fmc.h"
#include "drv_gpio.h"
#include "drv_motor.h"
#include "drv_rgb_led.h"
#include "drv_serial.h"
#include "drv_spi.h"
#include "drv_spi_soft.h"
#include "drv_time.h"
#include "drv_usb.h"
#include "failloop.h"
#include "flash.h"
#include "flight/control.h"
#include "flight/filter.h"
#include "flight/gestures.h"
#include "flight/imu.h"
#include "flight/pid.h"
#include "flight/sixaxis.h"
#include "io/blackbox.h"
#include "io/buzzer.h"
#include "io/led.h"
#include "io/rgb_led.h"
#include "io/usb_configurator.h"
#include "io/vbat.h"
#include "io/vtx.h"
#include "osd_render.h"
#include "profile.h"
#include "project.h"
#include "reset.h"
#include "rx.h"
#include "util/util.h"

#ifdef USE_SERIAL_4WAY_BLHELI_INTERFACE
#include "drv_serial_4way.h"
#include "drv_serial_soft.h"
#endif

extern profile_t profile;

uint32_t lastlooptime;
uint8_t looptime_warning;
uint8_t blown_loop_counter;
float looptime_buffer[255];

int random_seed = 0;

__attribute__((__used__)) void memory_section_init() {
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
  memcpy(&_dma_ram_start, &_dma_ram_data, (size_t)(&_dma_ram_end - &_dma_ram_start));
#endif
}

__attribute__((__used__)) int main() {
  system_check_for_bootloader();

  // init some initial values
  // attempt 8k looptime for f405 or 4k looptime for f411
  state.looptime = LOOPTIME * 1e-6;
  state.looptime_autodetect = LOOPTIME;

  // init timer so we can use delays etc
  time_init();

  // load default profile
  profile_set_defaults();

  // setup filters early
  filter_global_init();
  pid_init();

  // read pid identifier for values in file pid.c
  flash_hard_coded_pid_identifier();

  // load flash saved variables
  flash_load();
  time_delay_us(1000);

  // init some hardware things
  gpio_init();
  usb_init();
  ledon(255); // Turn on LED during boot so that if a delay is used as part of using programming pins for other functions, the FC does not appear inactive while programming times out
  spi_init();
#if defined(RX_DSMX) || defined(RX_DSM2) || defined(RX_UNIFIED_SERIAL)
  rx_spektrum_bind();
#endif

  time_delay_us(100000);

  // init the firmware things
  motor_init();
  motor_set_all(0);

  if (!sixaxis_init()) {
    // gyro not found
    failloop(FAILLOOP_GYRO);
  }

#ifdef ENABLE_OSD
  time_delay_us(300000);
  osd_init();
#endif

  adc_init();

  // set always on channel to on
  state.aux[AUX_CHANNEL_ON] = 1;
  state.aux[AUX_CHANNEL_OFF] = 0;
#ifdef GESTURE_AUX_START_ON
  state.aux[AUX_CHANNEL_GESTURE] = 1;
#endif

  vtx_init();
  rx_init();

  time_delay_us(1000);
  vbat_init();

#ifdef RX_BAYANG_BLE_APP
  // for randomising MAC adddress of ble app - this will make the int = raw float value
  random_seed = *(int *)&state.vbat_filtered;
  random_seed = random_seed & 0xff;
#endif

  sixaxis_gyro_cal();
  rgb_init();

#ifdef ENABLE_BLACKBOX
  blackbox_init();
#endif

  imu_init();

#ifdef ENABLE_OSD
  osd_clear();
#endif

  extern int liberror;
  if (liberror) {
    failloop(FAILLOOP_SPI);
  }

  perf_counter_init();

  lastlooptime = time_micros();

  //
  //
  //    MAIN LOOP
  //
  //

  while (1) {
    uint32_t time = time_micros();
    state.looptime = ((uint32_t)(time - lastlooptime));
    lastlooptime = time;

    perf_counter_start(PERF_COUNTER_TOTAL);

    if (state.looptime <= 0)
      state.looptime = 1;
    state.looptime = state.looptime * 1e-6f;
    if (state.looptime > 0.02f) { // max loop 20ms
      failloop(FAILLOOP_LOOPTIME);
      // endless loop
    }

    // looptime_autodetect sequence
    static uint8_t loop_ctr = 0;
    if (loop_ctr < 255) {
      looptime_buffer[loop_ctr] = state.looptime;
      loop_ctr++;
      if (loop_ctr == 255) {
        float sum = 0;
        for (uint8_t i = 2; i < 255; i++)
          sum += looptime_buffer[i];
        float average_looptime = sum / 253.0f;
        if (average_looptime < .000130f)
          state.looptime_autodetect = LOOPTIME_8K;
        else if (average_looptime < .000255f)
          state.looptime_autodetect = LOOPTIME_4K;
        else
          state.looptime_autodetect = LOOPTIME_2K;
      }
    }

    state.uptime += state.looptime;
    if (flags.arm_state)
      state.armtime += state.looptime;

#ifdef DEBUG
    debug.totaltime += state.looptime;
    lpf(&debug.timefilt, state.looptime, 0.998);
#endif

    if (liberror > 20) {
      failloop(FAILLOOP_SPI_MAIN);
      // endless loop
    }

    // read gyro and accelerometer data
    perf_counter_start(PERF_COUNTER_GYRO);
    sixaxis_read();
    perf_counter_end(PERF_COUNTER_GYRO);

    // all flight calculations and motors
    perf_counter_start(PERF_COUNTER_CONTROL);
    control();
    perf_counter_end(PERF_COUNTER_CONTROL);

    perf_counter_start(PERF_COUNTER_MISC);

    // attitude calculations for level mode
    imu_calc();

    // battery low logic
    vbat_calc();

    // check gestures
    if (flags.on_ground && !flags.gestures_disabled) {
      gestures();
    }

    // handle led commands
    led_update();

#if (RGB_LED_NUMBER > 0)
    // RGB led control
    rgb_led_lvc();
#ifdef RGB_LED_DMA
    rgb_dma_start();
#endif
#endif

#ifdef BUZZER_ENABLE
    buzzer();
#endif

    vtx_update();

    perf_counter_end(PERF_COUNTER_MISC);

    // receiver function
    perf_counter_start(PERF_COUNTER_RX);
    rx_update();
    perf_counter_end(PERF_COUNTER_RX);

    uint8_t blackbox_active = 0;

#ifdef ENABLE_BLACKBOX
    perf_counter_start(PERF_COUNTER_BLACKBOX);
    blackbox_active = blackbox_update();
    perf_counter_end(PERF_COUNTER_BLACKBOX);
#endif

#ifdef ENABLE_OSD
    if (!blackbox_active) {
      perf_counter_start(PERF_COUNTER_OSD);
      osd_display();
      perf_counter_end(PERF_COUNTER_OSD);
    }
#endif

    state.cpu_load = (time_micros() - lastlooptime);
    // one last check to make sure we catch any looptime problems and rerun autodetect live
    if (loop_ctr == 255 && state.cpu_load > state.looptime_autodetect + 5) {
      blown_loop_counter++;
      if (blown_loop_counter > 100) {
        blown_loop_counter = 0;
        loop_ctr = 0;
        looptime_warning++;
      }
    }

    debug_update();

    perf_counter_end(PERF_COUNTER_TOTAL);
    perf_counter_update();

    if (usb_detect()) {
      flags.usb_active = 1;
#ifndef ALLOW_USB_ARMING
      if (rx_aux_on(AUX_ARMING))
        flags.arm_safety = 1; // final safety check to disallow arming during USB operation
#endif
      usb_configurator();
    } else {
      flags.usb_active = 0;
      motor_test.active = 0;
    }

    while ((time_micros() - time) < state.looptime_autodetect)
      __NOP();

  } // end loop
}
