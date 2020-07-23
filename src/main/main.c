#include <math.h>
#include <stdint.h>
#include <stdio.h>

#include "buzzer.h"
#include "control.h"
#include "debug.h"
#include "drv_adc.h"
#include "drv_fmc.h"
#include "drv_gpio.h"
#include "drv_motor.h"
#include "drv_rgb_led.h"
#include "drv_serial.h"
#include "drv_spi.h"
#include "drv_time.h"
#include "filter.h"
#include "flash.h"
#include "gestures.h"
#include "imu.h"
#include "led.h"
#include "osd_render.h"
#include "pid.h"
#include "profile.h"
#include "project.h"
#include "rgb_led.h"
#include "rx.h"
#include "sixaxis.h"
#include "util.h"
#include "vbat.h"
#include "vtx.h"

#ifdef USE_SERIAL_4WAY_BLHELI_INTERFACE
#include "drv_serial_4way.h"
#include "drv_serial_soft.h"
#endif

#ifdef STM32F4
#include "blackbox.h"
#include "drv_usb.h"
#include "usb_configurator.h"
#endif

extern profile_t profile;

uint32_t lastlooptime;
uint8_t looptime_warning;
uint8_t blown_loop_counter;
float looptime_buffer[255];

void failloop(int val);

int random_seed = 0;

int main() {
  //init some initial values
  //attempt 8k looptime for f405 or 4k looptime for f411
  state.looptime_autodetect = LOOPTIME;

  // init timer so we can use delays etc
  timer_init();

  // load default profile
  profile_set_defaults();

  // setup filters early
  filter_global_init();
  pid_init();

  // read pid identifier for values in file pid.c
  flash_hard_coded_pid_identifier();

  // load flash saved variables
  flash_load();
  timer_delay_us(1000);

  //init some hardware things
  gpio_init();
  usb_init();
  ledon(255); //Turn on LED during boot so that if a delay is used as part of using programming pins for other functions, the FC does not appear inactive while programming times out
  spi_init();
  usart_invert();
#if defined(RX_DSMX_2048) || defined(RX_DSM2_1024) || defined(RX_UNIFIED_SERIAL)
  rx_spektrum_bind();
#endif

  timer_delay_us(100000);

  //init the firmware things
  motor_init();
  motor_set_all(0);

  if (!sixaxis_init()) {
    //gyro not found
    failloop(4);
  }

#ifdef ENABLE_OSD
  timer_delay_us(300000);
  osd_init();
#endif

  adc_init();

  //set always on channel to on
  state.aux[AUX_CHANNEL_ON] = 1;
  state.aux[AUX_CHANNEL_OFF] = 0;
#ifdef GESTURE_AUX_START_ON
  state.aux[AUX_CHANNEL_GESTURE] = 1;
#endif

  vtx_init();

#ifdef SERIAL_RX
  // if our RX is a serial, only init if we have valid usart
  if (profile.serial.rx != USART_PORT_INVALID) {
    rx_init();
  }
#else
  // we have a spi RX
  rx_init();
#endif

  timer_delay_us(1000);
  vbat_init();

#ifdef RX_BAYANG_BLE_APP
  // for randomising MAC adddress of ble app - this will make the int = raw float value
  random_seed = *(int *)&state.vbattfilt;
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
    failloop(7);
  }

  perf_counter_init();

  lastlooptime = timer_micros();

  //
  //
  //    MAIN LOOP
  //
  //

  while (1) {
    uint32_t time = timer_micros();
    state.looptime = ((uint32_t)(time - lastlooptime));
    lastlooptime = time;

    perf_counter_start(PERF_COUNTER_TOTAL);

    if (state.looptime <= 0)
      state.looptime = 1;
    state.looptime = state.looptime * 1e-6f;
    if (state.looptime > 0.02f) { // max loop 20ms
      failloop(6);
      //endless loop
    }

    //looptime_autodetect sequence
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

#ifdef DEBUG
    debug.totaltime += state.looptime;
    lpf(&debug.timefilt, state.looptime, 0.998);
#endif

    if (liberror > 20) {
      failloop(8);
      // endless loop
    }

    // read gyro and accelerometer data
    perf_counter_start(PERF_COUNTER_GYRO);
    sixaxis_read();
    perf_counter_end(PERF_COUNTER_GYRO);

    // all flight calculations and motors
    perf_counter_start(PERF_COUNTER_CONTROL);
    control();
    perf_counter_start(PERF_COUNTER_CONTROL);

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

    // receiver function
    perf_counter_start(PERF_COUNTER_RX);
#ifdef SERIAL_RX
    // if our RX is a serial, only check if we have valid usart and its the one currently active
    if (serial_rx_port == profile.serial.rx && serial_rx_port != USART_PORT_INVALID) {
      rx_check();
    }
#else
    // we have a spi RX
    rx_check();
#endif
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

    state.cpu_load = (timer_micros() - lastlooptime);
    //one last check to make sure we catch any looptime problems and rerun autodetect live
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

#ifdef STM32F4
    if (usb_detect()) {
      flags.usb_active = 1;
#ifndef ALLOW_USB_ARMING
      if (rx_aux_on(AUX_ARMING))
        flags.arm_safety = 1; //final safety check to disallow arming during USB operation
#endif
      usb_configurator();
    } else {
      flags.usb_active = 0;
      extern usb_motor_test_t usb_motor_test;
      usb_motor_test.active = 0;
    }
#endif

    while ((timer_micros() - time) < state.looptime_autodetect)
      __NOP();

  } // end loop
}

// the error codes indicate a failure that prevents normal operation
// led flash codes - the quad will not fly / bind if flashing a code
// 2 - low battery at powerup - currently unused
// 3 - radio chip not found
// 4 - Gyro not found
// 5 - clock , intterrupts , systick , gcc bad code , bad memory access (code issues like bad pointers)- this should not come up
// 6 - loop time issue - if loop time exceeds 20mS
// 7 - spi error  - triggered by hardware spi driver only
// 8 - i2c error main loop  - triggered by depreciated hardware i2c driver only

const char *failloop_string(int val) {
  switch (val) {
  case 2:
    return "low battery at powerup - unused";
  case 3:
    return "radio chip not detected";
  case 4:
    return "Gyro not found";
  case 5:
    return "clock , intterrupts , systick";
  case 6:
    return "loop time issue";
  case 7:
    return "i2c error";
  case 8:
    return "i2c error main loop";
  default:
    return "unknown error";
  }
}

void failloop(int val) {
  for (int i = 0; i <= 3; i++) {
    motor_set(i, 0);
  }

  while (1) {
#if defined(STM32F4) && defined(DEBUG)
    quic_debugf("failloop %s (%d)", failloop_string(val), val);
    usb_detect();
#endif
    for (int i = 0; i < val; i++) {
      ledon(255);
      timer_delay_us(500000);
      ledoff(255);
      timer_delay_us(500000);
    }
    timer_delay_us(800000);
  }
}

void handle_fault() {
#if defined(STM32F4) && defined(RESET_ON_FAULT)
  extern void systemResetToBootloader();
  systemResetToBootloader();
#endif

  failloop(5);
}

void HardFault_Handler() {
  handle_fault();
}
void MemManage_Handler() {
  handle_fault();
}
void BusFault_Handler() {
  handle_fault();
}
void UsageFault_Handler() {
  handle_fault();
}
