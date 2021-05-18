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

#ifdef F4
#include "blackbox.h"
#include "drv_usb.h"
#include "usb_configurator.h"
#endif

// hal
void clk_init(void);
extern void flash_load(void);
extern void flash_hard_coded_pid_identifier(void);
uint32_t lastlooptime;
uint8_t looptime_warning;
uint8_t blown_loop_counter;
float looptime_buffer[255];
extern profile_t profile;

// for led flash on gestures
int ledcommand = 0;
int ledblink = 0;
unsigned long ledcommandtime = 0;

void failloop(int val);
#if defined(USE_SERIAL_4WAY_BLHELI_INTERFACE) && defined(F0)
volatile int switch_to_4way = 0;
static void setup_4way_external_interrupt(void);
#endif
int random_seed = 0;

int main(void) {
//init some initial values
  //attempt 8k looptime for f405 or 4k looptime for f411
  state.looptime_autodetect = LOOPTIME;
  // load default profile
  profile_set_defaults();
  // setup filters early
  filter_global_init();
  pid_init();
  // read pid identifier for values in file pid.c
  flash_hard_coded_pid_identifier();
  // load flash saved variables
  flash_load();

  delay(1000);

//init some hardware things
  gpio_init();
  usb_init();
  ledon(255); //Turn on LED during boot so that if a delay is used as part of using programming pins for other functions, the FC does not appear inactive while programming times out
  spi_init();
  timer_init();
  usart_invert();
#if defined(RX_DSMX_2048) || defined(RX_DSM2_1024) || defined(RX_UNIFIED_SERIAL)
  rx_spektrum_bind();
#endif

  delay(100000);

//init the firmware things
  motor_init();
  motor_set_all(0);
  sixaxis_init();
  if (!sixaxis_check()) {
    //gyro not found
    failloop(4);
  }
#ifdef ENABLE_OSD
  delay(300000);
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

  delay(1000);
  vbat_init();

#ifdef RX_BAYANG_BLE_APP
  // for randomising MAC adddress of ble app - this will make the int = raw float value
  random_seed = *(int *)&state.vbattfilt;
  random_seed = random_seed & 0xff;
#endif

  gyro_cal();
  rgb_init();
  blackbox_init();
  imu_init();

#ifdef ENABLE_OSD
  osd_clear();
#endif

  extern int liberror;
  if (liberror) {
    failloop(7);
  }

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

    { // gettime() needs to be called at least once per second
      volatile uint32_t _ = gettime();
      _;
    }

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
    sixaxis_read();

    // all flight calculations and motors
    control();

    // attitude calculations for level mode
    imu_calc();

    // battery low logic
    vbat_calc();

    // check gestures
    if (flags.on_ground) {
      gestures();
    }

    if (LED_NUMBER > 0) {
      // led flash logic
      if (flags.lowbatt)
        ledflash(500000, 8);
      else {
        if (flags.rx_mode == RXMODE_BIND) { // bind mode
          ledflash(100000, 12);
        } else { // non bind
          if (flags.failsafe) {
            ledflash(500000, 15);
          } else {
            int leds_on = !rx_aux_on(AUX_LEDS_ON);
            if (ledcommand) {
              if (!ledcommandtime)
                ledcommandtime = gettime();
              if (gettime() - ledcommandtime > 500000) {
                ledcommand = 0;
                ledcommandtime = 0;
              }
              ledflash(100000, 8);
            } else if (ledblink) {
              unsigned long time = gettime();
              if (!ledcommandtime) {
                ledcommandtime = time;
                if (leds_on)
                  ledoff(255);
                else
                  ledon(255);
              }
              if (time - ledcommandtime > 500000) {
                ledblink--;
                ledcommandtime = 0;
              }
              if (time - ledcommandtime > 300000) {
                if (leds_on)
                  ledon(255);
                else
                  ledoff(255);
              }
            } else if (leds_on) {
              if (LED_BRIGHTNESS != 15)
                led_pwm(LED_BRIGHTNESS);
              else
                ledon(255);
            } else
              ledoff(255);
          }
        }
      }
    }

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
#ifdef SERIAL_RX
    // if our RX is a serial, only check if we have valid usart and its the one currently active
    if (serial_rx_port == profile.serial.rx && serial_rx_port != USART_PORT_INVALID) {
      rx_check();
    }
#else
    // we have a spi RX
    rx_check();
#endif

#ifdef ENABLE_OSD
    osd_display();
#endif

#ifdef F4
    blackbox_update();
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

#ifdef DEBUG
    static uint32_t loop_counter = 0; //For tagging loops that ran long, short, freaked out, etc. Yes, Bobnova was here.

    debug.vbatt_comp = state.vbatt_comp;
    debug.cpu_load = state.cpu_load; // * 1e-3f;

    if (loop_counter > 10000) {
      if (debug.cpu_load > debug.max_cpu_load) // First "few" loops are messy
      {
        if (loop_counter < 11000) {
          debug.min_cpu_load = 1337.0f;
        }
        debug.max_cpu_load = debug.cpu_load;
        debug.loops_between_max_cpu_load = loop_counter - debug.max_cpu_loop_number;
        debug.max_cpu_loop_number = loop_counter;
      } else if (debug.cpu_load == debug.max_cpu_load) {
        debug.loops_between_max_cpu_load = loop_counter - debug.max_cpu_loop_number;
        debug.max_cpu_loop_number = loop_counter;
      } else if (debug.cpu_load < debug.min_cpu_load) // First "few" loops are messy
      {
        debug.min_cpu_load = debug.cpu_load;
        debug.loops_between_min_cpu_load = loop_counter - debug.min_cpu_loop_number;
        debug.min_cpu_loop_number = loop_counter;
      } else if (debug.cpu_load == debug.min_cpu_load) {
        debug.loops_between_min_cpu_load = loop_counter - debug.min_cpu_loop_number;
        debug.min_cpu_loop_number = loop_counter + 0x0001000;
      }
    }

    loop_counter++;
#endif

    while ((timer_micros() - time) < state.looptime_autodetect)
      __NOP();

  } // end loop
}

// 2 - low battery at powerup - if enabled by config
// 3 - radio chip not detected
// 4 - Gyro not found
// 5 - clock , intterrupts , systick
// 6 - loop time issue
// 7 - i2c error
// 8 - i2c error main loop

const char *failloop_string(int val) {
  switch (val) {
  case 2:
    return "low battery at powerup - if enabled by config";
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
#if defined(F4) && defined(DEBUG)
    quic_debugf("failloop %s (%d)", failloop_string(val), val);
    usb_detect();
#endif
    for (int i = 0; i < val; i++) {
      ledon(255);
      delay(500000);
      ledoff(255);
      delay(500000);
    }
    delay(800000);
  }
}

void handle_fault() {
#if defined(F4) && defined(RESET_ON_FAULT)
  extern void systemResetToBootloader(void);
  systemResetToBootloader();
#endif

  failloop(5);
}

void HardFault_Handler(void) {
  handle_fault();
}
void MemManage_Handler(void) {
  handle_fault();
}
void BusFault_Handler(void) {
  handle_fault();
}
void UsageFault_Handler(void) {
  handle_fault();
}

