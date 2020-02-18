#include <math.h>
#include <stdint.h>
#include <stdio.h>

#include "binary.h"
#include "buzzer.h"
#include "control.h"
#include "debug.h"
#include "drv_adc.h"
#include "drv_fmc2.h"
#include "drv_gpio.h"
#include "drv_i2c.h"
#include "drv_motor.h"
#include "drv_serial.h"
#include "drv_softi2c.h"
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
#include "rx.h"
#include "sixaxis.h"
#include "util.h"
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

// looptime in seconds
float looptime;
// running sum of looptimes
float osd_totaltime;
// filtered battery in volts
float vbattfilt = 0.0;
float vbattfilt_corr = 4.2;
float vbatt_comp = 4.2;
// voltage reference for vcc compensation
float vreffilt = 1.0;
// average of all motors
float thrfilt = 0;
float lipo_cell_count = 1.0;

uint32_t lastlooptime;
// signal for lowbattery
int lowbatt = 1;

//int minindex = 0;

extern profile_t profile;

// holds the main four channels, roll, pitch , yaw , throttle
float rx[4];

// holds auxilliary channels
// the last 2 are always on and off respectively
char aux[AUX_CHANNEL_MAX] = {0, 0, 0, 0, 0, 0};
char lastaux[AUX_CHANNEL_MAX];
// if an aux channel has just changed
char auxchange[AUX_CHANNEL_MAX];

// bind / normal rx mode
extern int rxmode;
// failsafe on / off
extern int failsafe;
extern int onground;
int in_air;
int armed_state;
int arming_release;
int binding_while_armed = 1;

//Flash Memory Feature defaults for a flash w/full chip erase
int flash_feature_1 = 1; //SETUP WIZARD
int flash_feature_2 = 0; //LVC

// for led flash on gestures
int ledcommand = 0;
int ledblink = 0;
unsigned long ledcommandtime = 0;

uint32_t loopCounter = 0; //For tagging loops that ran long, short, freaked out, etc. Yes, Bobnova was here.
float cpu_load = 0;

void failloop(int val);
#if defined(USE_SERIAL_4WAY_BLHELI_INTERFACE) && defined(F0)
volatile int switch_to_4way = 0;
static void setup_4way_external_interrupt(void);
#endif
int random_seed = 0;

int main(void) {
  // load default profile
  profile_set_defaults();

  // setup filters early
  filter_global_init();
  pid_init();

#ifdef FLASH_SAVE1
  // read pid identifier for values in file pid.c
  flash_hard_coded_pid_identifier();

  // load flash saved variables
  flash_load();
#endif

  delay(1000);

#ifdef ENABLE_OVERCLOCK
  clk_init();
#endif

  gpio_init();

#ifdef F4
  usb_init();
#endif

  ledon(255); //Turn on LED during boot so that if a delay is used as part of using programming pins for other functions, the FC does not appear inactive while programming times out
  spi_init();
  timer_init();
  usart_invert();

#if defined(RX_DSMX_2048) || defined(RX_DSM2_1024) || defined(RX_UNIFIED_SERIAL)
  rx_spektrum_bind();
#endif

  delay(100000);

  i2c_init();

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
  aux[AUX_CHANNEL_ON] = 1;
  aux[AUX_CHANNEL_OFF] = 0;

#ifdef GESTURE_AUX_START_ON
  aux[AUX_CHANNEL_GESTURE] = 1;
#endif

  //temp placeholder for old flash load function

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

  int count = 0;
  delay(1000);

  while (count < 5000) {
    float bootadc = adc_read(0) * vreffilt;
    lpf(&vreffilt, adc_read(1), 0.9968f);
    lpf(&vbattfilt, bootadc, 0.9968f);
    count++;
  }

  if (profile.voltage.lipo_cell_count == 0) {
    // Lipo count not specified, trigger auto detect
    for (int i = 6; i > 0; i--) {
      float cells = i;
      if (vbattfilt / cells > 3.7f) {
        lipo_cell_count = (float)cells;
        break;
      }
    }
  } else {
    lipo_cell_count = (float)profile.voltage.lipo_cell_count;
  }

  vbattfilt_corr *= (float)lipo_cell_count;

#ifdef RX_BAYANG_BLE_APP
  // for randomising MAC adddress of ble app - this will make the int = raw float value
  random_seed = *(int *)&vbattfilt;
  random_seed = random_seed & 0xff;
#endif

  gyro_cal();
  extern void rgb_init(void);
  rgb_init();
  blackbox_init();

#ifdef SERIAL_ENABLE
  serial_init();
#endif

  imu_init();
#ifdef FLASH_SAVE2
  // read accelerometer calibration values from option bytes ( 2* 8bit)
  extern float accelcal[3];
  accelcal[0] = flash2_readdata(OB->DATA0) - 127;
  accelcal[1] = flash2_readdata(OB->DATA1) - 127;
#endif

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

#if defined(USE_SERIAL_4WAY_BLHELI_INTERFACE) && defined(F0)
  setup_4way_external_interrupt();
#endif
  while (1) {
    // gettime() needs to be called at least once per second
    {
      volatile uint32_t _ = gettime();
      _;
    }

    uint32_t time = timer_micros();
    looptime = ((uint32_t)(time - lastlooptime));
    if (looptime <= 0)
      looptime = 1;
    looptime = looptime * 1e-6f;
    if (looptime > 0.02f) { // max loop 20ms
      failloop(6);
      //endless loop
    }
    osd_totaltime += looptime;
#ifdef DEBUG
    debug.totaltime += looptime;
    lpf(&debug.timefilt, looptime, 0.998);
#endif
    lastlooptime = time;

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

    // read acd and scale based on processor voltage
    float battadc = adc_read(0) * vreffilt;
    // read and filter internal reference
    lpf(&vreffilt, adc_read(1), 0.9968f);

    // average of all 4 motor thrusts
    // should be proportional with battery current
    extern float thrsum; // from control.c

    // filter motorpwm so it has the same delay as the filtered voltage
    // ( or they can use a single filter)
    lpf(&thrfilt, thrsum, 0.9968f); // 0.5 sec at 1.6ms loop time

    // li-ion battery model compensation time decay ( 18 seconds )
    lpf(&vbattfilt_corr, vbattfilt, FILTERCALC(1000, 18000e3));

    lpf(&vbattfilt, battadc, 0.9968f);

    // compensation factor for li-ion internal model
    // zero to bypass
#define CF1 0.25f

    float tempvolt = vbattfilt * (1.00f + CF1) - vbattfilt_corr * (CF1);

#ifdef AUTO_VDROP_FACTOR

    static float lastout[12];
    static float lastin[12];
    static float vcomp[12];
    static float score[12];
    static int z = 0;
    static int minindex = 0;
    static int firstrun = 1;

    if (thrfilt > 0.1f) {
      vcomp[z] = tempvolt + (float)z * 0.1f * thrfilt;

      if (firstrun) {
        for (int y = 0; y < 12; y++)
          lastin[y] = vcomp[z];
        firstrun = 0;
      }
      float ans;
      //  y(n) = x(n) - x(n-1) + R * y(n-1)
      //  out = in - lastin + coeff*lastout
      // hpf
      ans = vcomp[z] - lastin[z] + FILTERCALC(1000 * 12, 6000e3) * lastout[z];
      lastin[z] = vcomp[z];
      lastout[z] = ans;
      lpf(&score[z], ans * ans, FILTERCALC(1000 * 12, 60e6));
      z++;

      if (z >= 12) {
        z = 0;
        float min = score[0];
        for (int i = 0; i < 12; i++) {
          if ((score[i]) < min) {
            min = (score[i]);
            minindex = i;
            // add an offset because it seems to be usually early
            minindex++;
          }
        }
      }
    }

#undef VDROP_FACTOR
#define VDROP_FACTOR minindex * 0.1f
#endif

    float hyst;
    if (lowbatt)
      hyst = HYST;
    else
      hyst = 0.0f;

    if ((tempvolt + (float)VDROP_FACTOR * thrfilt < (profile.voltage.vbattlow * lipo_cell_count) + hyst) || (vbattfilt < (float)2.7f))
      lowbatt = 1;
    else
      lowbatt = 0;

    vbatt_comp = tempvolt + (float)VDROP_FACTOR * thrfilt;

#ifdef DEBUG
    debug.vbatt_comp = vbatt_comp;
#endif
    // check gestures
    if (onground) {
      gestures();
    }

    if (LED_NUMBER > 0) {
      // led flash logic
      if (lowbatt)
        ledflash(500000, 8);
      else {
        if (rxmode == RXMODE_BIND) { // bind mode
          ledflash(100000, 12);
        } else { // non bind
          if (failsafe) {
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
    extern void rgb_led_lvc(void);
    rgb_led_lvc();
#ifdef RGB_LED_DMA
    extern void rgb_dma_start();
    rgb_dma_start();
#endif
#endif

#ifdef BUZZER_ENABLE
    buzzer();
#endif

    vtx_update();

#if defined(USE_SERIAL_4WAY_BLHELI_INTERFACE) && defined(F0)
    extern int onground;
    if (onground) {
      NVIC_EnableIRQ(EXTI4_15_IRQn);

      if (switch_to_4way) {
        switch_to_4way = 0;

        NVIC_DisableIRQ(EXTI4_15_IRQn);
        ledon(2);
        esc4wayInit();
        esc4wayProcess();
        NVIC_EnableIRQ(EXTI4_15_IRQn);
        ledoff(2);

        reset_looptime();
      }
    } else {
      NVIC_DisableIRQ(EXTI4_15_IRQn);
    }
#endif

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
    usb_detect();
#endif

    cpu_load = (timer_micros() - lastlooptime);

#ifdef DEBUG
    debug.cpu_load = cpu_load; // * 1e-3f;

    if (loopCounter > 10000) {
      if (debug.cpu_load > debug.max_cpu_load) // First "few" loops are messy
      {
        if (loopCounter < 11000) {
          debug.min_cpu_load = 1337.0f;
        }
        debug.max_cpu_load = debug.cpu_load;
        debug.loops_between_max_cpu_load = loopCounter - debug.max_cpu_loop_number;
        debug.max_cpu_loop_number = loopCounter;
      } else if (debug.cpu_load == debug.max_cpu_load) {
        debug.loops_between_max_cpu_load = loopCounter - debug.max_cpu_loop_number;
        debug.max_cpu_loop_number = loopCounter;
      } else if (debug.cpu_load < debug.min_cpu_load) // First "few" loops are messy
      {
        debug.min_cpu_load = debug.cpu_load;
        debug.loops_between_min_cpu_load = loopCounter - debug.min_cpu_loop_number;
        debug.min_cpu_loop_number = loopCounter;
      } else if (debug.cpu_load == debug.min_cpu_load) {
        debug.loops_between_min_cpu_load = loopCounter - debug.min_cpu_loop_number;
        debug.min_cpu_loop_number = loopCounter + 0x0001000;
      }
    }

    loopCounter++;
#endif

    while ((timer_micros() - time) < LOOPTIME)
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

#if defined(USE_SERIAL_4WAY_BLHELI_INTERFACE) && defined(F0)

// set up external interrupt to check
// for 4way serial start byte
static void setup_4way_external_interrupt(void) {
  SYSCFG->EXTICR[3] &= ~(0x000F); //clear bits 3:0 in the SYSCFG_EXTICR1 reg
  EXTI->FTSR |= EXTI_FTSR_TR14;
  EXTI->IMR |= EXTI_IMR_MR14;
  NVIC_SetPriority(EXTI4_15_IRQn, 2);
}

// interrupt for detecting blheli serial 4way
// start byte (0x2F) on PA14 at 38400 baud
void EXTI4_15_IRQHandler(void) {
  if ((EXTI->IMR & EXTI_IMR_MR14) && (EXTI->PR & EXTI_PR_PR14)) {
#define IS_RX_HIGH (GPIOA->IDR & GPIO_Pin_14)
    uint32_t micros_per_bit = 26;
    uint32_t micros_per_bit_half = 13;

    uint32_t i = 0;
    // looking for 2F
    uint8_t start_byte = 0x2F;
    uint32_t time_next = gettime();
    time_next += micros_per_bit_half; // move away from edge to center of bit

    for (; i < 8; ++i) {
      time_next += micros_per_bit;
      delay_until(time_next);
      if ((0 == IS_RX_HIGH) != (0 == (start_byte & (1 << i)))) {
        i = 0;
        break;
      }
    }

    if (i == 8) {
      time_next += micros_per_bit;
      delay_until(time_next); // move away from edge

      if (IS_RX_HIGH) // stop bit
      {
        // got the start byte
        switch_to_4way = 1;
      }
    }

    // clear pending request
    EXTI->PR |= EXTI_PR_PR14;
  }
}
#endif
