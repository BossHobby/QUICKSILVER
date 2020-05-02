#include "sixaxis.h"

#include <math.h>
#include <stdint.h>
#include <stdio.h>

#include "binary.h"
#include "debug.h"
#include "drv_i2c.h"
#include "drv_serial.h"
#include "drv_spi_mpu6xxx.h"
#include "drv_time.h"
#include "filter.h"
#include "led.h"
#include "profile.h"
#include "project.h"
#include "sixaxis.h"
#include "util.h"

// this works only on newer boards (non mpu-6050)
// on older boards the hw gyro setting controls the acc as well
#define ACC_LOW_PASS_FILTER 5
#define CAL_TIME 2e6
#define GLOW_TIME 62500

// this is the value of both cos 45 and sin 45 = 1/sqrt(2)
#define INVSQRT2 0.707106781f

// temporary fix for compatibility between versions
#ifndef GYRO_ID_1
#define GYRO_ID_1 0x68
#endif
#ifndef GYRO_ID_2
#define GYRO_ID_2 0x98
#endif
#ifndef GYRO_ID_3
#define GYRO_ID_3 0x7D
#endif
#ifndef GYRO_ID_4
#define GYRO_ID_4 0x72
#endif

static filter_t filter[FILTER_MAX_SLOTS];
static filter_state_t filter_state[FILTER_MAX_SLOTS][3];

extern profile_t profile;

float accel[3];
float gyro[3];

float gyro_raw[3];

float accelcal[3];
float gyrocal[3];

void sixaxis_init(void) {
#ifdef F4
  //Initialize SPI
  spi_gyro_init();
  //Initialize Gyro
  MPU6XXX_dma_spi_write(MPU_RA_PWR_MGMT_1, MPU_BIT_H_RESET); //reg 107 soft reset  MPU_BIT_H_RESET
  delay(100000);
  MPU6XXX_dma_spi_write(MPU_RA_SIGNAL_PATH_RESET, MPU_RESET_SIGNAL_PATHWAYS);
  delay(100000);
  MPU6XXX_dma_spi_write(MPU_RA_PWR_MGMT_1, MPU_CLK_SEL_PLLGYROX); //reg 107 set pll clock to 1 for x axis reference
  delay(100000);
  MPU6XXX_dma_spi_write(MPU_RA_USER_CTRL, MPU_BIT_I2C_IF_DIS); //reg 106 to 16 enabling spi
  delay(1500);
  MPU6XXX_dma_spi_write(MPU_RA_PWR_MGMT_2, MPU_BITS_STDBY_MODE_OFF); //reg 108 disable standbye mode to 0
  delay(1500);
  MPU6XXX_dma_spi_write(MPU_RA_SMPLRT_DIV, MPU_BIT_SMPLRT_DIVIDER_OFF); //reg 25 sample rate divider to 0
  delay(1500);
  MPU6XXX_dma_spi_write(MPU_RA_CONFIG, MPU_BITS_DLPF_CFG_256HZ); //reg 26 dlpf to 0 - 8khz
  delay(1500);
  MPU6XXX_dma_spi_write(MPU_RA_ACCEL_CONFIG, MPU_BITS_FS_16G); //reg 28 accel scale to 16G
  delay(1500);
  MPU6XXX_dma_spi_write(MPU_RA_GYRO_CONFIG, MPU_BITS_FS_2000DPS); //reg 27 gyro scale to 2000deg/s
  delay(1500);
  MPU6XXX_dma_spi_write(MPU_RA_INT_ENABLE, MPU_BIT_INT_STATUS_DATA); //reg 56 data ready enable interrupt to 1
  delay(1500);
#endif

#ifdef F0
  // gyro soft reset
  i2c_writereg(107, 128);
  delay(40000);
  // set pll to 1, clear sleep bit old type gyro (mpu-6050)
  i2c_writereg(107, 1);
  int newboard = !(0x68 == i2c_readreg(117));
  delay(100);
  // set accelerometer scale to 16G
  i2c_writereg(28, B00011000);
  // acc lpf for the new gyro type
  //       0-6 ( same as gyro)
  if (newboard)
    i2c_writereg(29, ACC_LOW_PASS_FILTER);
  // gyro scale 2000 deg (FS =3)
  i2c_writereg(27, 24);
  // Gyro DLPF low pass filter
  i2c_writereg(26, GYRO_LOW_PASS_FILTER);
#endif

  for (uint8_t i = 0; i < FILTER_MAX_SLOTS; i++) {
    filter_init(profile.filter.gyro[i].type, &filter[i], filter_state[i], 3, profile.filter.gyro[i].cutoff_freq);
  }
}

int sixaxis_check(void) {
#ifndef DISABLE_GYRO_CHECK
// read "who am I" register
#ifdef F4
  uint8_t id = MPU6XXX_dma_spi_read(MPU_RA_WHO_AM_I);
#endif
#ifdef F0
  int id = i2c_readreg(117);
#endif
#ifdef DEBUG
  debug.gyroid = id;
#endif

  return (GYRO_ID_1 == id || GYRO_ID_2 == id || GYRO_ID_3 == id || GYRO_ID_4 == id);
#else
  return 1;
#endif
}

void sixaxis_read(void) {
  int data[14];

#ifdef F0
  i2c_readdata(59, data, 14);
#endif
#ifdef F4
  MPU6XXX_dma_read_data(59, data, 14);
#endif

  accel[0] = -(int16_t)((data[0] << 8) + data[1]);
  accel[1] = -(int16_t)((data[2] << 8) + data[3]);
  accel[2] = (int16_t)((data[4] << 8) + data[5]);

  if (profile.motor.gyro_orientation & GYRO_ROTATE_90_CW) {
    float temp = accel[1];
    accel[1] = accel[0];
    accel[0] = -temp;
  }

  if (profile.motor.gyro_orientation & GYRO_ROTATE_45_CCW) {
    float temp = accel[0];
    accel[0] = (accel[0] * INVSQRT2 + accel[1] * INVSQRT2);
    accel[1] = -(temp * INVSQRT2 - accel[1] * INVSQRT2);
  }

  if (profile.motor.gyro_orientation & GYRO_ROTATE_45_CW) {
    float temp = accel[1];
    accel[1] = (accel[1] * INVSQRT2 + accel[0] * INVSQRT2);
    accel[0] = -(temp * INVSQRT2 - accel[0] * INVSQRT2);
  }

  if (profile.motor.gyro_orientation & GYRO_ROTATE_90_CCW) {
    float temp = accel[1];
    accel[1] = -accel[0];
    accel[0] = temp;
  }

  if (profile.motor.gyro_orientation & GYRO_ROTATE_180) {
    accel[1] = -accel[1];
    accel[0] = -accel[0];
  }

  if (profile.motor.gyro_orientation & GYRO_FLIP_180) {
    accel[2] = -accel[2];
    accel[0] = -accel[0];
  }

  //order
  gyro_raw[1] = (int16_t)((data[8] << 8) + data[9]);
  gyro_raw[0] = (int16_t)((data[10] << 8) + data[11]);
  gyro_raw[2] = (int16_t)((data[12] << 8) + data[13]);

  gyro_raw[0] = gyro_raw[0] - gyrocal[0];
  gyro_raw[1] = gyro_raw[1] - gyrocal[1];
  gyro_raw[2] = gyro_raw[2] - gyrocal[2];

  if (profile.motor.gyro_orientation & GYRO_ROTATE_90_CW) {
    float temp = gyro_raw[1];
    gyro_raw[1] = -gyro_raw[0];
    gyro_raw[0] = temp;
  }

  if (profile.motor.gyro_orientation & GYRO_ROTATE_45_CCW) {
    float temp = gyro_raw[1];
    gyro_raw[1] = gyro_raw[0] * INVSQRT2 + gyro_raw[1] * INVSQRT2;
    gyro_raw[0] = gyro_raw[0] * INVSQRT2 - temp * INVSQRT2;
  }

  if (profile.motor.gyro_orientation & GYRO_ROTATE_45_CW) {
    float temp = gyro_raw[0];
    gyro_raw[0] = gyro_raw[1] * INVSQRT2 + gyro_raw[0] * INVSQRT2;
    gyro_raw[1] = gyro_raw[1] * INVSQRT2 - temp * INVSQRT2;
  }

  if (profile.motor.gyro_orientation & GYRO_ROTATE_90_CCW) {
    float temp = gyro_raw[1];
    gyro_raw[1] = gyro_raw[0];
    gyro_raw[0] = -temp;
  }

  if (profile.motor.gyro_orientation & GYRO_ROTATE_180) {
    gyro_raw[1] = -gyro_raw[1];
    gyro_raw[0] = -gyro_raw[0];
  }

  if (profile.motor.gyro_orientation & GYRO_FLIP_180) {
    gyro_raw[1] = -gyro_raw[1];
    gyro_raw[2] = -gyro_raw[2];
  }

  //gyro_raw[0] = - gyro_raw[0];
  gyro_raw[1] = -gyro_raw[1];
  gyro_raw[2] = -gyro_raw[2];

  filter_coeff(profile.filter.gyro[0].type, &filter[0], profile.filter.gyro[0].cutoff_freq);
  filter_coeff(profile.filter.gyro[1].type, &filter[1], profile.filter.gyro[1].cutoff_freq);

  for (int i = 0; i < 3; i++) {
    gyro[i] = gyro_raw[i] * 0.061035156f * DEGTORAD;

    gyro[i] = filter_step(profile.filter.gyro[0].type, &filter[0], &filter_state[0][i], gyro[i]);
    gyro[i] = filter_step(profile.filter.gyro[1].type, &filter[1], &filter_state[1][i], gyro[i]);
  }
}

void gyro_cal(void) {
  int data[6];
  float limit[3];
  unsigned long time = gettime();
  unsigned long timestart = time;
  unsigned long timemax = time;
  unsigned long lastlooptime = time;

  float gyro[3];

  for (int i = 0; i < 3; i++) {
    limit[i] = gyrocal[i];
  }

  // 2 and 15 seconds
  while (time - timestart < CAL_TIME && time - timemax < 15e6) {

    unsigned long looptime;
    looptime = time - lastlooptime;
    lastlooptime = time;
    if (looptime == 0)
      looptime = 1;

#ifdef F0
    i2c_readdata(67, data, 6);
#endif
#ifdef F4
    MPU6XXX_dma_read_data(67, data, 6);
#endif

    gyro[1] = (int16_t)((data[0] << 8) + data[1]);
    gyro[0] = (int16_t)((data[2] << 8) + data[3]);
    gyro[2] = (int16_t)((data[4] << 8) + data[5]);

    static int brightness = 0;
    led_pwm(brightness);
    if ((brightness & 1) ^ ((time - timestart) % GLOW_TIME > (GLOW_TIME >> 1))) {
      brightness++;
    }

    brightness &= 0xF;

    for (int i = 0; i < 3; i++) {

      if (gyro[i] > limit[i])
        limit[i] += 0.1f; // 100 gyro bias / second change
      if (gyro[i] < limit[i])
        limit[i] -= 0.1f;

      limitf(&limit[i], 800);

      if (fabsf(gyro[i]) > 100 + fabsf(limit[i])) {
        timestart = gettime();
        brightness = 1;
      } else {
        lpf(&gyrocal[i], gyro[i], lpfcalc((float)looptime, 0.5 * 1e6));
      }
    }

    while ((gettime() - time) < 1000)
      delay(10);
    time = gettime();
  }

  if (time - timestart < CAL_TIME) {
    for (int i = 0; i < 3; i++) {
      gyrocal[i] = 0;
    }
  }
}

void acc_cal(void) {
  accelcal[2] = 2048;
  for (int y = 0; y < 500; y++) {
    sixaxis_read();
    for (int x = 0; x < 3; x++) {
      lpf(&accelcal[x], accel[x], 0.92);
    }
    gettime(); // if it takes too long time will overflow so we call it here
  }
  accelcal[2] -= 2048;

#ifdef FLASH_SAVE2
  for (int x = 0; x < 3; x++) {
    limitf(&accelcal[x], 127);
  }
#endif

#ifdef FLASH_SAVE1
  for (int x = 0; x < 3; x++) {
    limitf(&accelcal[x], 500);
  }
#endif
}
