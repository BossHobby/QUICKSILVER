#include "drv_i2c.h"

#include "drv_hw_i2c.h"
#include "drv_softi2c.h"
#include "project.h"

#ifndef USE_HARDWARE_I2C
#ifndef USE_SOFTWARE_I2C
#ifndef USE_DUMMY_I2C
#define USE_SOFTWARE_I2C
#endif
#endif
#endif
#define SOFTI2C_GYRO_ADDRESS I2C_GYRO_ADDRESS
int liberror = 0;

void i2c_init(void) {
#ifdef USE_HARDWARE_I2C
  hw_i2c_init();
#endif

#ifdef USE_SOFTWARE_I2C
  softi2c_init();
#endif

#ifdef USE_DUMMY_I2C
#ifndef F4
#warning I2C FUNCTIONS DISABLED
#endif
#endif
}

void i2c_writereg(int reg, int data) {
#ifdef USE_HARDWARE_I2C
  hw_i2c_writereg(reg, data);
#endif

#ifdef USE_SOFTWARE_I2C
  softi2c_write(SOFTI2C_GYRO_ADDRESS, reg, data);
#endif

#ifdef USE_DUMMY_I2C

#endif
}

int i2c_readdata(int reg, int *data, int size) {
#ifdef USE_HARDWARE_I2C
  return hw_i2c_readdata(reg, data, size);
#endif

#ifdef USE_SOFTWARE_I2C
  softi2c_readdata(SOFTI2C_GYRO_ADDRESS, reg, data, size);
  return 1;
#endif

#ifdef USE_DUMMY_I2C
  return 1;
#endif
}

int i2c_readreg(int reg) {
#ifdef USE_HARDWARE_I2C
  return hw_i2c_readreg(reg);
#endif

#ifdef USE_SOFTWARE_I2C
  return softi2c_read(SOFTI2C_GYRO_ADDRESS, reg);
#endif

#ifdef USE_DUMMY_I2C
  return 255;
#endif
}
