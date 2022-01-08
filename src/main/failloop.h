#pragma once

// the error codes indicate a failure that prevents normal operation
// led flash codes - the quad will not fly / bind if flashing a code

typedef enum {
  FAILLOOP_LOW_BATTERY = 2, // - low battery at powerup - currently unused
  FAILLOOP_RADIO = 3,       // - radio chip not found
  FAILLOOP_GYRO = 4,        // - Gyro not found
  FAILLOOP_FAULT = 5,       // - clock, intterrupts, systick, gcc bad code, bad memory access (code issues like bad pointers) - this should not come up
  FAILLOOP_LOOPTIME = 6,    // - loop time issue - if loop time exceeds 20mS
  FAILLOOP_SPI = 7,         // - spi error  - triggered by hardware spi driver only
  FAILLOOP_I2C = 8,         // - i2c error main loop  - triggered by depreciated hardware i2c driver only
} failloop_t;

void failloop(failloop_t val);