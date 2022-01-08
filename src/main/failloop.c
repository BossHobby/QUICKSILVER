#include "failloop.h"

#include "drv_motor.h"
#include "drv_time.h"
#include "led.h"

const char *failloop_string(failloop_t val) {
  switch (val) {
  case FAILLOOP_LOW_BATTERY:
    return "low battery at powerup - unused";
  case FAILLOOP_RADIO:
    return "radio chip not detected";
  case FAILLOOP_GYRO:
    return "Gyro not found";
  case FAILLOOP_FAULT:
    return "clock, intterrupts, systick";
  case FAILLOOP_LOOPTIME:
    return "loop time issue";
  case FAILLOOP_SPI:
    return "i2c error";
  case FAILLOOP_I2C:
    return "i2c error main loop";
  default:
    return "unknown error";
  }
}

void failloop(failloop_t val) {
  motor_set_all(0);

  while (1) {
#if defined(STM32F4) && defined(DEBUG)
    quic_debugf("failloop %s (%d)", failloop_string(val), val);
    usb_detect();
#endif
    for (int i = 0; i < val; i++) {
      ledon(255);
      time_delay_us(500000);
      ledoff(255);
      time_delay_us(500000);
    }
    time_delay_us(800000);
  }
}

void handle_fault() {
#if defined(STM32F4) && defined(RESET_ON_FAULT)
  extern void systemResetToBootloader();
  systemResetToBootloader();
#endif

  failloop(FAILLOOP_FAULT);
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
