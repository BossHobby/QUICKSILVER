#include "failloop.h"

#include "drv_motor.h"
#include "drv_time.h"
#include "drv_usb.h"
#include "flight/control.h"
#include "led.h"
#include "usb_configurator.h"

const char *failloop_string(failloop_t val) {
  switch (val) {
  case FAILLOOP_LOW_BATTERY:
    return "low battery at powerup - unused";
  case FAILLOOP_RADIO:
    return "radio chip not found";
  case FAILLOOP_GYRO:
    return "gyro not found";
  case FAILLOOP_FAULT:
    return "clock, intterrupts, systick";
  case FAILLOOP_LOOPTIME:
    return "loop time issue";
  case FAILLOOP_SPI:
    return "spi error";
  case FAILLOOP_SPI_MAIN:
    return "spi error main loop";
  default:
    return "unknown error";
  }
}

void failloop(failloop_t val) {
  uint32_t blink_counter = 0;
  uint32_t blink_start = time_millis();

  while (1) {
    motor_set_all(0);

    state.failloop = val;

    if (usb_detect()) {
      usb_configurator();
    }

    if ((time_millis() - blink_start) >= 1000) {
      send_quic_strf(QUIC_CMD_LOG, QUIC_FLAG_NONE, "failloop %s (%d)", failloop_string(val), val);
    }

    if (blink_counter < val) {
      if ((time_millis() - blink_start) < 500) {
        ledon(LEDALL);
      } else if ((time_millis() - blink_start) < 1000) {
        ledoff(LEDALL);
      } else if ((time_millis() - blink_start) >= 1000) {
        blink_counter++;
        blink_start = time_millis();
      }
    } else if ((time_millis() - blink_start) >= 1000) {
      blink_counter = 0;
      blink_start = time_millis();
    }

    time_delay_us(100);
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
