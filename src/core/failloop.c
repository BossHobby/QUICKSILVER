#include "core/failloop.h"

#include "core/reset.h"
#include "driver/motor.h"
#include "driver/time.h"
#include "driver/usb.h"
#include "flight/control.h"
#include "io/led.h"
#include "io/usb_configurator.h"

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
  case FAILLOOP_DMA:
    return "dma error";
  case FAILLOOP_SPI:
    return "spi error";
  default:
    return "unknown error";
  }
}

void failloop(failloop_t val) {
  uint32_t blink_counter = 0;
  uint32_t blink_start = time_millis();

  while (1) {
    motor_set_all(MOTOR_OFF);

    state.failloop = val;

    if (usb_detect()) {
      usb_configurator();
    }

    if ((time_millis() - blink_start) >= 1000) {
      usb_quic_logf("failloop %s (%d)", failloop_string(val), val);
    }

    if (blink_counter < val) {
      if ((time_millis() - blink_start) < 500) {
        led_on(LEDALL);
      } else if ((time_millis() - blink_start) < 1000) {
        led_off(LEDALL);
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
#if defined(RESET_ON_FAULT)
  system_reset_to_bootloader();
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

void Default_Handler() {
  handle_fault();
}