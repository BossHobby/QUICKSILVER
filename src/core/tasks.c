#include "tasks.h"

#include <stddef.h>

#include "driver/serial.h"
#include "driver/usb.h"
#include "flight/control.h"
#include "flight/gestures.h"
#include "flight/imu.h"
#include "flight/sixaxis.h"
#include "io/blackbox.h"
#include "io/buzzer.h"
#include "io/led.h"
#include "io/usb_configurator.h"
#include "io/vbat.h"
#include "io/vtx.h"
#include "osd/render.h"
#include "profile.h"
#include "project.h"
#include "rx/rx.h"

void util_task() {
  // battery low logic
  vbat_calc();

  // check gestures
  gestures();

  // handle led commands
  led_update();

#if (RGB_LED_NUMBER > 0)
  // RGB led control
  rgb_led_lvc();
#ifdef RGB_LED_DMA
  rgb_dma_start();
#endif
#endif

  buzzer_update();
}

FAST_RAM task_t tasks[TASK_MAX] = {
    [TASK_GYRO] = CREATE_TASK("GYRO", TASK_MASK_ALWAYS, TASK_PRIORITY_REALTIME, sixaxis_read),
    [TASK_IMU] = CREATE_TASK("IMU", TASK_MASK_ALWAYS, TASK_PRIORITY_REALTIME, imu_calc),
    [TASK_PID] = CREATE_TASK("PID", TASK_MASK_ALWAYS, TASK_PRIORITY_REALTIME, control),
    [TASK_RX] = CREATE_TASK("RX", TASK_MASK_ALWAYS, TASK_PRIORITY_REALTIME, rx_update),
    [TASK_UTIL] = CREATE_TASK("UTIL", TASK_MASK_ALWAYS, TASK_PRIORITY_HIGH, util_task),
    [TASK_BLACKBOX] = CREATE_TASK("BLACKBOX", TASK_MASK_ALWAYS, TASK_PRIORITY_MEDIUM, blackbox_update),
    [TASK_OSD] = CREATE_TASK("OSD", TASK_MASK_ALWAYS, TASK_PRIORITY_MEDIUM, osd_display),
    [TASK_VTX] = CREATE_TASK("VTX", TASK_MASK_ON_GROUND, TASK_PRIORITY_LOW, vtx_update),
    [TASK_USB] = CREATE_TASK("USB", TASK_MASK_ON_GROUND, TASK_PRIORITY_LOW, usb_configurator),
};