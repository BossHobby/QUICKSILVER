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
#include "io/rgb_led.h"
#include "io/usb_configurator.h"
#include "io/vbat.h"
#include "io/vtx.h"
#include "osd/render.h"
#include "profile.h"
#include "project.h"
#include "rx/rx.h"

void util_task() {
  // handle led commands
  led_update();
  rgb_led_update();

  buzzer_update();
}

FAST_RAM task_t tasks[TASK_MAX] = {
    [TASK_GYRO] = CREATE_TASK("GYRO", TASK_MASK_ALWAYS, TASK_PRIORITY_REALTIME, sixaxis_read, 0),
    [TASK_IMU] = CREATE_TASK("IMU", TASK_MASK_ALWAYS, TASK_PRIORITY_REALTIME, imu_calc, 0),
    [TASK_PID] = CREATE_TASK("PID", TASK_MASK_ALWAYS, TASK_PRIORITY_REALTIME, control, 0),
    [TASK_RX] = CREATE_TASK("RX", TASK_MASK_ALWAYS, TASK_PRIORITY_REALTIME, rx_update, 0),
    [TASK_VBAT] = CREATE_TASK("VBAT", TASK_MASK_ALWAYS, TASK_PRIORITY_HIGH, vbat_calc, 1000),
    [TASK_UTIL] = CREATE_TASK("UTIL", TASK_MASK_ALWAYS, TASK_PRIORITY_HIGH, util_task, 1000),
    [TASK_GESTURES] = CREATE_TASK("GESTURES", TASK_MASK_ON_GROUND, TASK_PRIORITY_MEDIUM, gestures, 0),
    [TASK_BLACKBOX] = CREATE_TASK("BLACKBOX", TASK_MASK_ALWAYS, TASK_PRIORITY_MEDIUM, blackbox_update, 0),
    [TASK_OSD] = CREATE_TASK("OSD", TASK_MASK_ALWAYS, TASK_PRIORITY_MEDIUM, osd_display, 8000),
    [TASK_VTX] = CREATE_TASK("VTX", TASK_MASK_ON_GROUND, TASK_PRIORITY_LOW, vtx_update, 0),
    [TASK_USB] = CREATE_TASK("USB", TASK_MASK_ON_GROUND, TASK_PRIORITY_LOW, usb_configurator, 0),
};