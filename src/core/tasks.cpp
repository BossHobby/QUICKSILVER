#include "tasks.h"

#include <stddef.h>

#include "control/control.h"
#include "control/gestures.h"
#include "control/imu.h"
#ifdef VEHICLE_MULTI
#include "control/multi/navigation.h"
#endif
#include "control/sixaxis.h"
#include "core/scheduler.h"
#include "driver/baro/baro.h"
#include "driver/serial.h"
#include "driver/usb.h"
#include "io/blackbox.h"
#include "io/buzzer.h"
#include "io/gps.h"
#include "io/led.h"
#include "io/rgb_led.h"
#include "io/usb_configurator.h"
#include "io/vbat.h"
#include "io/vtx.h"
#include "osd/render.h"
#include "profile.h"
#include "project.h"
#include "rx/rx.h"

static StackType_t flight_stack[2048];

static void flight_task() {
  sixaxis_read();
  imu_calc();
  rx_process();
  control();
}

void util_task() {
  // handle led commands
  led_update();
  rgb_led_update();

  buzzer_update();
}

#ifndef VEHICLE_MULTI
static void task_noop() {
}
#endif

static void flight_thread(void *) {
  task_reset_runtime();
  while (1) {
    scheduler_run();
  }
}

extern "C" void thread_assert_failed() {
  failloop(FAILLOOP_FAULT);
}

extern "C" void vApplicationStackOverflowHook(TaskHandle_t, char *) {
  thread_assert_failed();
}

thread_t threads[THREAD_MAX] = {
    [THREAD_FLIGHT] = CREATE_THREAD("flight", 1, flight_thread, flight_stack),
};

void threads_start() {
  for (auto &thread : threads) {
    thread.handle = xTaskCreateStatic(thread.entry, thread.name, thread.stack_size, nullptr, thread.priority, thread.stack, &thread.control);
    configASSERT(thread.handle);
  }
  vTaskStartScheduler();
  thread_assert_failed();
}

FAST_RAM task_t tasks[TASK_MAX] = {
    [TASK_FLIGHT] = CREATE_TASK("FLIGHT", TASK_MASK_ALWAYS, TASK_PRIORITY_REALTIME, flight_task, 0),
    [TASK_RX] = CREATE_TASK("RX", TASK_MASK_ALWAYS, TASK_PRIORITY_HIGH, rx_update, 0),
    [TASK_VBAT] = CREATE_TASK("VBAT", TASK_MASK_ALWAYS, TASK_PRIORITY_HIGH, vbat_calc, 1000),
    [TASK_BARO] = CREATE_TASK("BARO", TASK_MASK_ALWAYS, TASK_PRIORITY_HIGH, baro_update, 10000),
#ifdef VEHICLE_MULTI
    [TASK_NAV] = CREATE_TASK("NAV", TASK_MASK_ALWAYS, TASK_PRIORITY_HIGH, nav_update, 10000),
#else
    [TASK_NAV] = CREATE_TASK("NAV", 0, TASK_PRIORITY_HIGH, task_noop, 0),
#endif
    [TASK_UTIL] = CREATE_TASK("UTIL", TASK_MASK_ALWAYS, TASK_PRIORITY_HIGH, util_task, 1000),
    [TASK_GESTURES] = CREATE_TASK("GESTURES", TASK_MASK_ON_GROUND, TASK_PRIORITY_MEDIUM, gestures, 0),
    [TASK_BLACKBOX] = CREATE_TASK("BLACKBOX", TASK_MASK_ALWAYS, TASK_PRIORITY_MEDIUM, blackbox_update, 0),
    [TASK_OSD] = CREATE_TASK("OSD", TASK_MASK_ALWAYS, TASK_PRIORITY_MEDIUM, osd_display, 1000),
    [TASK_VTX] = CREATE_TASK("VTX", TASK_MASK_ON_GROUND, TASK_PRIORITY_LOW, vtx_update, 0),
    [TASK_USB] = CREATE_TASK("USB", TASK_MASK_ON_GROUND, TASK_PRIORITY_LOW, usb_configurator, 0),
    [TASK_GPS] = CREATE_TASK("GPS", TASK_MASK_ALWAYS, TASK_PRIORITY_LOW, gps_task, 5000),
};
