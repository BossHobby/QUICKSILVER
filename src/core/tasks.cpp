#include "tasks.h"

#include <stddef.h>

#include "control/control.h"
#include "control/gestures.h"
#include "control/imu.h"
#ifdef VEHICLE_MULTI
#include "control/multi/navigation.h"
#endif
#include "control/sixaxis.h"
#include "core/failloop.h"
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

static void flight_task() {
  sixaxis_read();
  imu_calc();
  rx_process();
  control();
  blackbox_capture();
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

// Cortex-M budgets include callers and saved task context. DisplayPort uses a
// ~2 KiB frame and Blackbox encoding ~600 B; USB frame buffers live on the heap.
static FAST_RAM StackType_t flight_stack[1024]; // 4 KiB
static StackType_t blackbox_stack[512];        // 2 KiB
static StackType_t usb_stack[512];             // 2 KiB

extern "C" void thread_assert_failed() {
  failloop(FAILLOOP_FAULT);
}

extern "C" void vApplicationStackOverflowHook(TaskHandle_t, char *) {
  thread_assert_failed();
}

thread_t threads[THREAD_MAX] = {
    [THREAD_FLIGHT] = CREATE_THREAD("flight", TASK_MASK_ALWAYS, 2, flight_thread, flight_stack),
    [THREAD_BLACKBOX] = CREATE_THREAD("blackbox", TASK_MASK_ALWAYS, 1, blackbox_thread, blackbox_stack),
    [THREAD_USB] = CREATE_THREAD("usb", TASK_MASK_ON_GROUND, 1, usb_configurator_thread, usb_stack),
};

void thread_start(thread_id_t id) {
  auto &thread = threads[id];
  configASSERT(thread.handle == nullptr);
  thread.handle = xTaskCreateStatic(thread.entry, thread.name, thread.stack_size, nullptr, thread.priority, thread.stack, &thread.control);
  configASSERT(thread.handle);
}

void threads_update() {
  const uint8_t mask = TASK_MASK_DEFAULT | ((flags.arm_state || flags.in_air) ? TASK_MASK_IN_AIR : TASK_MASK_ON_GROUND);
  for (auto &thread : threads) {
    if (thread.handle == nullptr) {
      continue;
    }
    const bool suspended = !(thread.mask & mask);
    if (suspended == (eTaskGetState(thread.handle) == eSuspended)) {
      continue;
    }
    if (suspended) {
      vTaskSuspend(thread.handle);
    } else {
      vTaskResume(thread.handle);
    }
  }
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
    [TASK_OSD] = CREATE_TASK("OSD", TASK_MASK_ALWAYS, TASK_PRIORITY_MEDIUM, osd_display, 1000),
    [TASK_VTX] = CREATE_TASK("VTX", TASK_MASK_ON_GROUND, TASK_PRIORITY_LOW, vtx_update, 0),
    [TASK_GPS] = CREATE_TASK("GPS", TASK_MASK_ALWAYS, TASK_PRIORITY_LOW, gps_task, 5000),
};
