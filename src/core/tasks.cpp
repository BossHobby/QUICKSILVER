#include "tasks.h"

#include <stddef.h>

#include "control/control.h"
#include "core/failloop.h"
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

// Cortex-M budgets include callers and saved task context. DisplayPort uses a
// ~2 KiB frame and Blackbox encoding ~600 B; USB frame buffers live on the heap.
static FAST_RAM StackType_t flight_stack[1024]; // 4 KiB
static StackType_t blackbox_stack[512];        // 2 KiB
static StackType_t usb_stack[512];             // 2 KiB
static StackType_t osd_stack[1024];            // 4 KiB, including DisplayPort frames
static StackType_t io_stack[1024];             // 4 KiB: RX MSP dispatch and VTX frames.

extern "C" void thread_assert_failed() {
  failloop(FAILLOOP_FAULT);
}

extern "C" void vApplicationStackOverflowHook(TaskHandle_t, char *) {
  failloop(FAILLOOP_FAULT);
}

thread_t threads[THREAD_MAX] = {
    [THREAD_FLIGHT] = CREATE_THREAD("flight", TASK_MASK_ALWAYS, 2, flight_thread, flight_stack),
    [THREAD_BLACKBOX] = CREATE_THREAD("blackbox", TASK_MASK_ALWAYS, 1, blackbox_thread, blackbox_stack),
    [THREAD_USB] = CREATE_THREAD("usb", TASK_MASK_ON_GROUND, 1, usb_configurator_thread, usb_stack),
    [THREAD_OSD] = CREATE_THREAD("osd", TASK_MASK_ALWAYS, 1, osd_thread, osd_stack),
    [THREAD_IO] = CREATE_THREAD("io", TASK_MASK_ALWAYS, 1, io_thread, io_stack),
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

void io_thread(void *) {
  uint32_t last_baro = time_micros();
  uint32_t last_gps = last_baro;
  while (true) {
    // Configuration commands own their locks; receiving channels does not.
    rx_update();
    vbat_calc();
    const uint32_t now = time_micros();
    if (state.baro_detected && now - last_baro >= 10000) {
      last_baro = now;
      baro_update();
    }
    led_update();
    rgb_led_update();
    buzzer_update();
#ifdef USE_DIGITAL_VTX
    if (serial_displayport.config.port == SERIAL_PORT_INVALID)
#endif
    if (!flags.arm_state && !flags.in_air)
      vtx_update();
    if (profile.serial.gps != SERIAL_PORT_INVALID && now - last_gps >= 5000) {
      last_gps = now;
      gps_task();
    }
    vTaskDelay(1);
  }
}
