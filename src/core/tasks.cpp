#include "tasks.h"

#include <stddef.h>

#include <algorithm>

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
  thread_assert_failed();
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

static uint32_t io_pending_tasks(TickType_t *remaining, TickType_t elapsed, uint32_t pending) {
  uint32_t enabled = (1u << IO_TASK_COUNT) - 1;
#ifdef USE_DIGITAL_VTX
  // DisplayPort is initialized before IO starts; OSD owns its VTX service.
  if (serial_displayport.config.port != SERIAL_PORT_INVALID) {
    enabled &= ~IO_WORK_VTX;
    remaining[IO_VTX] = portMAX_DELAY;
  }
#endif
  for (unsigned i = 0; i < IO_TASK_COUNT; i++) {
    if (remaining[i] == portMAX_DELAY)
      continue;
    remaining[i] -= std::min(remaining[i], elapsed);
    if (remaining[i] == 0)
      pending |= 1u << i;
  }
  return pending & enabled;
}

void io_thread(void *) {
  TickType_t remaining[IO_TASK_COUNT] = {};
  TickType_t wait = 0;
  TickType_t last = xTaskGetTickCount();
  while (true) {
    // Pending notifications need not block; share bursts with ready workers.
    taskYIELD();
    uint32_t pending = 0;
    xTaskNotifyWait(0, UINT32_MAX, &pending, wait);
    const TickType_t now = xTaskGetTickCount();
    pending = io_pending_tasks(remaining, now - last, pending);
    last = now;
    if (pending & IO_WORK_RX)
      remaining[IO_RX] = rx_update();
    if (pending & IO_WORK_VBAT)
      remaining[IO_VBAT] = vbat_calc();
    if (pending & IO_WORK_LED)
      remaining[IO_LED] = led_update();
    if (pending & IO_WORK_RGB)
      remaining[IO_RGB] = rgb_led_update();
    if (pending & IO_WORK_BUZZER)
      remaining[IO_BUZZER] = buzzer_update();
    if (pending & IO_WORK_BARO)
      remaining[IO_BARO] = baro_update();
    if (pending & IO_WORK_VTX)
      remaining[IO_VTX] = vtx_update();
    if (pending & IO_WORK_GPS)
      remaining[IO_GPS] = gps_task();
    const TickType_t runtime = xTaskGetTickCount() - now;
    const TickType_t next = *std::min_element(remaining, remaining + IO_TASK_COUNT);
    wait = next == portMAX_DELAY ? portMAX_DELAY : std::max<TickType_t>(1, next - std::min(next, runtime));
  }
}
