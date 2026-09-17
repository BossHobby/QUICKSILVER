#include "driver/timer.h"

#include <FreeRTOS.h>
#include <task.h>

#include "core/tasks.h"

const timer_def_t timer_defs[TIMER_MAX] = {};

static struct {
  uint64_t period;
  uint64_t reload;
  uint64_t elapsed;
  uint16_t divider;
  bool running;
  bool pending;
} timers[TIMER_MAX];

void timer_up_init(timer_index_t tim, uint16_t divider, uint32_t period) {
  taskENTER_CRITICAL();
  timers[tim] = {};
  timers[tim].divider = divider;
  timers[tim].period = timers[tim].reload = ((uint64_t)period + 1) * divider;
  taskEXIT_CRITICAL();
}

void timer_up_set_period(timer_index_t tim, uint32_t period) {
  taskENTER_CRITICAL();
  timers[tim].reload = ((uint64_t)period + 1) * timers[tim].divider;
  taskEXIT_CRITICAL();
}

uint32_t timer_up_count(timer_index_t tim) {
  return timers[tim].elapsed / timers[tim].divider;
}

void timer_up_start(timer_index_t tim) {
  taskENTER_CRITICAL();
  timers[tim].period = timers[tim].reload;
  timers[tim].elapsed = 0;
  timers[tim].pending = false;
  timers[tim].running = true;
  taskEXIT_CRITICAL();
}

bool timer_up_pending(timer_index_t tim) {
  if (!timers[tim].pending) return false;
  timers[tim].pending = false;
  return true;
}

extern "C" void vApplicationTickHook() {
  // Native expirations are tick-granular. Coalesce elapsed periods like an
  // MCU update flag instead of replaying a backlog of simulated interrupts.
  for (auto &timer : timers) {
    if (!timer.running) continue;
    timer.elapsed += PWM_CLOCK_FREQ_HZ / configTICK_RATE_HZ;
    if (timer.elapsed >= timer.period) {
      timer.elapsed -= timer.period;
      timer.period = timer.reload;
      timer.elapsed %= timer.period;
      timer.pending = true;
    }
  }
  flight_timer_irq_handler();
  // Task notifications mark a pending yield; the POSIX tick ISR switches tasks
  // after xTaskIncrementTick() returns. Do not switch inside the tick hook.
}
