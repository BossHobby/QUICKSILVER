#pragma once

#include <FreeRTOS.h>
#include <semphr.h>

// Scoped ownership of a non-recursive task mutex. May block; never use in ISRs.
class mutex_guard_t {
  SemaphoreHandle_t mutex;

public:
  explicit mutex_guard_t(SemaphoreHandle_t mutex, bool enabled = true) : mutex(enabled ? mutex : nullptr) {
    if (!enabled)
      return;
    const BaseType_t acquired = xSemaphoreTake(mutex, portMAX_DELAY);
    configASSERT(acquired == pdTRUE);
  }

  ~mutex_guard_t() {
    if (mutex == nullptr)
      return;
    const BaseType_t released = xSemaphoreGive(mutex);
    configASSERT(released == pdTRUE);
  }

  mutex_guard_t(const mutex_guard_t &) = delete;
  mutex_guard_t &operator=(const mutex_guard_t &) = delete;
};
