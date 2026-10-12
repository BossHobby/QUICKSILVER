#pragma once

#include <stdbool.h>
#include <stdint.h>

#include <FreeRTOS.h>
#include <task.h>

#include "project.h"

#include "control/control.h"
#include "driver/mpu.h"
#include "driver/time.h"

typedef enum {
  TASK_MASK_DEFAULT = (0x1 << 0),
  TASK_MASK_ON_GROUND = (0x1 << 1),
  TASK_MASK_IN_AIR = (0x1 << 2),

  TASK_MASK_ALWAYS = 0xFF,
} task_mask_t;

enum thread_id_t {
  THREAD_FLIGHT,
  THREAD_BLACKBOX,
  THREAD_USB,
  THREAD_OSD,
  THREAD_IO,
  THREAD_MAX,
};

enum io_task_t {
  IO_RX,
  IO_BARO,
  IO_VTX,
  IO_GPS,
  IO_VBAT,
  IO_LED,
  IO_RGB,
  IO_BUZZER,
  IO_PINIO,
  IO_TASK_COUNT,
};

enum io_work_t : uint32_t {
  IO_WORK_RX = 1u << IO_RX,
  IO_WORK_BARO = 1u << IO_BARO,
  IO_WORK_VTX = 1u << IO_VTX,
  IO_WORK_GPS = 1u << IO_GPS,
  IO_WORK_VBAT = 1u << IO_VBAT,
  IO_WORK_LED = 1u << IO_LED,
  IO_WORK_RGB = 1u << IO_RGB,
  IO_WORK_BUZZER = 1u << IO_BUZZER,
  IO_WORK_PINIO = 1u << IO_PINIO,
};

// The MPU guard sits directly below the stack, so a task overflowing its stack
// faults instead of corrupting neighbouring memory.
template <uint32_t N>
struct alignas(MPU_GUARD_SIZE) thread_stack_t {
  uint8_t guard[MPU_GUARD_SIZE];
  StackType_t stack[N];
};

struct thread_t {
  const char *name;
  uint8_t mask;
  UBaseType_t priority;
  TaskFunction_t entry;
  const uint8_t *guard;
  StackType_t *stack;
  uint32_t stack_size;
  StaticTask_t control;
  TaskHandle_t handle;
};

#define CREATE_THREAD(p_name, p_mask, p_priority, p_entry, p_stack) \
  {p_name, p_mask, p_priority, p_entry, p_stack.guard, p_stack.stack, sizeof(p_stack.stack) / sizeof(p_stack.stack[0]), {}, nullptr}

extern thread_t threads[THREAD_MAX];

void flight_thread(void *);
void io_thread(void *);
// Called once before any thread starts.
void thread_stack_guard_init();
void thread_start(thread_id_t id);
// Called by Flight between passes, with ground configuration ownership held.
void threads_update();
bool flight_timer_irq_handler();
void flight_reset_runtime();
