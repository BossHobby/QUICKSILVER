#pragma once

#include <stdbool.h>
#include <stdint.h>

#include <FreeRTOS.h>
#include <task.h>

#include "project.h"

#include "control/control.h"
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
  IO_RX, IO_BARO, IO_VTX, IO_GPS, IO_VBAT, IO_LED, IO_RGB, IO_BUZZER, IO_TASK_COUNT,
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
};

struct thread_t {
  const char *name;
  uint8_t mask;
  UBaseType_t priority;
  TaskFunction_t entry;
  StackType_t *stack;
  uint32_t stack_size;
  StaticTask_t control;
  TaskHandle_t handle;
};

#define CREATE_THREAD(p_name, p_mask, p_priority, p_entry, p_stack) \
  {p_name, p_mask, p_priority, p_entry, p_stack, sizeof(p_stack) / sizeof(p_stack[0]), {}, nullptr}

extern thread_t threads[THREAD_MAX];

void flight_thread(void *);
void io_thread(void *);
void thread_start(thread_id_t id);
// Called by Flight between passes, with ground configuration ownership held.
void threads_update();
bool flight_timer_irq_handler();
void flight_reset_runtime();
