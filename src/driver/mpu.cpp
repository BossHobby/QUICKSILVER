#include "driver/mpu.h"

#include "core/project.h"

#ifndef SIMULATOR
static constexpr uint32_t MPU_GUARD_REGION_FIRST = 1;

static void mpu_guard_region(uint32_t region, uintptr_t base) {
  // Execute-never, no access; memory attributes are irrelevant without access.
  ARM_MPU_SetRegion(ARM_MPU_RBAR(region, base), ARM_MPU_RASR(1, ARM_MPU_AP_NONE, 0, 0, 1, 1, 0, ARM_MPU_REGION_SIZE_32B));
}
#endif

void mpu_stack_guard_init(const uint8_t *const *guards, uint32_t count) {
#ifndef SIMULATOR
  // The linker reserves _Min_Stack_Size below _estack for the main stack;
  // rounding up keeps the guard inside that reservation.
  extern uint8_t _estack;
  extern uint8_t _Min_Stack_Size;
  const uintptr_t main_bottom = (uintptr_t)&_estack - (uintptr_t)&_Min_Stack_Size;

  ARM_MPU_Disable();
  mpu_guard_region(MPU_GUARD_REGION_FIRST, (main_bottom + MPU_GUARD_SIZE - 1) & ~(MPU_GUARD_SIZE - 1));
  for (uint32_t i = 0; i < count; i++)
    mpu_guard_region(MPU_GUARD_REGION_FIRST + 1 + i, (uintptr_t)guards[i]);
  // Privileged code (all tasks and handlers) keeps the default map elsewhere.
  ARM_MPU_Enable(MPU_CTRL_PRIVDEFENA_Msk);
  // Older CMSIS issues its barriers before enabling.
  __DSB();
  __ISB();
#endif
}
