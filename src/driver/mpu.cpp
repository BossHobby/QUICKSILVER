#include "driver/mpu.h"

#include "core/project.h"

#ifndef SIMULATOR
static constexpr uint32_t MPU_GUARD_REGION_FIRST = 1;

#if defined(__ARM_ARCH_8M_MAIN__)
static constexpr uint32_t MPU_GUARD_ATTR = 0;

// ARMv8-M has no privileged no-access permission: the guard is read-only and
// execute-never, so the write that overruns a stack faults.
static void mpu_guard_region(uint32_t region, uintptr_t base) {
  ARM_MPU_SetRegion(region, ARM_MPU_RBAR(base, ARM_MPU_SH_NON, 1, 0, 1), ARM_MPU_RLAR(base + MPU_GUARD_SIZE - 1, MPU_GUARD_ATTR));
}
#else
static void mpu_guard_region(uint32_t region, uintptr_t base) {
  // Execute-never, no access; memory attributes are irrelevant without access.
  ARM_MPU_SetRegion(ARM_MPU_RBAR(region, base), ARM_MPU_RASR(1, ARM_MPU_AP_NONE, 0, 0, 1, 1, 0, ARM_MPU_REGION_SIZE_32B));
}
#endif

#ifdef USE_FAST_CODE
#ifdef USE_DMA_RAM
#error "region 0 maps either DMA RAM or ITCM code"
#endif
static constexpr uint32_t MPU_ITCM_REGION = 0;

// The 16 KiB ITCM starts at address 0: without this, a write through a null
// pointer would silently patch the Flight code running there. TCM accesses
// bypass the cache, so the attributes only need to describe normal memory.
static void mpu_itcm_region() {
  ARM_MPU_SetRegion(ARM_MPU_RBAR(MPU_ITCM_REGION, 0x00000000), ARM_MPU_RASR(0, ARM_MPU_AP_RO, 1, 0, 0, 0, 0, ARM_MPU_REGION_SIZE_16KB));
}
#endif
#endif

void mpu_init(const uint8_t *const *guards, uint32_t count) {
#ifndef SIMULATOR
  // The linker reserves _Min_Stack_Size below _estack for the main stack;
  // rounding up keeps the guard inside that reservation.
  extern uint8_t _estack;
  extern uint8_t _Min_Stack_Size;
  const uintptr_t main_bottom = (uintptr_t)&_estack - (uintptr_t)&_Min_Stack_Size;

  ARM_MPU_Disable();
#if defined(__ARM_ARCH_8M_MAIN__)
  // Normal, non-cacheable memory for the guard regions.
  ARM_MPU_SetMemAttr(MPU_GUARD_ATTR, ARM_MPU_ATTR(ARM_MPU_ATTR_NON_CACHEABLE, ARM_MPU_ATTR_NON_CACHEABLE));
#endif
#ifdef USE_FAST_CODE
  // memory_section_init() copied the code before main().
  mpu_itcm_region();
#endif
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
