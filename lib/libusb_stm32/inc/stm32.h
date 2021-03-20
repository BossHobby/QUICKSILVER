#ifndef _STM32_H_
#define _STM32_H_

/* modify bitfield */
#define _BMD(reg, msk, val) (reg) = (((reg) & ~(msk)) | (val))
/* set bitfield */
#define _BST(reg, bits) (reg) = ((reg) | (bits))
/* clear bitfield */
#define _BCL(reg, bits) (reg) = ((reg) & ~(bits))
/* wait until bitfield set */
#define _WBS(reg, bits) while (((reg) & (bits)) == 0)
/* wait until bitfield clear */
#define _WBC(reg, bits) while (((reg) & (bits)) != 0)
/* wait for bitfield value */
#define _WVL(reg, msk, val) while (((reg) & (msk)) != (val))
/* bit value */
#define _BV(bit) (0x01 << (bit))

#if defined(STM32F0)
#include "stm32f0xx.h"
#elif defined(STM32F1)
#include "stm32f1xx.h"
#elif defined(STM32F2)
#include "stm32f2xx.h"
#elif defined(STM32F3)
#include "stm32f3xx.h"
#elif defined(STM32F4)
#include "stm32f4xx.h"
#elif defined(STM32F7)
#include "stm32f7xx.h"
#elif defined(STM32H7)
#include "stm32h7xx.h"
#elif defined(STM32L0)
#include "stm32l0xx.h"
#elif defined(STM32L1)
#include "stm32l1xx.h"
#elif defined(STM32L4)
#include "stm32l4xx.h"
#elif defined(STM32L5)
#include "stm32l5xx.h"
#elif defined(STM32G0)
#include "stm32g0xx.h"
#elif defined(STM32G4)
#include "stm32g4xx.h"
#else
#error "STM32 family not defined"
#endif

#endif // _STM32_H_