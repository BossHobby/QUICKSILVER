#pragma once

#ifdef STM32H7
// H7 factory VREF calibration is 16-bit, while the ADC is configured for 12-bit reads.
#define VREFINT_CAL ((*(VREFINT_CAL_ADDR)) >> 4)
#else
#define VREFINT_CAL (*(VREFINT_CAL_ADDR))
#endif
