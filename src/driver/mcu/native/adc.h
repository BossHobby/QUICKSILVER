#pragma once

#define VREFINT_CAL (1489)
#define VREFINT_CAL_VREF (3300)

// Converts every configured channel once, as one hardware scan would. The tick
// hook calls it as the native conversion interrupt.
void adc_native_scan();
