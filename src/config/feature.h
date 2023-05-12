#pragma once

#if defined(USE_CC2500)
#define RX_FRSKY
#endif

#if defined(USE_A7105)
#define RX_FLYSKY
#endif

#if defined(USE_SX128X)
#define RX_EXPRESS_LRS
#endif

//*************************************Features that still need to be moved into targets and checked for compatability************************************************
// RGB led type ws2812 - ws2813
// numbers over 8 could decrease performance
// #define RGB_LED_NUMBER 0
// #define RGB_LED_DMA

// pin / port for the RGB led ( programming port ok )
// #define RGB_PIN LL_GPIO_PIN_11
// #define RGB_PORT GPIOA