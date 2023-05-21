#pragma once

#define RX_FRSKY
#define RX_FLYSKY
#define RX_EXPRESS_LRS

//*************************************Features that still need to be moved into targets and checked for compatability************************************************
// RGB led type ws2812 - ws2813
// numbers over 8 could decrease performance
// #define RGB_LED_NUMBER 0
// #define RGB_LED_DMA

// pin / port for the RGB led ( programming port ok )
// #define RGB_PIN LL_GPIO_PIN_11
// #define RGB_PORT GPIOA