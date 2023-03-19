#pragma once

#ifdef BRUSHLESS_TARGET
// dshot pin initialization & usb interface to esc
#define USE_DSHOT_DMA_DRIVER
#define USE_SERIAL_4WAY_BLHELI_INTERFACE
#endif

#ifdef BRUSHED_TARGET
// pwm pin initialization
#define USE_PWM_DRIVER
#endif

#if defined(USE_M25P16) || defined(USE_SDCARD)
#define ENABLE_BLACKBOX
#endif

#if defined(USE_CC2500)
#define RX_FRSKY
#endif

#if defined(USE_A7105)
#define RX_FLYSKY
#endif

#if defined(USE_SX128X)
#define RX_EXPRESS_LRS
#endif

#if defined(BUZZER_ENABLE) && !defined(BUZZER_PIN)
#undef BUZZER_ENABLE
#endif

//*************************************Features that still need to be moved into targets and checked for compatability************************************************
// RGB led type ws2812 - ws2813
// numbers over 8 could decrease performance
// #define RGB_LED_NUMBER 0
// #define RGB_LED_DMA

// pin / port for the RGB led ( programming port ok )
// #define RGB_PIN LL_GPIO_PIN_11
// #define RGB_PORT GPIOA
