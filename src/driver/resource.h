#pragma once

#include <stdint.h>

typedef enum {
  RESOURCE_INVALID,
  RESOURCE_TIM,
  RESOURCE_SPI,
  RESOURCE_SERIAL,
  RESOURCE_ADC,
} resource_type_t;

typedef enum {
  RES_SERIAL_INVALID,
  RES_SERIAL_RX,
  RES_SERIAL_TX,
} resource_serial_t;

typedef enum {
  RES_SPI_INVALID,
  RES_SPI_MOSI,
  RES_SPI_MISO,
  RES_SPI_SCK,
} resource_spi_t;

typedef uint32_t resource_tag_t;

#define RESOURCE_TAG(typ, val) ((uint32_t)((typ) << 24) | (uint32_t)(val))
#define RESOURCE_TAG_TYPE(tag) (resource_type_t)(((tag) >> 24) & 0xFF)

#define TIMER_TAG(tim, ch) RESOURCE_TAG(RESOURCE_TIM, (uint32_t)((tim) << 8) | (ch))
#define TIMER_TAG_TIM(tag) (uint8_t)(((tag) >> 8) & 0xFF)
#define TIMER_TAG_CH(tag) (uint8_t)((tag)&0xFF)

#define SPI_TAG(port, pin) RESOURCE_TAG(RESOURCE_SPI, (uint32_t)((port) << 8) | (pin))
#define SPI_TAG_PORT(tag) (uint8_t)(((tag) >> 8) & 0xFF)
#define SPI_TAG_PIN(tag) (uint8_t)((tag)&0xFF)

#define SERIAL_TAG(serial, pin) RESOURCE_TAG(RESOURCE_SERIAL, (uint32_t)((serial) << 8) | (pin))
#define SERIAL_TAG_PORT(tag) (uint8_t)(((tag) >> 8) & 0xFF)
#define SERIAL_TAG_PIN(tag) (uint8_t)((tag)&0xFF)

#define ADC_TAG(adc, ch) RESOURCE_TAG(RESOURCE_ADC, (uint32_t)((adc) << 8) | (ch))
#define ADC_TAG_DEV(tag) (uint8_t)(((tag) >> 8) & 0xFF)
#define ADC_TAG_CH(tag) (uint8_t)((tag)&0xFF)