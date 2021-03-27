#pragma once

#include <stdint.h>

typedef struct {
  uint8_t *const buffer;
  uint32_t head;
  uint32_t tail;
  const uint32_t size;
} circular_buffer_t;

uint32_t circular_buffer_free(volatile circular_buffer_t *c);
uint8_t circular_buffer_write(volatile circular_buffer_t *c, uint8_t data);
uint32_t circular_buffer_write_multi(volatile circular_buffer_t *c, const uint8_t *data, const uint32_t len);

uint32_t circular_buffer_available(volatile circular_buffer_t *c);
uint8_t circular_buffer_read(volatile circular_buffer_t *c, uint8_t *data);
uint32_t circular_buffer_read_multi(volatile circular_buffer_t *c, uint8_t *data, const uint32_t len);

void circular_buffer_clear(volatile circular_buffer_t *c);
