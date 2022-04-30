#pragma once

#include <stdint.h>

typedef struct {
  uint8_t *const buffer;
  volatile uint32_t head;
  volatile uint32_t tail;
  const uint32_t size;
} circular_buffer_t;

uint32_t circular_buffer_free(circular_buffer_t *c);
uint8_t circular_buffer_write(circular_buffer_t *c, uint8_t data);
uint32_t circular_buffer_write_multi(circular_buffer_t *c, const uint8_t *data, const uint32_t len);

uint32_t circular_buffer_available(circular_buffer_t *c);
uint8_t circular_buffer_read(circular_buffer_t *c, uint8_t *data);
uint32_t circular_buffer_read_multi(circular_buffer_t *c, uint8_t *data, const uint32_t len);

void circular_buffer_clear(circular_buffer_t *c);
