#pragma once

#include <stdint.h>

// This Ring Buffer _should_ be interrupt-safe given a single consumer/producer scenario.
// Monitor usage closely to ensure above condition is true. All internal blocking has been removed.
typedef struct {
  uint8_t *const buffer;
  volatile uint32_t head;
  volatile uint32_t tail;
  const uint32_t size;
} ring_buffer_t;

uint32_t ring_buffer_free(ring_buffer_t *c);
uint8_t ring_buffer_write(ring_buffer_t *c, uint8_t data);
uint32_t ring_buffer_write_multi(ring_buffer_t *c, const uint8_t *data, const uint32_t len);

uint32_t ring_buffer_available(ring_buffer_t *c);
uint8_t ring_buffer_read(ring_buffer_t *c, uint8_t *data);
uint32_t ring_buffer_read_multi(ring_buffer_t *c, uint8_t *data, const uint32_t len);

// only function with internal blocking as both head & tail are written to
void ring_buffer_clear(ring_buffer_t *c);
