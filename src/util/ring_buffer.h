#pragma once

#include <stdint.h>

// This Ring Buffer implementation is interrupt-safe for single producer/single consumer scenarios.
// 
// Safety guarantees:
// - One context writes (producer), one context reads (consumer)
// - Head is only modified by producer, tail only by consumer
// - ring_buffer_free() and ring_buffer_available() take atomic snapshots of volatile pointers
// - These functions may return slightly stale values if called during concurrent access,
//   but the values are always valid and safe (conservative for buffer overflow prevention)
// 
// Typical usage:
// - ISR writes to RX buffer, main reads from RX buffer
// - Main writes to TX buffer, ISR reads from TX buffer
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
