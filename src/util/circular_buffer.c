#include "circular_buffer.h"

#include "drv_interrupt.h"

uint32_t circular_buffer_free(circular_buffer_t *c) {
  ATOMIC_BLOCK(MAX_PRIORITY) {
    if (c->head >= c->tail) {
      return (c->size - c->head) + c->tail;
    }
    return (c->tail - c->head);
  }
  return 0;
}

uint8_t circular_buffer_write(circular_buffer_t *c, uint8_t data) {
  ATOMIC_BLOCK(MAX_PRIORITY) {
    const uint32_t next = (c->head + 1) % c->size;
    if (next == c->tail)
      return 0;

    c->buffer[c->head] = data;
    c->head = next;
    return 1;
  }
  return 0;
}

uint32_t circular_buffer_write_multi(circular_buffer_t *c, const uint8_t *data, const uint32_t len) {
  for (uint32_t i = 0; i < len; i++) {
    ATOMIC_BLOCK(MAX_PRIORITY) {
      const uint32_t next = (c->head + 1) % c->size;
      if (next == c->tail)
        return i;

      c->buffer[c->head] = data[i];
      c->head = next;
    }
  }
  return len;
}

uint32_t circular_buffer_available(circular_buffer_t *c) {
  ATOMIC_BLOCK(MAX_PRIORITY) {
    if (c->head >= c->tail) {
      return c->head - c->tail;
    }
    return c->size + c->head - c->tail;
  }
  return 0;
}

uint8_t circular_buffer_read(circular_buffer_t *c, uint8_t *data) {
  ATOMIC_BLOCK(MAX_PRIORITY) {
    if (c->head == c->tail)
      return 0;

    *data = c->buffer[c->tail];
    c->tail = (c->tail + 1) % c->size;
    return 1;
  }
  return 0;
}

uint32_t circular_buffer_read_multi(circular_buffer_t *c, uint8_t *data, const uint32_t len) {
  for (uint32_t i = 0; i < len; i++) {
    ATOMIC_BLOCK(MAX_PRIORITY) {
      if (c->head == c->tail)
        return i;

      data[i] = c->buffer[c->tail];
      c->tail = (c->tail + 1) % c->size;
    }
  }
  return len;
}

void circular_buffer_clear(circular_buffer_t *c) {
  c->tail = c->head;
}
