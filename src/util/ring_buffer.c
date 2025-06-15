#include "ring_buffer.h"

#include <string.h>

#include "driver/interrupt.h"

static uint32_t ring_buffer_contiguous_write_space(ring_buffer_t *c) {
  if (c->head >= c->tail) {
    uint32_t space_to_end = c->size - c->head;
    return (c->tail == 0) ? space_to_end - 1 : space_to_end;
  }
  return c->tail - c->head - 1;
}

static uint32_t ring_buffer_contiguous_read_space(ring_buffer_t *c) {
  if (c->tail >= c->head) {
    return c->size - c->tail;
  }
  return c->head - c->tail;
}

static uint32_t ring_buffer_write_chunk(ring_buffer_t *c, const uint8_t *data, uint32_t len, uint32_t max_space) {
  uint32_t contiguous = ring_buffer_contiguous_write_space(c);
  uint32_t to_write = len;
  
  if (to_write > contiguous) {
    to_write = contiguous;
  }
  if (to_write > max_space) {
    to_write = max_space;
  }
  if (to_write == 0) {
    return 0;
  }
  
  memcpy(&c->buffer[c->head], data, to_write);
  c->head = (c->head + to_write) % c->size;
  return to_write;
}

static uint32_t ring_buffer_read_chunk(ring_buffer_t *c, uint8_t *data, uint32_t len, uint32_t max_available) {
  uint32_t contiguous = ring_buffer_contiguous_read_space(c);
  uint32_t to_read = len;
  
  if (to_read > contiguous) {
    to_read = contiguous;
  }
  if (to_read > max_available) {
    to_read = max_available;
  }
  if (to_read == 0) {
    return 0;
  }
  
  memcpy(data, &c->buffer[c->tail], to_read);
  c->tail = (c->tail + to_read) % c->size;
  return to_read;
}

uint32_t ring_buffer_free(ring_buffer_t *c) {
  const uint32_t head = c->head;
  const uint32_t tail = c->tail;
  
  if (head >= tail) {
    return (c->size - 1) - (head - tail);
  }
  return (tail - head) - 1;
}

uint8_t ring_buffer_write(ring_buffer_t *c, uint8_t data) {
  const uint32_t next = (c->head + 1) % c->size;
  if (next == c->tail)
    return 0;

  c->buffer[c->head] = data;
  c->head = next;
  return 1;
}

uint32_t ring_buffer_write_multi(ring_buffer_t *c, const uint8_t *data, const uint32_t len) {
  if (!data || len == 0) {
    return 0;
  }
  
  if (len == 1) {
    return ring_buffer_write(c, data[0]);
  }
  
  uint32_t written = 0;
  uint32_t remaining_space = ring_buffer_free(c);
  
  while (written < len && remaining_space > 0) {
    uint32_t chunk_size = ring_buffer_write_chunk(c, &data[written], 
                                                 len - written, remaining_space);
    if (chunk_size == 0) {
      break;
    }
    
    written += chunk_size;
    remaining_space -= chunk_size;
  }
  
  return written;
}

uint32_t ring_buffer_available(ring_buffer_t *c) {
  const uint32_t head = c->head;
  const uint32_t tail = c->tail;
  
  if (head >= tail) {
    return head - tail;
  }
  return c->size + head - tail;
}

uint8_t ring_buffer_read(ring_buffer_t *c, uint8_t *data) {
  if (!data) {
    return 0;
  }
  
  if (c->head == c->tail)
    return 0;

  *data = c->buffer[c->tail];
  c->tail = (c->tail + 1) % c->size;
  return 1;
}

uint32_t ring_buffer_read_multi(ring_buffer_t *c, uint8_t *data, const uint32_t len) {
  if (!data || len == 0) {
    return 0;
  }
  
  if (len == 1) {
    return ring_buffer_read(c, data);
  }
  
  uint32_t read_count = 0;
  uint32_t remaining_data = ring_buffer_available(c);
  
  while (read_count < len && remaining_data > 0) {
    uint32_t chunk_size = ring_buffer_read_chunk(c, &data[read_count], 
                                                len - read_count, remaining_data);
    if (chunk_size == 0) {
      break;
    }
    
    read_count += chunk_size;
    remaining_data -= chunk_size;
  }
  
  return read_count;
}

void ring_buffer_clear(ring_buffer_t *c) {
  ATOMIC_BLOCK_ALL {
    c->tail = c->head = 0;
  }
}
