#include <sys/types.h>

#ifndef SIMULATOR

extern uint8_t _heap_start;
extern uint8_t _heap_end;

__attribute__((__used__)) void *_sbrk(ptrdiff_t incr) {
  static uint8_t *heap_end = &_heap_end;

  const uint8_t *prev_heap_end = heap_end;
  if ((heap_end + incr > &_heap_start)) {
    return (void *)-1;
  }
  heap_end += incr;

  return (void *)prev_heap_end;
}

#endif
