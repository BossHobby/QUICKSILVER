#include <sys/types.h>

register char *stack_ptr asm("sp");
uint __heap_limit = 0xcafedead;

__attribute__((__used__)) void *_sbrk(ptrdiff_t incr) {
  extern char end asm("end"); /* Defined by the linker.  */
  static char *heap_end;

  if (heap_end == NULL)
    heap_end = &end;

  const char *prev_heap_end = heap_end;

  if ((heap_end + incr > stack_ptr) ||
      (__heap_limit != 0xcafedead && heap_end + incr > (char *)__heap_limit)) {
    return (void *)-1;
  }

  heap_end += incr;
  return (void *)prev_heap_end;
}
