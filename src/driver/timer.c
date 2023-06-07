#include "driver/timer.h"

#define TIMER_ASSIGMENT_MAX 32

timer_assigment_t timer_assigments[TIMER_ASSIGMENT_MAX] = {};

void timer_alloc_init() {
  if (target.brushless) {
    timer_assigments[0].use = TIMER_USE_MOTOR_DSHOT;
    timer_assigments[0].tag = TIMER_TAG(TIMER1, TIMER_CH1 | TIMER_CH3 | TIMER_CH4);
  }
}

bool timer_alloc_tag(timer_use_t use, resource_tag_t tag) {
  for (uint32_t i = 0; i < TIMER_ASSIGMENT_MAX; i++) {
    timer_assigment_t *ass = &timer_assigments[i];
    if (ass->use == TIMER_USE_FREE) {
      // found free spot
      timer_assigments[i].use = use;
      timer_assigments[i].tag = tag;
      return true;
    }
    if (TIMER_TAG_TIM(ass->tag) != TIMER_TAG_TIM(tag)) {
      continue;
    }
    if (ass->use != use) {
      // timer reserved and use does not match
      return false;
    }
    if ((TIMER_TAG_CH(ass->tag) & TIMER_TAG_CH(tag)) != 0) {
      // use matches but all timer channels are used up
      return false;
    }
    // add channel to existing assigment
    ass->tag = TIMER_TAG(TIMER_TAG_TIM(ass->tag), TIMER_TAG_CH(ass->tag) | TIMER_TAG_CH(tag));
    return true;
  }

  // we ran out of assigments
  return false;
}

resource_tag_t timer_alloc(timer_use_t use) {
  for (uint8_t i = TIMER1; i < TIMER_MAX; i++) {
    const resource_tag_t tag = TIMER_TAG(i, TIMER_CH_ALL);
    if (timer_alloc_tag(use, tag)) {
      return tag;
    }
  }
  return TIMER_TAG(TIMER_INVALID, TIMER_CH_INVALID);
}