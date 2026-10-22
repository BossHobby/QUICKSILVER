#pragma once

#include <stdbool.h>
#include <stdint.h>

// UTC wall clock, anchored to an external time source and free-running on
// time_millis() in between. GPS sets the anchor from the IO worker; CRSF (IO)
// and Blackbox read it, so the anchor is copied in a critical section.

typedef struct {
  uint16_t year;
  uint8_t month;  // 1..12
  uint8_t day;    // 1..31
  uint8_t hour;   // 0..23
  uint8_t minute; // 0..59
  uint8_t second; // 0..59
  uint16_t millisecond;
} datetime_t;

int64_t datetime_to_epoch_ms(const datetime_t *dt);
void datetime_from_epoch_ms(int64_t epoch_ms, datetime_t *dt);

// Anchor the clock to a UTC timestamp valid at the current time_millis().
void datetime_set_epoch_ms(int64_t epoch_ms);
bool datetime_valid();
// Both return false while no source has set the clock yet.
bool datetime_now_epoch_ms(int64_t *epoch_ms);
bool datetime_now(datetime_t *dt);

#ifdef PIO_UNIT_TESTING
void datetime_test_reset();
#endif
