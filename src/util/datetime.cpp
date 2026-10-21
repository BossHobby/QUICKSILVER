#include "util/datetime.h"

#include "core/project.h"
#include "driver/time.h"

#define MS_PER_DAY (24LL * 60 * 60 * 1000)

static bool clock_valid = false;
static int64_t anchor_epoch_ms = 0;
static uint32_t anchor_millis = 0;

// Civil date <-> days since 1970-01-01, Howard Hinnant's algorithm.
static int32_t days_from_civil(int32_t y, uint32_t m, uint32_t d) {
  y -= m <= 2;
  const int32_t era = (y >= 0 ? y : y - 399) / 400;
  const uint32_t yoe = (uint32_t)(y - era * 400);
  const uint32_t doy = (153 * (m > 2 ? m - 3 : m + 9) + 2) / 5 + d - 1;
  const uint32_t doe = yoe * 365 + yoe / 4 - yoe / 100 + doy;
  return era * 146097 + (int32_t)doe - 719468;
}

static void civil_from_days(int32_t z, datetime_t *dt) {
  z += 719468;
  const int32_t era = (z >= 0 ? z : z - 146096) / 146097;
  const uint32_t doe = (uint32_t)(z - era * 146097);
  const uint32_t yoe = (doe - doe / 1460 + doe / 36524 - doe / 146096) / 365;
  const uint32_t doy = doe - (365 * yoe + yoe / 4 - yoe / 100);
  const uint32_t mp = (5 * doy + 2) / 153;
  const uint32_t m = mp < 10 ? mp + 3 : mp - 9;
  dt->year = (uint16_t)((int32_t)yoe + era * 400 + (m <= 2));
  dt->month = (uint8_t)m;
  dt->day = (uint8_t)(doy - (153 * mp + 2) / 5 + 1);
}

int64_t datetime_to_epoch_ms(const datetime_t *dt) {
  const int64_t days = days_from_civil(dt->year, dt->month, dt->day);
  const int64_t seconds = (int64_t)dt->hour * 3600 + dt->minute * 60 + dt->second;
  return days * MS_PER_DAY + seconds * 1000 + dt->millisecond;
}

void datetime_from_epoch_ms(int64_t epoch_ms, datetime_t *dt) {
  int64_t days = epoch_ms / MS_PER_DAY;
  int64_t ms_of_day = epoch_ms % MS_PER_DAY;
  if (ms_of_day < 0) {
    days--;
    ms_of_day += MS_PER_DAY;
  }

  civil_from_days((int32_t)days, dt);
  const uint32_t ms = (uint32_t)ms_of_day;
  dt->hour = ms / 3600000;
  dt->minute = (ms / 60000) % 60;
  dt->second = (ms / 1000) % 60;
  dt->millisecond = ms % 1000;
}

void datetime_set_epoch_ms(int64_t epoch_ms) {
  anchor_millis = time_millis();
  anchor_epoch_ms = epoch_ms;
  clock_valid = true;
}

bool datetime_valid() {
  return clock_valid;
}

bool datetime_now(datetime_t *dt) {
  if (!clock_valid)
    return false;

  // Unsigned elapsed time stays exact for 49 days without a new anchor.
  const uint32_t elapsed = time_millis() - anchor_millis;
  datetime_from_epoch_ms(anchor_epoch_ms + elapsed, dt);
  return true;
}

#ifdef PIO_UNIT_TESTING
void datetime_test_reset() {
  clock_valid = false;
  anchor_epoch_ms = 0;
  anchor_millis = 0;
}
#endif
