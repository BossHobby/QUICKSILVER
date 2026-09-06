#include "util/util.h"

// Mixed bounds retain the former C macro's comparisons and result promotions.
static_assert(constrain(uint8_t{200}, 0, 300) == 200);
static_assert(constrain(uint8_t{10}, -1, 100) == 10);
static_assert(constrain(0, 0.5f, 1.5f) == 0.5f);
static_assert(constrain(2, 0.5f, 1.5f) == 1.5f);
static_assert(constrain(-2.0f, -1, 1) == -1.0f);
static_assert(constrain(0.25f, -1, 1) == 0.25f);
static_assert(constrain(2.0f, -1, 1) == 1.0f);
static_assert(std::is_same_v<decltype(constrain(uint8_t{200}, 0, 300)), int>);
static_assert(std::is_same_v<decltype(constrain(0, 0.5f, 1.5f)), float>);

// C promotes byte-sized conditional operands before subsequent arithmetic.
static_assert(MIN(uint8_t{1}, uint8_t{2}) - uint8_t{2} == -1);
static_assert(std::is_same_v<decltype(MIN(uint8_t{1}, uint8_t{2})), int>);
static_assert(std::is_same_v<decltype(MAX(uint16_t{1}, uint16_t{2})), int>);
static_assert(std::is_same_v<decltype(constrain(uint8_t{1}, uint8_t{0}, uint8_t{2})), int>);
static_assert(MIN(-1, 1U) == 1U);
static_assert(MAX(-1, 1U) == UINT32_MAX);
