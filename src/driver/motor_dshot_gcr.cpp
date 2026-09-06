#include "motor_dshot.h"

#ifdef SRAM_BB_BASE
typedef struct {
  uint32_t value;
  uint32_t _other[15];
} dshot_bitband_t;

#define DSHOT_BITBAND
#define BITBAND_PIN(VAL) (__CLZ(__RBIT(VAL)))
#define BITBAND_SRAM(a, b) ((SRAM_BB_BASE + (((a) - SRAM_BASE) << 5) + ((b) << 2)))
#define pin_type dshot_bitband_t
#define pin_value ((ptr++)->value)
#else
#define pin_type uint16_t
#define pin_value (*ptr++ & pin_mask)
#endif

#define GCR_INVALID 0xffffffff
#define GCR_MIN_VALID_COUNT ((21 - 2) * 3) // 57
#define GCR_MAX_VALID_COUNT ((21 + 2) * 3) // 69

static const uint32_t gcr_dict[32] = {
    GCR_INVALID,
    GCR_INVALID,
    GCR_INVALID,
    GCR_INVALID,
    GCR_INVALID,
    GCR_INVALID,
    GCR_INVALID,
    GCR_INVALID,
    GCR_INVALID,
    9,
    10,
    11,
    GCR_INVALID,
    13,
    14,
    15,
    GCR_INVALID,
    GCR_INVALID,
    2,
    3,
    GCR_INVALID,
    5,
    6,
    7,
    GCR_INVALID,
    0,
    8,
    1,
    GCR_INVALID,
    4,
    12,
    GCR_INVALID,
};

uint32_t dshot_decode_eRPM_telemetry_value(uint16_t value) {
  // eRPM range
  if (value == 0x0fff) {
    return 0;
  }

  // Convert value to 16 bit from the GCR telemetry format (eeem mmmm mmmm)
  value = (value & 0x01ff) << ((value & 0xfe00) >> 9);
  if (!value) {
    return 0;
  }

  // Convert period to erpm * 100
  return (1000000 * 60 / 100 + value / 2) / value;
}

uint32_t dshot_decode_gcr(uint16_t *dma_buffer, uint32_t pin_mask) {
#ifdef DSHOT_BITBAND
  const dshot_bitband_t *buf = (dshot_bitband_t *)BITBAND_SRAM((uint32_t)dma_buffer, BITBAND_PIN(pin_mask));
#else
  const uint16_t *buf = dma_buffer;
#endif

  const pin_type *ptr = buf + 10;
  const pin_type *ptr_end = buf + (GCR_DMA_BUFFER_SIZE - GCR_MIN_VALID_COUNT);

  // find falling edge
  while (ptr < ptr_end) {
    if (pin_value == 0 ||
        pin_value == 0 ||
        pin_value == 0 ||
        pin_value == 0) {
      break;
    }
  }
  if (ptr >= ptr_end) {
    return 0;
  }

  ptr_end = ptr + GCR_MAX_VALID_COUNT;

  uint32_t value = 0;
  uint32_t bit_len = 0;
  const pin_type *last_ptr = ptr;
  while (ptr < ptr_end) {
    // find rising edge
    while (ptr < ptr_end) {
      if (pin_value != 0 ||
          pin_value != 0 ||
          pin_value != 0 ||
          pin_value != 0) {
        break;
      }
    }
    if (ptr >= ptr_end) {
      break;
    }
    {
      const uint32_t len = MAX((ptr - last_ptr + 1) / 3, 1);
      value <<= len;
      last_ptr = ptr;
      bit_len += len;
    }

    // find falling edge
    while (ptr < ptr_end) {
      if (pin_value == 0 ||
          pin_value == 0 ||
          pin_value == 0 ||
          pin_value == 0) {
        break;
      }
    }
    if (ptr >= ptr_end) {
      break;
    }
    {
      const uint32_t len = MAX((ptr - last_ptr + 1) / 3, 1);
      value <<= len;
      value |= (0x1 << len) - 1;
      last_ptr = ptr;
      bit_len += len;
    }
  }

  if (bit_len < 18 || bit_len > 21) {
    return 0;
  }

  const uint32_t fill_len = (21 - bit_len);
  value <<= fill_len;
  value |= (0x1 << fill_len) - 1;
  value = (value ^ (value >> 1));

  const uint32_t decoded =
      gcr_dict[(value >> 0) & 0x1f] |
      gcr_dict[(value >> 5) & 0x1f] << 4 |
      gcr_dict[(value >> 10) & 0x1f] << 8 |
      gcr_dict[(value >> 15) & 0x1f] << 12;

  uint32_t csum = decoded;
  csum = csum ^ (csum >> 8);
  csum = csum ^ (csum >> 4);

  if ((csum & 0xf) != 0xf || decoded > 0xffff) {
    return 0;
  }

  return decoded >> 4;
}