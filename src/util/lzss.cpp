#include "util/lzss.h"

#include "util/util.h"

static constexpr uint16_t LZSS_EMPTY = UINT16_MAX;

static uint32_t lzss_hash(const uint8_t *p) {
  const uint32_t prefix = (uint32_t)p[0] << 16 | (uint32_t)p[1] << 8 | p[2];
  return (prefix * 2654435761u) >> (32 - LZSS_HASH_BITS);
}

uint32_t lzss_encode(const uint8_t *in, uint32_t in_size, uint16_t table[LZSS_HASH_SIZE], lzss_emit_fn_t emit, void *priv) {
  for (uint32_t i = 0; i < LZSS_HASH_SIZE; i++)
    table[i] = LZSS_EMPTY;

  // Positions with a full prefix ahead of them; later ones stay literals.
  const uint32_t hashed = in_size >= LZSS_MIN_MATCH ? in_size - LZSS_MIN_MATCH + 1 : 0;

  uint32_t size = 0;
  uint32_t i = 0;
  while (i < in_size) {
    uint8_t group[1 + 8 * 2];
    uint32_t group_size = 1;
    group[0] = 0;

    for (uint32_t k = 0; k < 8 && i < in_size; k++) {
      uint32_t len = 0;
      uint32_t dist = 0;
      if (i < hashed) {
        const uint32_t hash = lzss_hash(in + i);
        const uint32_t candidate = table[hash];
        if (candidate != LZSS_EMPTY && i - candidate <= LZSS_WINDOW) {
          const uint32_t max_len = MIN(LZSS_MAX_MATCH, in_size - i);
          while (len < max_len && in[candidate + len] == in[i + len])
            len++;
          dist = i - candidate;
        }
        table[hash] = i;
      }

      if (len >= LZSS_MIN_MATCH) {
        const uint16_t token = (dist - 1) << LZSS_LENGTH_BITS | (len - LZSS_MIN_MATCH);
        group[group_size++] = token >> 8;
        group[group_size++] = token & 0xff;
        group[0] |= 1 << k;
        for (uint32_t p = i + 1; p < i + len && p < hashed; p++)
          table[lzss_hash(in + p)] = p;
        i += len;
      } else {
        group[group_size++] = in[i++];
      }
    }

    if (emit)
      emit(group, group_size, priv);
    size += group_size;
  }
  return size;
}
