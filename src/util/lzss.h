#pragma once

#include <stdint.h>

// Byte-aligned LZSS. A control byte precedes each group of up to eight
// tokens, bit k set when token k is a match. Literals are one byte; matches
// are two bytes, big-endian, holding distance - 1 in the upper
// LZSS_OFFSET_BITS and length - LZSS_MIN_MATCH in the rest. The stream ends
// with its input, so the container carries the compressed length.
static constexpr uint32_t LZSS_OFFSET_BITS = 11;
static constexpr uint32_t LZSS_LENGTH_BITS = 16 - LZSS_OFFSET_BITS;
static constexpr uint32_t LZSS_WINDOW = 1 << LZSS_OFFSET_BITS;
static constexpr uint32_t LZSS_MIN_MATCH = 3;
static constexpr uint32_t LZSS_MAX_MATCH = LZSS_MIN_MATCH + (1 << LZSS_LENGTH_BITS) - 1;

// Matches are found through a table holding the latest position of each
// hashed three-byte prefix; the caller provides its storage. Positions are
// 16 bits, so inputs stay below UINT16_MAX bytes.
// TODO: once the heap has room, chain earlier positions through a
// LZSS_WINDOW entry prev table (4 KiB more) and probe about four candidates;
// that brings the profile within 1% of an exhaustive search instead of 7%.
static constexpr uint32_t LZSS_HASH_BITS = 10;
static constexpr uint32_t LZSS_HASH_SIZE = 1 << LZSS_HASH_BITS;

// Receives the output a control byte group at a time.
typedef void (*lzss_emit_fn_t)(const uint8_t *data, uint32_t len, void *priv);

// Returns the compressed size. Output goes to emit unless it is NULL, so a
// first pass can size the stream; the encoding is deterministic.
uint32_t lzss_encode(const uint8_t *in, uint32_t in_size, uint16_t table[LZSS_HASH_SIZE], lzss_emit_fn_t emit, void *priv);
