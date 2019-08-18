/*
 * Copyright (C) 2019 Koen Zandberg
 *
 * This file is subject to the terms and conditions of the GNU Lesser
 * General Public License v2.1. See the file LICENSE in the top level
 * directory for more details.
 */

/**
 * @defgroup    NanoCBOR minimalistic CBOR library
 * @brief       Provides a minimal CBOR library
 *
 * NanoCBOR is a minimal CBOR encoder. For protocols such as CoAP, OSCORE,
 * SenML and CORECONF a well defined and thus predictable CBOR structure is
 * required. NanoCBOR tries to fill this requirement by providing a very
 * minimal CBOR encoder. Supported is:
 *  - All major types
 *  - Arrays including indefinite length arrays
 *  - Maps including indefinite length maps
 *  - Safe for decoding untrusted input
 *
 * Not included:
 *  - Date and time
 *  - Big numbers (numbers encoded as byte strings)
 *
 * @{
 *
 * @file
 * @see         [rfc 7049](https://tools.ietf.org/html/rfc7049)
 *
 * @author      Koen Zandberg <koen@bergzand.net>
 */

#ifndef NANOCBOR_H
#define NANOCBOR_H

#include <stdbool.h>
#include <stdint.h>
#include <stdlib.h>

#ifdef __cplusplus
extern "C" {
#endif

#define NANOCBOR_TYPE_OFFSET    (5U)   /**< Bit shift for CBOR major types */
#define NANOCBOR_TYPE_MASK      0xE0U  /**< Mask for CBOR major types */
#define NANOCBOR_VALUE_MASK     0x1FU  /**< Mask for CBOR values */

/**
 * @name CBOR type numbers
 * @{
 */
#define NANOCBOR_TYPE_UINT      (0x00U) /**< positive integer type */
#define NANOCBOR_TYPE_NINT      (0x01U) /**< negative integer type */
#define NANOCBOR_TYPE_BSTR      (0x02U) /**< byte string type */
#define NANOCBOR_TYPE_TSTR      (0x03U) /**< text string type */
#define NANOCBOR_TYPE_ARR       (0x04U) /**< array type */
#define NANOCBOR_TYPE_MAP       (0x05U) /**< map type */
#define NANOCBOR_TYPE_TAG       (0x06U) /**< tag type */
#define NANOCBOR_TYPE_FLOAT     (0x07U) /**< float type */
/** @} */

/**
 * @name CBOR major types including the bit shift
 * @{
 */
#define NANOCBOR_MASK_UINT      (NANOCBOR_TYPE_UINT  << NANOCBOR_TYPE_OFFSET)
#define NANOCBOR_MASK_NINT      (NANOCBOR_TYPE_NINT  << NANOCBOR_TYPE_OFFSET)
#define NANOCBOR_MASK_BSTR      (NANOCBOR_TYPE_BSTR  << NANOCBOR_TYPE_OFFSET)
#define NANOCBOR_MASK_TSTR      (NANOCBOR_TYPE_TSTR  << NANOCBOR_TYPE_OFFSET)
#define NANOCBOR_MASK_ARR       (NANOCBOR_TYPE_ARR   << NANOCBOR_TYPE_OFFSET)
#define NANOCBOR_MASK_MAP       (NANOCBOR_TYPE_MAP   << NANOCBOR_TYPE_OFFSET)
#define NANOCBOR_MASK_TAG       (NANOCBOR_TYPE_TAG   << NANOCBOR_TYPE_OFFSET)
#define NANOCBOR_MASK_FLOAT     (NANOCBOR_TYPE_FLOAT << NANOCBOR_TYPE_OFFSET)
/** @} */

/**
 * @name CBOR simple data types
 * @{
 */
#define NANOCBOR_SIMPLE_FALSE       20U /**< False     */
#define NANOCBOR_SIMPLE_TRUE        21U /**< True      */
#define NANOCBOR_SIMPLE_NULL        22U /**< NULL      */
#define NANOCBOR_SIMPLE_UNDEF       23U /**< Undefined */
/** @} */

/**
 * @name CBOR data sizes
 * @{
 */
#define NANOCBOR_SIZE_BYTE          24U /**< Value contained in a byte */
#define NANOCBOR_SIZE_SHORT         25U /**< Value contained in a short */
#define NANOCBOR_SIZE_WORD          26U /**< Value contained in a word */
#define NANOCBOR_SIZE_LONG          27U /**< Value contained in a long */
#define NANOCBOR_SIZE_INDEFINITE    31U /**< Indefinite sized container */
/** @} */

/**
 * @brief NanoCBOR decoder errors
 */
typedef enum {
    /**
     * @brief No error
     */
    NANOCBOR_OK = 0,

    /**
     * @brief Overflow in the getter. This can happen due to retrieving a
     *        number size larger than the function provides
     */
    NANOCBOR_ERR_OVERFLOW = -1,

    /**
     * Decoder get function attempts to retrieve the wrong type
     */
    NANOCBOR_ERR_INVALID_TYPE = -2,

    /**
     * @brief decoder is beyond the end of the buffer
     */
    NANOCBOR_ERR_END = -3,

    /**
     * @brief Decoder hits the recursion limit
     */
    NANOCBOR_ERR_RECURSION = -4,
} nanocbor_error_t;


/**
 * @brief decoder context
 */
typedef struct nanocbor_value {
    const uint8_t *cur;   /**< Current position in the buffer             */
    const uint8_t *end;   /**< End of the buffer                          */
    uint32_t remaining;   /**< Number of items remaining in the container */
    uint8_t flags;        /**< Flags for decoding hints                   */
} nanocbor_value_t;

/**
 * @brief encoder context
 */
typedef struct nanocbor_encoder {
    uint8_t *cur;   /**< Current position in the buffer */
    uint8_t *end;   /**< end of the buffer                      */
    size_t len;     /**< Length in bytes of supplied cbor data. Incremented
                      *  separate from the buffer check  */
} nanocbor_encoder_t;

/**
 * @name decoder flags
 * @{
 */

/**
 * @brief decoder value is inside a container
 */
#define NANOCBOR_DECODER_FLAG_CONTAINER  (0x01U)

/**
 * @brief decoder value is inside an indefinite length container
 */
#define NANOCBOR_DECODER_FLAG_INDEFINITE (0x02U)
/** @} */

/**
 * @name NanoCBOR parser functions
 * @{
 */

/**
 * @brief Initialize a decoder context decoding the CBOR structure from @p buf
 *        with @p len bytes
 *
 * The decoder will attempt to decode CBOR types until the buffer is exhausted
 *
 * @param[in]   value   decoder value context
 * @param[in]   buf     Buffer to decode from
 * @param[in]   len     Length in bytes of the buffer
 */
void nanocbor_decoder_init(nanocbor_value_t *value,
                           const uint8_t *buf, size_t len);

/**
 * @brief Retrieve the type of the CBOR value at the current position
 *
 * @param[in]   value   decoder value context
 *
 * @return              major type
 * @return              NANOCBOR_ERR_OVERFLOW if the buffer is exhausted
 */
int nanocbor_get_type(const nanocbor_value_t *value);

/**
 * @brief Check if the current buffer or container is exhausted
 *
 * @param[in]   it      decoder value context
 *
 * @return              true if it is exhausted
 * @return              false if there are more items
 */
bool nanocbor_at_end(const nanocbor_value_t *it);

/**
 * @brief Retrieve a positive integer as uint32_t from the stream
 *
 * The resulting @p value is undefined if the result is an error condition
 *
 * @param[in]   cvalue  CBOR value to decode from
 * @param[out]  value   returned positive integer
 *
 * @return              number of bytes read
 * @return              negative on error
 */
int nanocbor_get_uint32(nanocbor_value_t *cvalue, uint32_t *value);

/**
 * @brief Retrieve a signed integer as int32_t from the stream
 *
 * The resulting @p value is undefined if the result is an error condition
 *
 * @param[in]   cvalue  CBOR value to decode from
 * @param[out]  value   returned signed integer
 *
 * @return              number of bytes read
 * @return              negative on error
 */
int nanocbor_get_int32(nanocbor_value_t *cvalue, int32_t *value);

/**
 * @brief Retrieve a byte string from the stream
 *
 * The resulting @p buf and @p len are undefined if the result is an error
 * condition
 *
 * @param[in]   cvalue  CBOR value to decode from
 * @param[out]  buf     pointer to the byte string
 * @param[out]  len     length of the byte string
 *
 * @return              number of bytes read
 * @return              negative on error
 */
int nanocbor_get_bstr(nanocbor_value_t *cvalue, const uint8_t **buf, size_t *len);

/**
 * @brief Retrieve a text string from the stream
 *
 * The resulting @p buf and @p len are undefined if the result is an error
 * condition
 *
 * @param[in]   cvalue  CBOR value to decode from
 * @param[out]  buf     pointer to the text string
 * @param[out]  len     length of the text string
 *
 * @return              number of bytes read
 * @return              negative on error
 */
int nanocbor_get_tstr(nanocbor_value_t *cvalue, const uint8_t **buf, size_t *len);

/**
 * @brief Enter a array type
 *
 * @param[in]   it      CBOR value to decode from
 * @param[out]  array   CBOR value to decode the array members with
 *
 * @return              NANOCBOR_OK on success
 * @return              negative on error
 */
int nanocbor_enter_array(nanocbor_value_t *it, nanocbor_value_t *array);

/**
 * @brief Enter a map type
 *
 * @param[in]   it      CBOR value to decode from
 * @param[out]  map     CBOR value to decode the map members with
 *
 * @return              NANOCBOR_OK on success
 * @return              negative on error
 */
int nanocbor_enter_map(nanocbor_value_t *it, nanocbor_value_t *map);

/**
 * @brief leave the container
 *
 * This must be called with the same @ref nanocbor_value_t struct that was used
 * to enter the container. Furthermore, the @p container must be at the end of
 * the container.
 *
 * @param[in]   it          parent CBOR structure
 * @param[in]   container   exhausted CBOR container
 */
void nanocbor_leave_container(nanocbor_value_t *it, nanocbor_value_t *container);

/**
 * @brief Retrieve a tag as positive uint32_t from the stream
 *
 * The resulting @p value is undefined if the result is an error condition
 *
 * @param[in]   cvalue  CBOR value to decode from
 * @param[out]  value   returned tag as positive integer
 *
 * @return              number of bytes read
 * @return              negative on error
 */
int nanocbor_get_tag(nanocbor_value_t *cvalue, uint32_t *tag);

/**
 * @brief Retrieve a null value from the stream
 *
 * This function checks if the next CBOR value is a NULL value and advances to
 * the next value if no error is detected
 *
 * @param[in]   cvalue  CBOR value to decode from
 *
 * @return              NANOCBOR_OK on success
 */
int nanocbor_get_null(nanocbor_value_t *cvalue);

/**
 * @brief Retrieve a boolean value from the stream
 *
 * @param[in]   cvalue  CBOR value to decode from
 * @param[out]  value   Boolean value retrieved from the stream
 *
 * @return              NANOCBOR_OK on success
 * @return              negative on error
 */
int nanocbor_get_bool(nanocbor_value_t *cvalue, bool *value);

/**
 * @brief Skip to the next value in the CBOR stream
 *
 * This function is able to skip over nested structures in the CBOR stream
 * such as (nested) arrays and maps. It uses limited recursion to do so.
 *
 * Recursion is limited with @ref NANOCBOR_RECURSION_MAX
 *
 * @param[in]   it  CBOR stream to skip a value from
 *
 * @return              NANOCBOR_OK on success
 * @return              negative on error
 */
int nanocbor_skip(nanocbor_value_t *it);

/**
 * @brief Skip a single simple value in the CBOR stream
 *
 * This is a cheaper version of @ref nanocbor_skip, the downside is that this
 * function is unable to skip nested structures.
 *
 * @param[in]   it  CBOR value to skip
 *
 * @return              NANOCBOR_OK on success
 * @return              negative on error
 */
int nanocbor_skip_simple(nanocbor_value_t *it);

/**
 * @brief Retrieve part of the CBOR stream for separate parsing
 *
 * This function retrieves the pointer and length of a single CBOR item. This
 * item can be stored for later processing.
 *
 * @param[in]   it      CBOR value to retrieve
 * @param[out]  start   start of the CBOR item
 * @param[out]  len     length of the CBOR item
 *
 * @return              NANOCBOR_OK on success
 * @return              negative on error
 */
int nanocbor_get_subcbor(nanocbor_value_t *it, const uint8_t **start,
                         size_t *len);

/**
 * @brief Retrieve the number of remaining values is a CBOR container
 *
 * The returned value is undefined when not inside a container or when the
 * container is of indefinite length. For a map, the number is the full number
 * of CBOR items remaining (twice the number of key/value pairs).
 *
 * @param[in]   value   value inside a CBOR container
 *
 * @return              number of items remaining
 */
static inline uint32_t nanocbor_container_remaining(const nanocbor_value_t *value)
{
    return value->remaining;
}

/**
 * @brief Check whether a container is an indefinite-length container
 *
 * @param[in]   container   value inside a CBOR container
 *
 * @return                  True when the container is indefinite in length
 * @return                  False when not indefinite-length or not in a
 *                          container
 */
static inline bool nanocbor_container_indefinite(const nanocbor_value_t *container)
{
    return (container->flags ==
        (NANOCBOR_DECODER_FLAG_INDEFINITE | NANOCBOR_DECODER_FLAG_CONTAINER));
}

static inline bool nanocbor_in_container(const nanocbor_value_t *container)
{
    return container->flags & (NANOCBOR_DECODER_FLAG_CONTAINER);
}

/** @} */

/**
 * @name NanoCBOR encoder functions
 * @{
 */

/**
 * @brief Initializes an encoder context with a buffer.
 *
 * It is safe to pass `NULL` to @p buf with @p len is `0` to determine the size
 * of a CBOR structure.
 *
 * @param[in]   enc     Encoder context
 * @param[in]   buf     Buffer to write into
 * @param[in]   len     length of the buffer
 */
void nanocbor_encoder_init(nanocbor_encoder_t *enc,
                           uint8_t *buf, size_t len);

/**
 * @brief Retrieve the encoded length of the CBOR structure
 *
 * This function doesn't take the length of the buffer supplied to
 * @ref nanocbor_encoder_init into account, it only returns the number of bytes
 * the current CBOR structure would take up.
 *
 * @param[in]   enc Encoder context
 *
 * @return          Length of the encoded structure
 */
size_t nanocbor_encoded_len(nanocbor_encoder_t *enc);



/**
 * @brief Write a CBOR boolean value into a buffer
 *
 * @param[in]   enc     Encoder context
 * @param[in]   content Boolean value to write
 *
 * @return              Number of bytes written
 */
int nanocbor_fmt_bool(nanocbor_encoder_t *enc, bool content);

/**
 * @brief Write an unsigned integer of at most sizeof uint64_t into the buffer
 *
 * @param[in]   enc     Encoder context
 * @param[in]   num     unsigned integer to write
 *
 * @return  number of bytes written
 */
int nanocbor_fmt_uint(nanocbor_encoder_t *enc, uint64_t num);

/**
 * @brief Write a CBOR tag of at most sizeof uint64_t into the buffer
 *
 * @param[in]   enc     Encoder context
 * @param[in]   num     tag value to write into the buffer
 *
 * @return  number of bytes written
 */
int nanocbor_fmt_tag(nanocbor_encoder_t *enc, uint64_t num);

/**
 * @brief Write a signed integer of at most sizeof int32_t into the buffer
 *
 * If it is not certain if the data is signed, use this function.
 *
 * @param[in]   enc     Encoder context
 * @param[in]   num     unsigned integer to write
 *
 * @return              number of bytes written
 */
int nanocbor_fmt_int(nanocbor_encoder_t *enc, int64_t num);

/**
 * @brief Write a byte string indicator for a byte string with specific length
 * into the encoder buffer
 *
 * This doesn't write any byte string into the encoder buffer, only the type
 * and length indicator for the byte string
 *
 * @param[in]   enc     Encoder context
 * @param[in]   len     Length of the byte string
 *
 * @return              number of bytes written
 */
int nanocbor_fmt_bstr(nanocbor_encoder_t *enc, size_t len);

/**
 * @brief Write a text string indicator for a string with specific length
 * into the encoder buffer
 *
 * This doesn't write any text string into the encoder buffer, only the type
 * and length indicator for the text string
 *
 * @param[in]   enc     Encoder context
 * @param[in]   len     Length of the text string
 *
 * @return              number of bytes written
 */
int nanocbor_fmt_tstr(nanocbor_encoder_t *enc, size_t len);

/**
 * @brief Copy a byte string with indicator into the encoder buffer
 *
 * @param[in]   enc     Encoder context
 * @param[in]   str     byte string to encode
 * @param[in]   len     Length of the string
 *
 * @return              number of bytes written
 */
int nanocbor_put_bstr(nanocbor_encoder_t *enc, const uint8_t *str, size_t len);

/**
 * @brief Copy a text string with indicator into the encoder buffer
 *
 * @param[in]   enc     Encoder context
 * @param[in]   str     null terminated text string to encode
 *
 * @return              number of bytes written
 */
int nanocbor_put_tstr(nanocbor_encoder_t *enc, const char *str);

/**
 * @brief Write an array indicator with @p len items
 *
 * It is assumed that the calling code will encode @p len items after calling
 * this function. The array automatically terminates after @p len items are
 * added, no function to close the container is necessary.
 *
 * @param[in]   enc     Encoder context
 * @param[in]   len     Number of items in the array
 *
 * @return              Number of bytes written
 */
int nanocbor_fmt_array(nanocbor_encoder_t *enc, size_t len);

/**
 * @brief Write a map indicator with @p len pairs
 *
 * It is assumed that the calling code will encode @p len item pairs after
 * calling this function. The array automatically terminates after @p len item
 * pairs are added, no function to close the container is necessary.
 *
 * @param[in]   enc     Encoder context
 * @param[in]   len     Number of pairs in the map
 *
 * @return              Number of bytes written
 */
int nanocbor_fmt_map(nanocbor_encoder_t *enc, size_t len);

/**
 * @brief Write an indefinite-length array indicator
 *
 * @param[in]   enc     Encoder context
 *
 * @return              Number of bytes written
 */
int nanocbor_fmt_array_indefinite(nanocbor_encoder_t *enc);

/**
 * @brief Write an indefinite-length map indicator
 *
 * @param[in]   enc     Encoder context
 *
 * @return              Number of bytes written
 */
int nanocbor_fmt_map_indefinite(nanocbor_encoder_t *enc);

/**
 * @brief Write a stop code for indefinite length containers
 *
 * @param[in]   enc     Encoder context
 *
 * @return              Number of bytes written
 */
int nanocbor_fmt_end_indefinite(nanocbor_encoder_t *enc);

/**
 * @brief Write a Null value into the encoder buffer
 *
 * @param[in]   enc     Encoder context
 *
 * @return              Number of bytes written
 */
int nanocbor_fmt_null(nanocbor_encoder_t *enc);

/**
 * @brief Write a float value into the encoder buffer
 *
 * @param[in]   enc     Encoder context
 * @param[in]   num     Floating point to encode
 *
 * @return              Number of bytes written
 */
int nanocbor_fmt_float(nanocbor_encoder_t *enc, float num);

/**
 * @brief Write a double floating point value into the encoder buffer
 *
 * @param[in]   enc     Encoder context
 * @param[in]   num     Floating point to encode
 *
 * @return              Number of bytes written
 */
int nanocbor_fmt_double(nanocbor_encoder_t *enc, double num);

/** @} */

#ifdef __cplusplus
}
#endif

#endif /* NANOCBOR_H */
/** @} */
