/*
 * Copyright (C) 2019 Koen Zandberg
 *
 * This file is subject to the terms and conditions of the GNU Lesser
 * General Public License v2.1. See the file LICENSE in the top level
 * directory for more details.
 */

/**
 * @ingroup nanocbor
 * @{
 * @file
 * @brief   Minimalistic CBOR encoder implementation
 *
 * @author  Koen Zandberg <koen@bergzand.net>
 * @}
 */

#include <stddef.h>
#include <stdint.h>
#include <stdlib.h>
#include <string.h>

#include "nanocbor/config.h"
#include "nanocbor/nanocbor.h"

#include NANOCBOR_BYTEORDER_HEADER

void nanocbor_encoder_init(nanocbor_encoder_t *enc, uint8_t *buf, size_t len)
{
    enc->len = 0;
    enc->cur = buf;
    enc->end = buf + len;
}

size_t nanocbor_encoded_len(nanocbor_encoder_t *enc)
{
    return enc->len;
}

static int _fits(nanocbor_encoder_t *enc, size_t len)
{
    enc->len += len;
    return ((size_t)(enc->end - enc->cur) >= len) ? 0 : -1;
}

static int _fmt_single(nanocbor_encoder_t *enc, uint8_t single)
{
    int res = _fits(enc, 1);

    if (res == 0) {
        *enc->cur++ = single;
    }
    return res;
}

int nanocbor_fmt_bool(nanocbor_encoder_t *enc, bool content)
{
    uint8_t single = NANOCBOR_MASK_FLOAT | (content ? NANOCBOR_SIMPLE_TRUE
                                            : NANOCBOR_SIMPLE_FALSE);

    return _fmt_single(enc, single);
}

static int _fmt_uint64(nanocbor_encoder_t *enc, uint64_t num, uint8_t type)
{
    unsigned extrabytes = 0;

    if (num < NANOCBOR_SIZE_BYTE) {
        type |= num;
    }
    else {
        if (num > UINT32_MAX) {
            /* Requires long size */
            type |= NANOCBOR_SIZE_LONG;
            extrabytes = sizeof(uint64_t);
        }
        else if (num > UINT16_MAX) {
            /* At least word size */
            type |= NANOCBOR_SIZE_WORD;
            extrabytes = sizeof(uint32_t);
        }
        else if (num > UINT8_MAX) {
            type |= NANOCBOR_SIZE_SHORT;
            extrabytes = sizeof(uint16_t);
        }
        else {
            type |= NANOCBOR_SIZE_BYTE;
            extrabytes = sizeof(uint8_t);
        }
    }
    int res = _fits(enc, extrabytes + 1);
    if (res == 0) {
        *enc->cur++ = type;

        /* NOLINTNEXTLINE: user supplied function */
        uint64_t benum = NANOCBOR_HTOBE64_FUNC(num);

        memcpy(enc->cur, (uint8_t *)&benum + sizeof(benum) - extrabytes,
               extrabytes);
        enc->cur += extrabytes;
    }
    return res;
}

int nanocbor_fmt_uint(nanocbor_encoder_t *enc, uint64_t num)
{
    return _fmt_uint64(enc, num, NANOCBOR_MASK_UINT);
}

int nanocbor_fmt_tag(nanocbor_encoder_t *enc, uint64_t num)
{
    return _fmt_uint64(enc, num, NANOCBOR_MASK_TAG);
}

int nanocbor_fmt_int(nanocbor_encoder_t *enc, int64_t num)
{
    if (num < 0) {
        /* Always negative at this point */
        num = -1 * (num + 1);
        return _fmt_uint64(enc, num, NANOCBOR_MASK_NINT);
    }
    return nanocbor_fmt_uint(enc, (uint64_t)num);
}

int nanocbor_fmt_bstr(nanocbor_encoder_t *enc, size_t len)
{
    return _fmt_uint64(enc, (uint64_t)len, NANOCBOR_MASK_BSTR);
}

int nanocbor_fmt_tstr(nanocbor_encoder_t *enc, size_t len)
{
    return _fmt_uint64(enc, (uint64_t)len, NANOCBOR_MASK_TSTR);
}

static int _put_bytes(nanocbor_encoder_t *enc, const uint8_t *str, size_t len)
{
    int res = _fits(enc, len);

    if (res == 0) {
        memcpy(enc->cur, str, len);
        enc->cur += len;
    }
    return res;
}

int nanocbor_put_tstr(nanocbor_encoder_t *enc, const char *str)
{
    size_t len = strlen(str);

    nanocbor_fmt_tstr(enc, len);
    return _put_bytes(enc, (const uint8_t *)str, len);
}

int nanocbor_put_bstr(nanocbor_encoder_t *enc, const uint8_t *str, size_t len)
{
    nanocbor_fmt_bstr(enc, len);
    return _put_bytes(enc, str, len);
}

int nanocbor_fmt_array(nanocbor_encoder_t *enc, size_t len)
{
    return _fmt_uint64(enc, (uint64_t)len, NANOCBOR_MASK_ARR);
}

int nanocbor_fmt_map(nanocbor_encoder_t *enc, size_t len)
{
    return _fmt_uint64(enc, (uint64_t)len, NANOCBOR_MASK_MAP);
}

int nanocbor_fmt_array_indefinite(nanocbor_encoder_t *enc)
{
    return _fmt_single(enc, NANOCBOR_MASK_ARR | NANOCBOR_SIZE_INDEFINITE);
}

int nanocbor_fmt_map_indefinite(nanocbor_encoder_t *enc)
{
    return _fmt_single(enc, NANOCBOR_MASK_MAP | NANOCBOR_SIZE_INDEFINITE);
}

int nanocbor_fmt_end_indefinite(nanocbor_encoder_t *enc)
{
    /* End is marked with float major and indefinite minor number */
    return _fmt_single(enc, NANOCBOR_MASK_FLOAT | NANOCBOR_SIZE_INDEFINITE);
}

int nanocbor_fmt_null(nanocbor_encoder_t *enc)
{
    return _fmt_single(enc, NANOCBOR_MASK_FLOAT | NANOCBOR_SIMPLE_NULL);
}

/* Double bit mask related defines */
#define DOUBLE_EXP_OFFSET                                 (1023U)
#define DOUBLE_SIZE                                         (64U)
#define DOUBLE_EXP_POS                                      (52U)
#define DOUBLE_SIGN_POS                                     (63U)
#define DOUBLE_EXP_MASK                        ((uint64_t)0x7FFU)
#define DOUBLE_SIGN_MASK        ((uint64_t)1U << DOUBLE_SIGN_POS)
#define DOUBLE_EXP_IS_NAN                                (0x7FFU)
#define DOUBLE_IS_ZERO                      (~(DOUBLE_SIGN_MASK))
#define DOUBLE_FLOAT_LOSS                           (0x1FFFFFFFU)

/* float bit mask related defines */
#define FLOAT_EXP_OFFSET                                   (127U)
#define FLOAT_SIZE                                          (32U)
#define FLOAT_EXP_POS                                       (23U)
#define FLOAT_EXP_MASK                          ((uint32_t)0xFFU)
#define FLOAT_SIGN_POS                                      (31U)
#define FLOAT_FRAC_MASK                               (0x7FFFFFU)
#define FLOAT_SIGN_MASK          ((uint32_t)1U << FLOAT_SIGN_POS)
#define FLOAT_EXP_IS_NAN                                  (0xFFU)
#define FLOAT_IS_ZERO                        (~(FLOAT_SIGN_MASK))
/* Part where a float to halffloat leads to precision loss */
#define FLOAT_HALF_LOSS                                 (0x1FFFU)

/* halffloat bit mask related defines */
#define HALF_EXP_OFFSET                                     (15U)
#define HALF_SIZE                                           (16U)
#define HALF_EXP_POS                                        (10U)
#define HALF_EXP_MASK                                     (0x1FU)
#define HALF_SIGN_POS                                       (15U)
#define HALF_FRAC_MASK                                   (0x3FFU)
#define HALF_SIGN_MASK          ((uint16_t)(1U << HALF_SIGN_POS))
#define HALF_MASK_HALF                                    (0xFFU)

/* Check special cases for single precision floats */
static bool _single_is_inf_nan(uint8_t exp)
{
    return exp == FLOAT_EXP_IS_NAN;
}

static bool _single_is_zero(uint32_t num)
{
    return (num & FLOAT_IS_ZERO) == 0;
}

static bool _single_in_range(uint8_t exp, uint32_t num)
{
    /* Check if lower 13 bits of fraction are zero, if so we might be able to
     * convert without precision loss */
    if (exp <= (HALF_EXP_OFFSET + FLOAT_EXP_OFFSET) &&
        exp >= ((-HALF_EXP_OFFSET + 1) + FLOAT_EXP_OFFSET) &&
        ((num & FLOAT_HALF_LOSS) == 0)) {
        return true;
    }
    return false;
}

static int _fmt_halffloat(nanocbor_encoder_t *enc, uint16_t half)
{
    int res = _fits(enc, sizeof(uint16_t) + 1);
    if (res == 0) {
        *enc->cur++ = NANOCBOR_MASK_FLOAT | NANOCBOR_SIZE_SHORT;
        *enc->cur++ = (half >> HALF_SIZE/2);
        *enc->cur++ =  half & HALF_MASK_HALF;
        res = sizeof(uint16_t) + 1;
    }
    return res;
}

/* Check special cases for single precision floats */
static bool _double_is_inf_nan(uint16_t exp)
{
    return (exp == DOUBLE_EXP_IS_NAN);
}

static bool _double_is_zero(uint64_t num)
{
    return (num & DOUBLE_IS_ZERO) == 0;
}

static bool _double_in_range(uint16_t exp, uint64_t num)
{
    /* Check if lower 13 bits of fraction are zero, if so we might be able to
     * convert without precision loss */
    if (exp <= (DOUBLE_EXP_OFFSET + FLOAT_EXP_OFFSET) &&
        exp >= (DOUBLE_EXP_OFFSET - FLOAT_EXP_OFFSET + 1) &&
        ((num & DOUBLE_FLOAT_LOSS) == 0)) { /* First 29 bits must be zero */
        return true;
    }
    return false;
}

int nanocbor_fmt_float(nanocbor_encoder_t *enc, float num)
{
    /* Allow bitwise access to float */
    uint32_t *unum = (uint32_t *)&num;

    /* Retrieve exponent */
    uint8_t exp = (*unum >> FLOAT_EXP_POS) & FLOAT_EXP_MASK;
    if (_single_is_inf_nan(exp) ||
            _single_is_zero(*unum) ||
            _single_in_range(exp, *unum)) {
        /* Copy sign bit */
        uint16_t half = ((*unum >> (FLOAT_SIZE - HALF_SIZE)) & HALF_SIGN_MASK);
        /* Shift exponent */
        if (exp != FLOAT_EXP_IS_NAN && exp != 0) {
            exp = exp + (HALF_EXP_OFFSET - FLOAT_EXP_OFFSET);
        }
        /* Add exponent */
        half |= ((exp & HALF_EXP_MASK) << HALF_EXP_POS) |
                ((*unum >> (FLOAT_EXP_POS - HALF_EXP_POS)) & HALF_FRAC_MASK);
        return _fmt_halffloat(enc, half);
    }
    /* normal float */
    int res = _fits(enc, 1 + sizeof(float));
    if (res == 0) {
        *enc->cur++ = NANOCBOR_MASK_FLOAT | NANOCBOR_SIZE_WORD;
        /* NOLINTNEXTLINE: user supplied function */
        uint32_t bnum = NANOCBOR_HTOBE32_FUNC(*unum);
        memcpy(enc->cur, &bnum, sizeof(bnum));
        enc->cur += sizeof(float);
        res = sizeof(float) + 1;
    }
    return res;
}

int nanocbor_fmt_double(nanocbor_encoder_t *enc, double num)
{
    uint64_t *unum = (uint64_t *)&num;
    uint16_t exp = (*unum >> DOUBLE_EXP_POS) & DOUBLE_EXP_MASK;
    if (_double_is_inf_nan(exp) ||
            _double_is_zero(*unum) ||
            _double_in_range(exp, *unum)) {
        /* copy sign bit over */
        uint32_t single = (*unum >> (DOUBLE_SIZE - FLOAT_SIZE)) & (FLOAT_SIGN_MASK);
        /* Shift exponent */
        if (exp != DOUBLE_EXP_IS_NAN && exp != 0) {
            exp = exp + FLOAT_EXP_OFFSET - DOUBLE_EXP_OFFSET;
        }
        single |= ((exp & FLOAT_EXP_MASK) << FLOAT_EXP_POS) |
                  ((*unum >> (DOUBLE_EXP_POS - FLOAT_EXP_POS)) & FLOAT_FRAC_MASK);
        float *fsingle = (float*)&single;
        return nanocbor_fmt_float(enc, *fsingle);
    }
    int res = _fits(enc, 1 + sizeof(double));
    if (res == 0) {
        *enc->cur++ = NANOCBOR_MASK_FLOAT | NANOCBOR_SIZE_LONG;
        /* NOLINTNEXTLINE: user supplied function */
        uint64_t bnum = NANOCBOR_HTOBE64_FUNC(*unum);
        memcpy(enc->cur, &bnum, sizeof(bnum));
        enc->cur += sizeof(double);
        res = sizeof(double) + 1;
    }
    return res;
}
