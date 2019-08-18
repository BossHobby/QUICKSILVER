/*
 * Copyright (C) 2019 Koen Zandberg
 *
 * This file is subject to the terms and conditions of the GNU Lesser
 * General Public License v2.1. See the file LICENSE in the top level
 * directory for more details.
 */

#include "test.h"
#include "nanocbor/nanocbor.h"
#include <math.h>
#include <float.h>
#include <CUnit/CUnit.h>

static void print_bytestr(const uint8_t *bytes, size_t len)
{
    printf("\n");
    for(unsigned int idx=0; idx < len; idx++)
    {
        printf("%02X", bytes[idx]);
    }
    printf("\n");
}

static void test_encode_float_specials(void)
{
    uint8_t buf[64];
    nanocbor_encoder_t enc;
    nanocbor_encoder_init(&enc, buf, sizeof(buf));

    nanocbor_fmt_array_indefinite(&enc);
    CU_ASSERT_EQUAL(nanocbor_fmt_float(&enc, NAN), 3);
    CU_ASSERT_EQUAL(nanocbor_fmt_float(&enc, -NAN), 3);
    CU_ASSERT_EQUAL(nanocbor_fmt_float(&enc, INFINITY), 3);
    CU_ASSERT_EQUAL(nanocbor_fmt_float(&enc, -INFINITY), 3);

    CU_ASSERT_EQUAL(nanocbor_fmt_double(&enc, NAN), 3);
    CU_ASSERT_EQUAL(nanocbor_fmt_double(&enc, -NAN), 3);
    CU_ASSERT_EQUAL(nanocbor_fmt_double(&enc, INFINITY), 3);
    CU_ASSERT_EQUAL(nanocbor_fmt_double(&enc, -INFINITY), 3);
    nanocbor_fmt_end_indefinite(&enc);
    print_bytestr(buf, nanocbor_encoded_len(&enc));
}

static void test_encode_float_to_half(void)
{
    uint8_t buf[64];
    nanocbor_encoder_t enc;
    nanocbor_encoder_init(&enc, buf, sizeof(buf));

    nanocbor_fmt_array_indefinite(&enc);
    CU_ASSERT_EQUAL(nanocbor_fmt_float(&enc, 1.75), 3);
    CU_ASSERT_EQUAL(nanocbor_fmt_float(&enc, 1.9990234375), 3);
    CU_ASSERT_EQUAL(nanocbor_fmt_float(&enc, 1.99951171875), 5);
    CU_ASSERT_EQUAL(nanocbor_fmt_float(&enc, 2.0009765625), 5);

    CU_ASSERT_EQUAL(nanocbor_fmt_float(&enc, -1.75), 3);
    CU_ASSERT_EQUAL(nanocbor_fmt_float(&enc, -1.9990234375), 3);
    CU_ASSERT_EQUAL(nanocbor_fmt_float(&enc, -1.99951171875), 5);
    CU_ASSERT_EQUAL(nanocbor_fmt_float(&enc, -2.0009765625), 5);

    nanocbor_fmt_end_indefinite(&enc);
    print_bytestr(buf, nanocbor_encoded_len(&enc));
}

static void test_encode_double_to_float(void)
{
    uint8_t buf[128];
    nanocbor_encoder_t enc;
    nanocbor_encoder_init(&enc, buf, sizeof(buf));

    nanocbor_fmt_array_indefinite(&enc);
    CU_ASSERT_EQUAL(nanocbor_fmt_double(&enc, 1.75), 3);
    CU_ASSERT_EQUAL(nanocbor_fmt_double(&enc, 1.9990234375), 3);
    CU_ASSERT_EQUAL(nanocbor_fmt_double(&enc, 1.99951171875), 5);
    CU_ASSERT_EQUAL(nanocbor_fmt_double(&enc, 2.0009765625), 5);

    CU_ASSERT_EQUAL(nanocbor_fmt_double(&enc, -1.75), 3);
    CU_ASSERT_EQUAL(nanocbor_fmt_double(&enc, -1.9990234375), 3);
    CU_ASSERT_EQUAL(nanocbor_fmt_double(&enc, -1.99951171875), 5);
    CU_ASSERT_EQUAL(nanocbor_fmt_double(&enc, -2.0009765625), 5);

    CU_ASSERT_EQUAL(nanocbor_fmt_double(&enc, 1.00000011920928955078125), 5);
    CU_ASSERT_EQUAL(nanocbor_fmt_double(&enc, DBL_MIN), 9);
    CU_ASSERT_EQUAL(nanocbor_fmt_double(&enc, DBL_MAX), 9);
    CU_ASSERT_EQUAL(nanocbor_fmt_double(&enc, 1e39), 9);

    CU_ASSERT_EQUAL(nanocbor_fmt_double(&enc, FLT_MIN), 5);
    CU_ASSERT_EQUAL(nanocbor_fmt_double(&enc, FLT_MAX), 5);

    nanocbor_fmt_end_indefinite(&enc);
    print_bytestr(buf, nanocbor_encoded_len(&enc));
}

const test_t tests_encoder[] = {
    {
        .f = test_encode_float_specials,
        .n = "Float reduction encoder test",
    },
    {
        .f = test_encode_float_to_half,
        .n = "Float reduction encoder test",
    },
    {
        .f = test_encode_double_to_float,
        .n = "Double reduction encoder test",
    },
    {
        .f = NULL,
        .n = NULL,
    }
};
