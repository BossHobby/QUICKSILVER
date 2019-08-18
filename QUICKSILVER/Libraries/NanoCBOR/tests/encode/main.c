/*
 * Copyright (C) 2019 Koen Zandberg
 *
 * This file is subject to the terms and conditions of the GNU Lesser
 * General Public License v2.1. See the file LICENSE in the top level
 * directory for more details.
 */

#include <stdio.h>
#include <stddef.h>
#include <stdint.h>
#include <string.h>
#include <unistd.h>

#include "nanocbor/nanocbor.h"

static void _encode(nanocbor_encoder_t *enc)
{
    nanocbor_fmt_array_indefinite(enc);
    nanocbor_fmt_bool(enc, true);
    nanocbor_fmt_bool(enc, false);
    nanocbor_fmt_uint(enc, UINT32_MAX);
    nanocbor_fmt_int(enc, INT32_MIN);
    nanocbor_fmt_map(enc, 4);
    nanocbor_fmt_uint(enc, 8);
    nanocbor_fmt_int(enc, 30);
    nanocbor_fmt_int(enc, -30);
    nanocbor_fmt_int(enc, 500);
    nanocbor_fmt_int(enc, -500);
    nanocbor_put_tstr(enc, "this is a long string");
    nanocbor_fmt_float(enc, 0.34);
    nanocbor_put_bstr(enc, (uint8_t*)"bytez", sizeof("bytez"));
    nanocbor_fmt_null(enc);
    nanocbor_fmt_end_indefinite(enc);
}

int main(void)
{
    nanocbor_encoder_t enc;
    nanocbor_encoder_init(&enc, NULL, 0);

    _encode(&enc);

    size_t required = nanocbor_encoded_len(&enc);

    uint8_t *buf = malloc(required);
    if (!buf) {
        return -1;
    }

    nanocbor_encoder_init(&enc, buf, required);
    _encode(&enc);

    //printf("Bytes: %u\n", (unsigned)nanocbor_encoded_len(&enc));
    fwrite(buf, 1, nanocbor_encoded_len(&enc), stdout);

    return 0;
}
