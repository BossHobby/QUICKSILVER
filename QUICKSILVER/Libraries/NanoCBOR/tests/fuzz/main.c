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

char buffer[256];

static int _parse_type(nanocbor_value_t *value, unsigned indent);

static void _indent(unsigned indent)
{
    for(unsigned i = 0; i < indent; i++) {
        printf("  ");
    }
}

void _parse_cbor(nanocbor_value_t *it, unsigned indent)
{
    while (!nanocbor_at_end(it)) {
        _indent(indent);
        int res = _parse_type(it, indent);
        printf(",\n");
        if (res < 0) {
            printf("Err\n");
            break;
        }
    }
}

void _parse_map(nanocbor_value_t *it, unsigned indent)
{
    while (!nanocbor_at_end(it)) {
        _indent(indent);
        int res = _parse_type(it, indent);
        printf(": ");
        if (res < 0) {
            printf("Err\n");
            break;
        }
        res = _parse_type(it, indent);
        if (res < 0) {
            printf("Err\n");
            break;
        }
        printf(",\n");
    }
}

static int _parse_type(nanocbor_value_t *value, unsigned indent)
{
    uint8_t type = nanocbor_get_type(value);
    if (indent > 10) {
        return -2;
    }
    switch (type) {
        case NANOCBOR_TYPE_UINT:
            {
                uint32_t uint;
                if (nanocbor_get_uint32(value, &uint) >= 0) {
                    printf("%lu", (long unsigned)uint);
                }
                else {
                    return -1;
                }
            }
            break;
        case NANOCBOR_TYPE_NINT:
            {
                int32_t int32;
                if (nanocbor_get_int32(value, &int32) >= 0) {
                    printf("%ld", (long unsigned)int32);
                }
                else {
                    return -1;
                }
            }
            break;
        case NANOCBOR_TYPE_BSTR:
            {
                const uint8_t *buf = NULL;
                size_t len;
                if (nanocbor_get_bstr(value, &buf, &len) >= 0 && buf) {
                    size_t iter = 0;
                    printf("\"");
                    while(iter < len) {
                        printf("0x%.2x, ", buf[iter]);
                        iter++;
                    }
                    printf("\"");
                }
                else {
                    return -1;
                }
            }
            break;
        case NANOCBOR_TYPE_TSTR:
            {
                const uint8_t *buf;
                size_t len;
                if (nanocbor_get_tstr(value, &buf, &len) >= 0) {
                    printf("\"%.*s\"", (int)len, buf);
                }
                else {
                    return -1;
                }
            }
            break;
        case NANOCBOR_TYPE_ARR:
            {
                nanocbor_value_t arr;
                if (nanocbor_enter_array(value, &arr) >= 0) {
                    printf("[\n");
                    _parse_cbor(&arr, indent + 1);
                    nanocbor_leave_container(value, &arr);
                    _indent(indent);
                    printf("]");
                }
                else {
                    return -1;
                }
            }
            break;
        case NANOCBOR_TYPE_MAP:
            {
                nanocbor_value_t map;
                if (nanocbor_enter_map(value, &map) >= NANOCBOR_OK) {;
                    printf("{\n");
                    _parse_map(&map, indent + 1);
                    nanocbor_leave_container(value, &map);
                    _indent(indent);
                    printf("}");
                }
                else {
                    return -1;
                }
            }
            break;
        case NANOCBOR_TYPE_FLOAT:
            {
                bool test;
                if (nanocbor_get_bool(value, &test) >= NANOCBOR_OK) {
                    test ? printf("True") : printf("False");
                }
                else if (nanocbor_get_null(value) >= NANOCBOR_OK) {
                    printf("NULL");
                }
                else if (nanocbor_skip_simple(value) >= 0) {
                    printf("Unsupported float");
                }
                else {
                    return -1;
                }
                break;
            }
        default:
            printf("Unsupported type\n");
            return -1;

    }
    return 1;
}

int main(void)
{
    ssize_t len = read(STDIN_FILENO, buffer, sizeof(buffer));
    printf("Reading %ld bytes from stdin\n", (long signed)len);
    if (len < 0) {
        return -1;
    }

    nanocbor_value_t it;
    nanocbor_decoder_init(&it, (uint8_t*)buffer, len);
    while (!nanocbor_at_end(&it)) {
        printf("advancing\n");
        if(nanocbor_skip(&it) < 0) {
            break;
        }
    }

    nanocbor_decoder_init(&it, (uint8_t*)buffer, len);
    printf("parsing cbor\n");
    _parse_cbor(&it, 0);
    printf("Done parsing cbor\n");

    return 0;
}
