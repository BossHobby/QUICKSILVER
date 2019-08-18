/*
 * Copyright (C) 2019 Koen Zandberg
 *
 * This file is subject to the terms and conditions of the GNU Lesser
 * General Public License v2.1. See the file LICENSE in the top level
 * directory for more details.
 */

#include <time.h>
#include <stdio.h>
#include <stdlib.h>

#include "CUnit/CUnit.h"
#include "CUnit/Basic.h"
#include "test.h"

extern const test_t tests_decoder[];
extern const test_t tests_encoder[];

static int add_tests(CU_pSuite pSuite, const test_t* tests)
{
   /* add the tests to the suite */
    for(int i = 0; tests[i].n !=NULL; i++) {
        if(!(CU_add_test(pSuite, tests[i].n, tests[i].f))) {
            printf("Error adding function %s\n",tests[i].n);
            CU_cleanup_registry();
            return CU_get_error();
        }
    }
    return 0;
}

int main()
{
    CU_pSuite pSuite = NULL;
    if (CUE_SUCCESS != CU_initialize_registry())
        return CU_get_error();
    pSuite = CU_add_suite("Nanocbor decode", NULL, NULL);
    if (NULL == pSuite) {
        CU_cleanup_registry();
        return CU_get_error();
    }
    add_tests(pSuite, tests_decoder);

    pSuite = CU_add_suite("Nanocbor encode", NULL, NULL);
    if (NULL == pSuite) {
        CU_cleanup_registry();
        return CU_get_error();
    }
    add_tests(pSuite, tests_encoder);

    CU_basic_set_mode(CU_BRM_VERBOSE);
    CU_basic_run_tests();
    printf("\n");

    CU_cleanup_registry();
    if (CU_get_number_of_failure_records()) {
        exit(2);
    }
    return CU_get_error();
}
