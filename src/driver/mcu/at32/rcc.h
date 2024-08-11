#pragma once

#define RCC_ENCODE(periph) (rcc_reg_t)(CRM_##periph##_PERIPH_CLOCK)
