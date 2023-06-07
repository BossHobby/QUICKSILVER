#include "driver/rcc.h"

void rcc_enable(rcc_reg_t reg) {
  crm_periph_clock_enable(reg, TRUE);
}