#include "driver/rcc.h"

// Peripherals are clocked by default; enabling one takes it out of reset.
// reg holds the RESETS_RESET_*_BITS of the peripheral.
void rcc_enable(rcc_reg_t reg) {
  unreset_block_wait(reg);
}
