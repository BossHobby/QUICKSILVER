#pragma once

#include <stdint.h>

uint8_t sixaxis_init();
void sixaxis_read();

void sixaxis_gyro_cal();
void sixaxis_acc_cal();
