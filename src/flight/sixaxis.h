#pragma once

#include <stdbool.h>
#include <stdint.h>

bool sixaxis_detect();
void sixaxis_init();
void sixaxis_read();

void sixaxis_gyro_cal();
void sixaxis_acc_cal();
