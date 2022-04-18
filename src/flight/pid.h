#pragma once

void rotateErrors();

void pid_init();
void pid_precalc();
void pid_calc();

int next_pid_term(); // Return value : 0 - p, 1 - i, 2 - d
int next_pid_axis(); // Return value : 0 - Roll, 1 - Pitch, 2 - Yaw
int increase_pid();
int decrease_pid();