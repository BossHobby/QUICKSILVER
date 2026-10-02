#pragma once

void nav_update_multi(float dt, bool gps_valid, bool home_valid);
void nav_rth_start();
void nav_rth_stop();

bool nav_rth_home_valid();
bool nav_gps_ready();
bool nav_altitude_ready();
