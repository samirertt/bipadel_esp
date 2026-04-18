#pragma once

#include "types.h"
#include "config.h" 

void lqr_controller_init();

// Pass the dynamic gains into the function
float lqr_compute_balance_rad_s(const RobotState& state, const LqrGains& dynamic_gains);