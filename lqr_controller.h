#pragma once

#include "types.h"

void lqr_controller_init();
float lqr_compute_balance_rad_s(const RobotState& state);