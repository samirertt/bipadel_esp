#pragma once

#include "types.h"

void control_manager_init();

// NEW: Added standby_mode to the end of the signature
ControlOutput control_manager_update(float dt, const RobotState& state, float forward_cmd, float turn_cmd, bool r3_reset, float current_height_mm, bool standby_mode);