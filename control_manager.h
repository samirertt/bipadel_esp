#pragma once

#include "types.h"

void control_manager_init();

// Updated to include time step and controller commands
ControlOutput control_manager_update(float dt, const RobotState& state, float forward_cmd, float turn_cmd);