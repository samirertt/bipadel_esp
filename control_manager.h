#pragma once
#include "types.h"

void control_manager_init();
ControlOutput control_manager_update(const RobotState& state, float forward_torque);