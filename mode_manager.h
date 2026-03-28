#pragma once
#include "types.h"

void mode_manager_init();
void mode_manager_update(bool imu_ok, bool can_ok, bool angle_safe, bool arm_request);
RobotMode mode_manager_get();
bool mode_manager_allows_torque();