#pragma once

#include "types.h"

void estimator_init();
void estimator_update(float dt, const ImuData& imu, const MotorFeedback& fb);
RobotState estimator_get_state();