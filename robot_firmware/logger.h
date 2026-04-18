#pragma once

#include "types.h"

void logger_init();

void logger_log(
    unsigned long t_ms,
    float dt,
    const ImuData& imu,
    const MotorFeedback& fb,
    const RobotState& state,
    const ControlOutput& out,
    bool safety_stop,
    int cmd_currently
);