#pragma once
#include "types.h"

struct JointAngles {
    float knee_angle_deg;
    float torso_angle_deg;
    float alpha_deg;
    float com_y;
    bool  valid;
};

void kinematics_init();
JointAngles kinematics_compute(float target_height_mm);
