#pragma once

bool motors_init();
void motors_enable();
void motors_stop();
void motors_set_wheel_torque(float left_wheel_nm, float right_wheel_nm);

// NEW: Accepts degrees directly so we can handle absolute offsets in the main loop
void motors_set_leg_positions_deg(float l_knee_deg, float r_knee_deg, float l_torso_deg, float r_torso_deg);