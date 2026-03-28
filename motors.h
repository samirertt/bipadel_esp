#pragma once

bool motors_init();
void motors_enable();
void motors_stop();
void motors_set_wheel_torque(float left_wheel_nm, float right_wheel_nm);