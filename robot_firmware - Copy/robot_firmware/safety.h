#pragma once
#include "types.h"

bool safety_angle_ok_deg(float angle_deg);

// Add current_time_ms to the signature
bool safety_motors_ok(const MotorFeedback& fb, uint32_t current_time_ms);