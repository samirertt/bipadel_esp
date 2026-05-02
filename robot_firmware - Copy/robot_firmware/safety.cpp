#include "safety.h"
#include "config.h"
#include <math.h>

bool safety_angle_ok_deg(float angle_deg) {
  return fabs(angle_deg) <= SAFE_ANGLE_DEG;
}

bool safety_motors_ok(const MotorFeedback& fb, uint32_t current_time_ms) {
  // --- 1. Disconnection Timeout (Heartbeat Monitor) ---
  // If any of the 6 motors haven't spoken in 500ms, they are physically offline.
  for (int i = 1; i <= 6; i++) {
     if ((current_time_ms - fb.last_msg_time_ms[i]) > 500) {
         return false; 
     }
  }

  // --- 2. Hardware Error Codes ---
  // A value > 0 means the motor's internal driver shut itself down (overheat, overcurrent, etc)
  if (fb.left_wheel_error > 0)  return false;
  if (fb.right_wheel_error > 0) return false;
  if (fb.left_knee_error > 0)   return false;
  if (fb.right_knee_error > 0)  return false;
  if (fb.left_torso_error > 0)  return false;
  if (fb.right_torso_error > 0) return false;

  return true; // All 6 motors are present, connected, and healthy
}