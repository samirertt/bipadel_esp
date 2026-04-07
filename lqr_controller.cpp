#include "lqr_controller.h"
#include "config.h"
#include <math.h>

void lqr_controller_init() {
}

float lqr_compute_balance_rad_s(const RobotState& s) {
  float pos_error = s.x_pos;

  // Positional deadband: Disables K1 restoring force within a +/- 0.15 radian boundary.
  // Adjust this threshold to match the physical backlash and static friction of the chassis.
  if (fabs(pos_error) < 0.75f) {
      pos_error = 0.0f;
  }

  return (K1 * pos_error +
          K2 * s.x_vel +
          K3 * s.angle_rad +
          K4 * s.rate_rad);
}