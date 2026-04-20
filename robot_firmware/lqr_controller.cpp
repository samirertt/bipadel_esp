#include "lqr_controller.h"
#include <math.h>

void lqr_controller_init() {
}

// Use the passed-in gains (g)
float lqr_compute_balance_rad_s(const RobotState& s, const LqrGains& g) {
  float pos_error = s.x_pos;

  // Positional deadband: Disables K1 restoring force within a +/- 0.75 radian boundary.
  if (fabs(pos_error) < 0.25f) {
      pos_error = 0.0f;
  }

  // Multiply state by the dynamic, real-time interpolated gains
  return (g.k1 * pos_error +
          g.k2 * s.x_vel +
          g.k3 * s.angle_rad +
          g.k4 * s.rate_rad);
}