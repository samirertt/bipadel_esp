#include "lqr_controller.h"
#include "config.h"

void lqr_controller_init() {
}

float lqr_compute_balance_rad_s(const RobotState& s) {
  return -(K1 * s.x_pos +
           K2 * s.x_vel +
           K3 * s.angle_rad +
           K4 * s.rate_rad);
}