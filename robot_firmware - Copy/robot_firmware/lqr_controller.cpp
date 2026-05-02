#include "lqr_controller.h"
#include <math.h>

// Divisor to lower the frequency of k1 and k2.
// Since the main loop is 500Hz, a divider of 5 runs k1/k2 at 50Hz.
// Adjust this value to tune the priority of the balance loop.
static constexpr int POS_VEL_UPDATE_DIVIDER = 2;

void lqr_controller_init() {
}

// Use the passed-in gains (g)
float lqr_compute_balance_rad_s(const RobotState& s, const LqrGains& g) {
  static float last_pos_vel_term = 0.0f;
  static int tick_counter = 0;

  // --- HIGH FREQUENCY LOOP (e.g., 500 Hz) ---
  // Always evaluate angle and rate to give maximum priority to balancing
  float angle_rate_term = g.k3 * s.angle_rad + g.k4 * s.rate_rad;

  // --- LOW FREQUENCY LOOP (e.g., 50 Hz) ---
  // Only evaluate position and velocity periodically
  if (tick_counter == 0) {
      float pos_error = s.x_pos;

      // Positional deadband: Disables K1 restoring force within a +/- 0.25 radian boundary.
      if (fabs(pos_error) < 0.25f) {
          pos_error = 0.0f;
      }

      // Calculate and hold the k1/k2 term
      last_pos_vel_term = (g.k1 * pos_error) + (g.k2 * s.x_vel);
  }

  // Increment and wrap the tick counter
  tick_counter++;
  if (tick_counter >= POS_VEL_UPDATE_DIVIDER) {
      tick_counter = 0;
  }

  // Combine the high-frequency balance terms with the low-frequency position terms
  return angle_rate_term + last_pos_vel_term;
}