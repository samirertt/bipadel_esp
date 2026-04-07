#include "estimator.h"
#include "config.h"
#include "utils.h"

static RobotState g_state = {0};

static bool g_zeroed = false;
static float g_left_pos0_deg = 0.0f;
static float g_right_pos0_deg = 0.0f;

void estimator_init() {
  g_state = {0};
  g_zeroed = false;
}

void estimator_update(float dt, const ImuData& imu, const MotorFeedback& fb) {
  (void)dt;

  // --- THE BULLETPROOF FIX ---
  // We permanently force the right wheel to match the left wheel's forward direction.
  // No 'if' statements. It cannot fail.
  float r_pos_fixed = -fb.right_pos_deg;
  float r_vel_fixed = -fb.right_vel_dps;

  if (!g_zeroed) {
    g_left_pos0_deg  = fb.left_pos_deg;
    g_right_pos0_deg = r_pos_fixed;      
    g_zeroed = true;
  }

  float left_pos_deg  = fb.left_pos_deg  - g_left_pos0_deg;
  float right_pos_deg = r_pos_fixed - g_right_pos0_deg; 

  // The math will now successfully add them together instead of canceling to 0.00!
  float avg_pos_deg = 0.5f * (left_pos_deg + right_pos_deg);
  float avg_vel_deg = 0.5f * (fb.left_vel_dps + r_vel_fixed); 

  float avg_pos_rad = deg2rad(avg_pos_deg);
  float avg_vel_rad = deg2rad(avg_vel_deg);

  g_state.angle_rad = deg2rad(imu.fused_angle_deg);
  g_state.rate_rad  = deg2rad(imu.gyro_rate_dps);

  if (USE_LINEAR_POSITION_UNITS) {
    g_state.x_pos = avg_pos_rad * WHEEL_RADIUS_M;
    g_state.x_vel = avg_vel_rad * WHEEL_RADIUS_M;
  } else {
    g_state.x_pos = avg_pos_rad;
    g_state.x_vel = avg_vel_rad;
  }
}

RobotState estimator_get_state() {
  return g_state;
}