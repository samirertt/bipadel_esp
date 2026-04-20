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
  float r_pos_fixed = -fb.right_pos_deg;
  float r_vel_fixed = fb.right_vel_dps;

  if (!g_zeroed) {
    g_left_pos0_deg  = fb.left_pos_deg;
    g_right_pos0_deg = r_pos_fixed;      
    g_zeroed = true;
  }

  float left_pos_deg  = fb.left_pos_deg  - g_left_pos0_deg;
  float right_pos_deg = r_pos_fixed - g_right_pos0_deg; 

  float avg_pos_deg = 0.5f * (left_pos_deg + right_pos_deg);
  float avg_vel_deg = 0.5f * (fb.left_vel_dps + r_vel_fixed); 

  g_state.angle_rad = deg2rad(imu.fused_angle_deg);
  g_state.rate_rad  = deg2rad(imu.gyro_rate_dps);
  g_state.yaw_angle_deg = imu.yaw_angle_deg;
  g_state.yaw_rate_dps  = imu.yaw_rate_dps;

  if (USE_LINEAR_POSITION_UNITS) {
    g_state.x_pos = -deg2rad(avg_pos_deg) * WHEEL_RADIUS_M;
    g_state.x_vel = -deg2rad(avg_vel_deg) * WHEEL_RADIUS_M;
  } else {
    g_state.x_pos = -deg2rad(avg_pos_deg);
    g_state.x_vel = -deg2rad(avg_vel_deg);
  }
}

RobotState estimator_get_state() { return g_state; }