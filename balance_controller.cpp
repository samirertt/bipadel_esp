#include "balance_controller.h"
#include "config.h"
#include "utils.h"

static BalanceControllerType g_type = BalanceControllerType::CASCADE_PID;

// Outer loop state
static float g_pos_integral = 0.0f;

// Inner loop state
static float g_angle_integral = 0.0f;

void balance_controller_init() {
  g_type = BalanceControllerType::CASCADE_PID;
  g_pos_integral = 0.0f;
  g_angle_integral = 0.0f;
}

void balance_controller_set_type(BalanceControllerType type) {
  g_type = type;
}

BalanceControllerType balance_controller_get_type() {
  return g_type;
}

static float compute_lqr(const RobotState& s) {
  return -(K1 * s.x_pos +
           K2 * s.x_vel +
           K3 * s.angle_rad +
           K4 * s.rate_rad);
}

static float compute_cascade_pid(const RobotState& s) {
  const float dt = 1.0f / LOOP_HZ;

  // Outer loop: position -> target angle
  const float KP_POS = 0.8f;
  const float KI_POS = 0.15f;
  const float KD_POS = 0.25f;

  g_pos_integral += s.x_pos * dt;
  g_pos_integral = clampf(g_pos_integral, -0.5f, 0.5f);

  float angle_target =
      -(KP_POS * s.x_pos +
        KI_POS * g_pos_integral +
        KD_POS * s.x_vel);

  angle_target = clampf(angle_target, deg2rad(-6.0f), deg2rad(6.0f));

  // Inner loop: angle -> torque
  const float KP_ANG = 18.0f;
  const float KI_ANG = 0.0f;
  const float KD_ANG = 1.2f;

  float angle_error = angle_target - s.angle_rad;

  g_angle_integral += angle_error * dt;
  g_angle_integral = clampf(g_angle_integral, -0.2f, 0.2f);

  float torque =
      KP_ANG * angle_error +
      KI_ANG * g_angle_integral -
      KD_ANG * s.rate_rad;

  return torque;
}

float balance_controller_compute(const RobotState& state) {
  if (g_type == BalanceControllerType::LQR) {
    return compute_lqr(state);
  }

  return compute_cascade_pid(state);
}