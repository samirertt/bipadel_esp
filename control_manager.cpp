#include "control_manager.h"
#include "balance_controller.h"
#include "command_filter.h"
#include "config.h"
#include "utils.h"

static float g_prev_left = 0.0f;
static float g_prev_right = 0.0f;

void control_manager_init() {
  g_prev_left = 0.0f;
  g_prev_right = 0.0f;
}

ControlOutput control_manager_update(const RobotState& state, float forward_torque) {
  ControlOutput out = {};

  float balance_torque = balance_controller_compute(state);

  float left_cmd  = balance_torque + forward_torque;
  float right_cmd = balance_torque + forward_torque;

  left_cmd  = clampf(left_cmd,  -MAX_WHEEL_TORQUE_NM, MAX_WHEEL_TORQUE_NM);
  right_cmd = clampf(right_cmd, -MAX_WHEEL_TORQUE_NM, MAX_WHEEL_TORQUE_NM);

  const float max_step = 0.02f;
  left_cmd  = command_filter_limit(left_cmd,  g_prev_left,  max_step);
  right_cmd = command_filter_limit(right_cmd, g_prev_right, max_step);

  g_prev_left = left_cmd;
  g_prev_right = right_cmd;

  out.balance_torque = balance_torque;
  out.forward_torque = forward_torque;
  out.wheels.left_torque = left_cmd;
  out.wheels.right_torque = right_cmd;

  return out;
}