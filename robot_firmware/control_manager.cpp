#include "control_manager.h"
#include "lqr_controller.h"
#include "config.h"
#include "utils.h"
#include <Arduino.h>

static constexpr float KP_VEL = 0.0f;  // 0.02
static constexpr float KI_VEL = 0.0f; // 0.001
static constexpr float MAX_TARGET_VELOCITY = 2.50f;     // it was 7.50
static constexpr float MAX_PITCH_OFFSET_RAD = 0.105f;    
static constexpr float TURN_TORQUE_NM = 1.0f;                    

static float velocity_integral = 0.0f;
static float target_x_pos = 0.0f;
static bool is_position_locked = true;

static float target_yaw_deg = 0.0f;
static constexpr float KP_YAW = 0.0f;  //0.08
static constexpr float KD_YAW = 0.0f; //0.005

void control_manager_init() {
  velocity_integral = 0.0f;
  target_x_pos = 0.0f;
  is_position_locked = true;
  target_yaw_deg = 0.0f;
}

static float map_float(float x, float in_min, float in_max, float out_min, float out_max) {
  return (x - in_min) * (out_max - out_min) / (in_max - in_min) + out_min;
}

// NEW: Added standby_mode flag
ControlOutput control_manager_update(float dt, const RobotState& state, float forward_cmd, float turn_cmd, bool r3_reset, float current_height_mm, bool standby_mode) {
  ControlOutput out = {};

  // --- NEW: STANDBY OVERRIDE ---
  if (standby_mode) {
    // Continuously lock the internal targets to wherever the robot currently is.
    // This prevents the math from "winding up" while it rests on the stand.
    velocity_integral = 0.0f;
    target_x_pos = state.x_pos; 
    target_yaw_deg = state.yaw_angle_deg;
    is_position_locked = true;

    // Send exactly zero torque to the balancing wheels
    out.balance_torque = 0.0f;
    out.wheels.left_torque = 0.0f;
    out.wheels.right_torque = 0.0f;
    return out;
  }

  // ... (Keep the rest of your normal balancing logic exactly the same below here) ...
  float target_vel = forward_cmd * MAX_TARGET_VELOCITY;
  float vel_error = target_vel - state.x_vel;
  velocity_integral += vel_error * dt;
  velocity_integral = clampf(velocity_integral, -10.0f, 10.0f); 
  float required_lean_angle = (KP_VEL * vel_error) + (KI_VEL * velocity_integral);
  required_lean_angle = clampf(required_lean_angle, -MAX_PITCH_OFFSET_RAD, MAX_PITCH_OFFSET_RAD);

  bool is_driving = (abs(forward_cmd) > 0.05f);
  if (is_driving) {
      target_x_pos = state.x_pos;
      is_position_locked = false;
  } else {
      if (!is_position_locked && abs(state.x_vel) < 0.5f) {
          target_x_pos = state.x_pos; 
          is_position_locked = true;
      } else if (!is_position_locked) target_x_pos = state.x_pos; 
  }

  RobotState target_state = state;
  target_state.angle_rad -= required_lean_angle;
  target_state.x_pos = state.x_pos - target_x_pos;
  
  float safe_h = clampf(current_height_mm, HEIGHT_SHORT_MM, HEIGHT_TALL_MM);

  LqrGains current_gains;
  current_gains.k1 = map_float(safe_h, HEIGHT_SHORT_MM, HEIGHT_TALL_MM, GAINS_SHORT.k1, GAINS_TALL.k1);
  current_gains.k2 = map_float(safe_h, HEIGHT_SHORT_MM, HEIGHT_TALL_MM, GAINS_SHORT.k2, GAINS_TALL.k2);
  current_gains.k3 = map_float(safe_h, HEIGHT_SHORT_MM, HEIGHT_TALL_MM, GAINS_SHORT.k3, GAINS_TALL.k3);
  current_gains.k4 = map_float(safe_h, HEIGHT_SHORT_MM, HEIGHT_TALL_MM, GAINS_SHORT.k4, GAINS_TALL.k4);

  float balance_torque = lqr_compute_balance_rad_s(target_state, current_gains); 

  const float FRICTION_OFFSET = 0.40f; 
  bool is_stalled = (abs(state.x_vel) < 0.1f);
  if (balance_torque > 0.02f) {
      if (is_stalled) balance_torque += FRICTION_OFFSET;
  } else if (balance_torque < -0.02f) {
      if (is_stalled) balance_torque -= FRICTION_OFFSET;
  } else balance_torque = 0.0f;

  if (r3_reset) target_yaw_deg = state.yaw_angle_deg;
  else if (abs(turn_cmd) > 0.05f) target_yaw_deg -= turn_cmd * (120.0f * dt); 

  float yaw_error = target_yaw_deg - state.yaw_angle_deg;
  float yaw_lock_torque = (KP_YAW * yaw_error) - (KD_YAW * state.yaw_rate_dps);
  yaw_lock_torque = clampf(yaw_lock_torque, -TURN_TORQUE_NM, TURN_TORQUE_NM);

  out.balance_torque = balance_torque;
  out.wheels.left_torque  = clampf(balance_torque - yaw_lock_torque, -8.0f, 8.0f);
  out.wheels.right_torque = clampf(balance_torque + yaw_lock_torque, -8.0f, 8.0f);

  return out;
}