#include "control_manager.h"
#include "lqr_controller.h"
#include "config.h"
#include "utils.h"
#include <Arduino.h>

// ==========================================
// DRIVING TUNING PARAMETERS
// ==========================================
// IMPORTANT: Start these at 0.0f until your LQR balances perfectly still!
static constexpr float KP_VEL = 0.02f;  // Proportional: Initial lean kick
static constexpr float KI_VEL = 0.001f;  // Integral: Builds lean to overcome friction

static constexpr float MAX_TARGET_VELOCITY = 7.50f;     // Max speed rad/s  (1.53m/s @ 15rad/s)
static constexpr float MAX_PITCH_OFFSET_RAD = 0.105f;    // Max allowed lean angle (~8.5 deg)
static constexpr float TURN_TORQUE_NM = 1.0f;           // Max turning differential torque         

static float velocity_integral = 0.0f;

// --- NEW: Virtual Anchor Variables ---
static float target_x_pos = 0.0f;
static bool is_position_locked = true;

void control_manager_init() {
  velocity_integral = 0.0f;
  target_x_pos = 0.0f;
  is_position_locked = true;
}

ControlOutput control_manager_update(float dt, const RobotState& state, float forward_cmd, float turn_cmd) {
  ControlOutput out = {};

  // ==========================================
  // 1. OUTER LOOP (Velocity -> Required Lean Angle)
  // ==========================================
  float target_vel = forward_cmd * MAX_TARGET_VELOCITY;
  float vel_error = target_vel - state.x_vel;
  
  velocity_integral += vel_error * dt;
  velocity_integral = clampf(velocity_integral, -10.0f, 10.0f); 

  float required_lean_angle = (KP_VEL * vel_error) + (KI_VEL * velocity_integral);
  required_lean_angle = clampf(required_lean_angle, -MAX_PITCH_OFFSET_RAD, MAX_PITCH_OFFSET_RAD);


  // ==========================================
  // 2. THE K1 POSITION FIX (Virtual Anchor)
  // ==========================================
  bool is_driving = (abs(forward_cmd) > 0.05f);

  if (is_driving) {
      // User is driving. Drag the anchor with the robot so K1 doesn't fight back.
      target_x_pos = state.x_pos;
      is_position_locked = false;
  } else {
      // User let go of the joystick. Wait for the robot to slow down before anchoring.
      if (!is_position_locked && abs(state.x_vel) < 0.5f) {
          target_x_pos = state.x_pos; // Drop the anchor at this new location
          is_position_locked = true;
      } else if (!is_position_locked) {
          // Still braking! Keep dragging the anchor so it doesn't violently reverse
          target_x_pos = state.x_pos; 
      }
  }


  // ==========================================
  // 3. INNER LOOP (LQR Balance + Forward Lean)
  // ==========================================
  RobotState target_state = state;
  
  // Apply the lean angle for the PI loop
  target_state.angle_rad -= required_lean_angle;
  
  // Apply the position fix. The LQR now calculates error from our Virtual Anchor.
  target_state.x_pos = state.x_pos - target_x_pos;
  
  float balance_torque = lqr_compute_balance_rad_s(target_state); 

  // Friction compensation
  const float FRICTION_OFFSET = 0.45f; 
  float stall_speed_threshold = 0.1f; 
  bool is_stalled = (abs(state.x_vel) < stall_speed_threshold);

  if (balance_torque > 0.02f) {
      if (is_stalled) balance_torque += FRICTION_OFFSET;
  } else if (balance_torque < -0.02f) {
      if (is_stalled) balance_torque -= FRICTION_OFFSET;
  } else {
      balance_torque = 0.0f;
  }

  out.balance_torque = balance_torque;


  // ==========================================
  // 4. TURNING (Differential Torque)
  // ==========================================
  float turn_offset = turn_cmd * TURN_TORQUE_NM;

  out.wheels.left_torque  = balance_torque - turn_offset;
  out.wheels.right_torque = balance_torque + turn_offset;

  out.wheels.left_torque  = clampf(out.wheels.left_torque,  -8.0f, 8.0f);
  out.wheels.right_torque = clampf(out.wheels.right_torque, -8.0f, 8.0f);

  return out;
}