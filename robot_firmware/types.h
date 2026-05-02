#pragma once
#include <Arduino.h>

struct ImuData {
  float accel_x;
  float accel_y;
  float accel_z;

  float gyro_x;
  float gyro_y;
  float gyro_z;

  float accel_angle_deg;
  float gyro_rate_dps;
  float fused_angle_deg;

  float yaw_rate_dps;
  float yaw_angle_deg;
};

struct MotorFeedback {
  float left_pos_deg;
  float left_vel_dps;
  float right_pos_deg;
  float right_vel_dps;
  
  float left_knee_pos_deg;
  float right_knee_pos_deg;
  float left_torso_pos_deg;
  float right_torso_pos_deg;

  uint32_t left_wheel_error;
  uint32_t right_wheel_error;
  uint32_t left_knee_error;
  uint32_t right_knee_error;
  uint32_t left_torso_error;
  uint32_t right_torso_error;

  // --- NEW: Tracks the last time we received data from Node 1-6 ---
  uint32_t last_msg_time_ms[7]; 
};

struct RobotState {
  float angle_rad;
  float rate_rad;
  float x_pos;
  float x_vel;
  float yaw_angle_deg; 
  float yaw_rate_dps;  
};

struct WheelCommand {
  float left_torque;
  float right_torque;
};

struct LegCommand {
  float left_knee_rad;
  float right_knee_rad;
  float left_torso_rad;
  float right_torso_rad;
};

struct ControlOutput {
  float balance_torque;
  WheelCommand wheels;
  LegCommand legs;      
};