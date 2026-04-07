#pragma once

#include <Arduino.h>

struct ImuData {
  float accel_angle_deg;
  float gyro_rate_dps;
  float fused_angle_deg;
};

struct MotorFeedback {
  float left_pos_deg;
  float left_vel_dps;
  float right_pos_deg;
  float right_vel_dps;
};

struct RobotState {
  float angle_rad;
  float rate_rad;
  float x_pos;
  float x_vel;
};

struct WheelCommand {
  float left_torque;
  float right_torque;
};

struct ControlOutput {
  float balance_torque;
  WheelCommand wheels;
};