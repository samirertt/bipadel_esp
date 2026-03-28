#pragma once
#include <Arduino.h>

enum class RobotMode : uint8_t {
  INIT = 0,
  CALIBRATING,
  STANDBY,
  ARMED,
  BALANCING,
  FAULT
};

struct ImuData {
  float accel_angle_deg = 0.0f;
  float gyro_rate_dps   = 0.0f;
  float fused_angle_deg = 0.0f;
};

struct MotorFeedback {
  float left_pos_deg  = 0.0f;
  float left_vel_dps  = 0.0f;
  float right_pos_deg = 0.0f;
  float right_vel_dps = 0.0f;
};

struct RobotState {
  float angle_rad = 0.0f;
  float rate_rad  = 0.0f;
  float x_pos     = 0.0f;
  float x_vel     = 0.0f;
};

struct WheelCommand {
  float left_torque  = 0.0f;
  float right_torque = 0.0f;
};

struct ControlOutput {
  float balance_torque = 0.0f;
  float forward_torque = 0.0f;
  WheelCommand wheels;
};