#include "mode_manager.h"
#include <Arduino.h>

static RobotMode g_mode = RobotMode::INIT;
static unsigned long g_mode_enter_ms = 0;

static void set_mode(RobotMode mode) {
  g_mode = mode;
  g_mode_enter_ms = millis();
}

void mode_manager_init() {
  set_mode(RobotMode::CALIBRATING);
}

void mode_manager_update(bool imu_ok, bool can_ok, bool angle_safe, bool arm_request) {
  unsigned long now = millis();

  switch (g_mode) {
    case RobotMode::INIT:
      set_mode(RobotMode::CALIBRATING);
      break;

    case RobotMode::CALIBRATING:
      if (!imu_ok || !can_ok) {
        set_mode(RobotMode::FAULT);
      } else if ((now - g_mode_enter_ms) > 1500) {
        set_mode(RobotMode::STANDBY);
      }
      break;

    case RobotMode::STANDBY:
      if (!imu_ok || !can_ok) {
        set_mode(RobotMode::FAULT);
      } else if (arm_request && angle_safe) {
        set_mode(RobotMode::ARMED);
      }
      break;

    case RobotMode::ARMED:
      if (!imu_ok || !can_ok || !angle_safe) {
        set_mode(RobotMode::FAULT);
      } else {
        set_mode(RobotMode::BALANCING);
      }
      break;

    case RobotMode::BALANCING:
      if (!imu_ok || !can_ok || !angle_safe) {
        set_mode(RobotMode::FAULT);
      }
      break;

    case RobotMode::FAULT:
      if (imu_ok && can_ok && angle_safe && !arm_request) {
        set_mode(RobotMode::STANDBY);
      }
      break;
  }
}

RobotMode mode_manager_get() {
  return g_mode;
}

bool mode_manager_allows_torque() {
  return g_mode == RobotMode::BALANCING;
}