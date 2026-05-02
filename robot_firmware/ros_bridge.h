#pragma once

#include <Arduino.h>
#include "types.h"

bool ros_bridge_init(unsigned long baudrate = 921600);
void ros_bridge_update();

void ros_bridge_get_commands(float &forward_cmd, float &turn_cmd, bool &has_ros_cmd);
void ros_bridge_set_timeout_ms(unsigned long timeout_ms);

bool ros_bridge_reset_requested();
void ros_bridge_clear_reset_request();

bool ros_bridge_motor_enabled();
uint8_t ros_bridge_control_mode();
float ros_bridge_left_target_rad_s();
float ros_bridge_right_target_rad_s();

void ros_bridge_publish_feedback(const MotorFeedback &fb, const ImuData &Imu);
void ros_bridge_publish_full_state(const MotorFeedback &fb, float pitch, float gyro_y);