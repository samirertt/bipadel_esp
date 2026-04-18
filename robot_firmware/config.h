#pragma once

// ============================================================
// config.h
// ============================================================
// Central configuration file for the robot firmware.
// Every tunable constant lives here so you only need to edit
// one file when adjusting hardware or control parameters.
// ============================================================

#include <Arduino.h>
#include "driver/twai.h"

// ============================================================
// SECTION 1 – ROS 2 BRIDGE FEATURE FLAGS
// ============================================================
// ROS_BRIDGE_ENABLED   : Master switch.  Set 1 to compile in the
//                        Serial-based ROS 2 hardware interface bridge.
//                        Set 0 to use the PS4 controller only.
//
// ROS_BRIDGE_TEST_MODE : When 1, the control loop reads commands
//                        from ROS instead of the PS4 controller,
//                        and publishes encoder feedback over Serial.
//                        Useful for bench-testing ros2_control.
//
// ROS_BRIDGE_DEMO_FEEDBACK : When 1, the bridge simulates encoder
//                            feedback by integrating wheel commands
//                            internally (no real CAN readback).
//                            Useful when CAN motors are not connected.
// ============================================================
#define ROS_BRIDGE_ENABLED       1
#define ROS_BRIDGE_TEST_MODE     1
#define ROS_BRIDGE_DEMO_FEEDBACK 0

// ============================================================
// SECTION 2 – ROS BRIDGE PROTOCOL SETTINGS
// ============================================================
// Serial baud rate shared between the ESP32 and the ROS host.
// Must match the value in your ros2_control hardware interface.
#define ROS_BRIDGE_BAUD            115200UL

// Commands sent by the ROS 2 hardware interface over Serial.
// Single-character opcodes; must match your ROS driver.
#define GET_BAUDRATE               'b'   // ROS asks: "what baud are you?"
#define READ_ENCODERS              'e'   // ROS asks: "give me wheel positions"
#define RESET_ENCODERS             'r'   // ROS asks: "zero the encoders"
#define MOTOR_SPEEDS               'm'   // ROS sends: "m <left_mrad_s> <right_mrad_s>"

// Watchdog: if no 'm' command arrives within this window, zero the
// wheel commands.  Prevents runaway if the ROS host crashes.
#define ROS_CMD_TIMEOUT_MS         500UL

// Maximum expected wheel speed in rad/s from ROS (used for normalization
// to the internal [-1,1] forward/turn command range).
#define ROS_BRIDGE_MAX_WHEEL_RAD_S 10.0f

// Dead-band below which a normalized command is treated as zero.
#define ROS_BRIDGE_CMD_DEADBAND    0.02f

// ============================================================
// SECTION 3 – MAIN CONTROL LOOP
// ============================================================
#define LOOP_HZ          500    // Control loop frequency in Hz
#define SAFE_ANGLE_DEG   20.0f  // Tilt beyond this → safety stop
#define ALPHA            0.98f  // Complementary filter weight (gyro vs accel)

// ============================================================
// SECTION 4 – LQR GAIN SCHEDULING
// ============================================================
// Gains are linearly interpolated between SHORT and TALL based
// on the robot's current leg height.

struct LqrGains {
  float k1;  // Wheel position gain
  float k2;  // Wheel velocity gain
  float k3;  // Body angle gain
  float k4;  // Body rate gain
};

// Physical height bounds for gain interpolation
static constexpr float HEIGHT_TALL_MM  = 502.75f;  // Maximum standing height
static constexpr float HEIGHT_SHORT_MM = 280.0f;   // Minimum squat height

// Tall-stance LQR gains  (tuned at ~400 mm)
static constexpr LqrGains GAINS_TALL = {
  -0.0f,    // k1 – wheel position
  -0.0f,    // k2 – wheel velocity
  -70.0f,   // k3 – body angle
  -2.0f     // k4 – body rate
};

// Short-stance LQR gains  (tuned at ~280 mm; re-tune after mechanical changes)
static constexpr LqrGains GAINS_SHORT = {
   0.0f,    // k1
   0.0f,    // k2
  -15.0f,   // k3
  -4.50f    // k4
};

// ============================================================
// SECTION 5 – ROBOT / DRIVETRAIN
// ============================================================
#define GEAR_RATIO           8.0f   // Motor-to-joint reduction ratio
#define RIGHT_WHEEL_INVERT   true   // Flip sign on right wheel torque output

// Odometry unit selection:
//   false → positions in radians  (default, matches ros2_control convention)
//   true  → positions in metres   (multiply by WHEEL_RADIUS_M)
#define USE_LINEAR_POSITION_UNITS false
static constexpr float WHEEL_RADIUS_M = 0.1f;  // Wheel radius in metres

// ============================================================
// SECTION 6 – HARDWARE PINS
// ============================================================
#define SDA_PIN    21   // I2C data  (IMU)
#define SCL_PIN    22   // I2C clock (IMU)
#define CAN_RX_PIN  4   // TWAI / CAN receive
#define CAN_TX_PIN  5   // TWAI / CAN transmit

// ============================================================
// SECTION 7 – IMU
// ============================================================
#define GYRO_ADDR       0x68   // I2C address of ITG-3200 gyroscope
#define ANGLE_OFFSET_DEG 3.3f  // Static pitch trim to level the robot
