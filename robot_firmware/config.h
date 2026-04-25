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
#define ROS_BRIDGE_ENABLED       0
#define ROS_BRIDGE_TEST_MODE     0
#define ROS_BRIDGE_DEMO_FEEDBACK 0

// ============================================================
// SECTION 2 – ROS BRIDGE PROTOCOL SETTINGS
// ============================================================
#define ROS_BRIDGE_BAUD           921600

#define GET_BAUDRATE               'b'
#define READ_ENCODERS              'e'
#define RESET_ENCODERS             'r'
#define MOTOR_SPEEDS               'm'

#define ROS_CMD_TIMEOUT_MS         500UL
#define ROS_BRIDGE_MAX_WHEEL_RAD_S 10.0f
#define ROS_BRIDGE_CMD_DEADBAND    0.02f

// ============================================================
// SECTION 3 – MAIN CONTROL LOOP
// ============================================================
#define LOOP_HZ          200    // Control loop frequency in Hz
#define SAFE_ANGLE_DEG   35.0f  // Tilt beyond this → safety stop
#define ALPHA            0.98f  // Complementary filter weight (gyro vs accel)

// ============================================================
// SECTION 4 – LQR GAIN SCHEDULING
// ============================================================
struct LqrGains {
  float k1;  // Wheel position gain
  float k2;  // Wheel velocity gain
  float k3;  // Body angle gain
  float k4;  // Body rate gain
};

static constexpr float HEIGHT_TALL_MM  = 450.0f;
static constexpr float HEIGHT_SHORT_MM = 280.0f;

static constexpr LqrGains GAINS_TALL = {
  -0.0f,    // k1 – wheel position
  -0.0f,     // k2 – wheel velocity
  -25.0f,    // k3 – body angle
  -2.350f      // k4 – body rate
};

static constexpr LqrGains GAINS_SHORT = {
   0.0f,    // k1
   0.0f,    // k2
   0.0f,    // k3
   0.0f     // k4
};

// ============================================================
// SECTION 5 – ROBOT / DRIVETRAIN
// ============================================================
#define GEAR_RATIO           8.0f
#define RIGHT_WHEEL_INVERT   true

#define USE_LINEAR_POSITION_UNITS false
static constexpr float WHEEL_RADIUS_M = 0.1f;

// ============================================================
// SECTION 6 – HARDWARE PINS
// ============================================================
#define SDA_PIN    21
#define SCL_PIN    22
#define CAN_RX_PIN  4
#define CAN_TX_PIN  5

// ============================================================
// SECTION 7 – IMU
// ============================================================
#define GYRO_ADDR       0x68

// ------------------------------------------------------------
// 7.1 – HEIGHT-SCHEDULED ANGLE OFFSET
// ------------------------------------------------------------
// The fused pitch angle needs a small bias (`offset`) added to it
// to represent true upright, because the IMU is not mounted exactly
// on the body's gravitational vertical axis.  As the robot changes
// height, the body CoM moves forward/back relative to the IMU, so
// the required offset changes with height.
//
// This table is a simple lookup/interpolation: sorted by height
// (ascending).  The applied offset is linearly interpolated between
// neighbouring entries.  Outside the range, the endpoint value is
// used (clamped).  Add as many entries as you like — minimum 2.
//
// Procedure to populate:
//   1. Command the robot to a fixed height (e.g. 280 mm).
//   2. Manually find the offset that makes it hover / drift least.
//   3. Record the (height, offset) pair below.
//   4. Repeat for 3–5 heights across your working range.
//
// Entries MUST be sorted by height_mm ascending.
// ------------------------------------------------------------
struct AngleOffsetPoint {
  float height_mm;
  float offset_deg;
};

static constexpr AngleOffsetPoint ANGLE_OFFSET_TABLE[] = {
  // { height_mm, offset_deg }
  { 280.0f, 4.7f },   // squat    — measure and tune
  { 360.0f, 4.5f },   // mid      — measure and tune
  { 435.0f, 4.2f },   // boot     — measure and tune
  { 450.0f, 4.2f },   // tall     — measure and tune
};

static constexpr int ANGLE_OFFSET_TABLE_SIZE =
    sizeof(ANGLE_OFFSET_TABLE) / sizeof(ANGLE_OFFSET_TABLE[0]);

// Linear-interpolation helper.  Call this from imu.cpp every cycle
// with the current filtered height; it returns the offset in degrees
// to add to the fused pitch angle.
static inline float angle_offset_for_height(float height_mm) {
  // Clamp to table endpoints.
  if (height_mm <= ANGLE_OFFSET_TABLE[0].height_mm) {
    return ANGLE_OFFSET_TABLE[0].offset_deg;
  }
  if (height_mm >= ANGLE_OFFSET_TABLE[ANGLE_OFFSET_TABLE_SIZE - 1].height_mm) {
    return ANGLE_OFFSET_TABLE[ANGLE_OFFSET_TABLE_SIZE - 1].offset_deg;
  }
  // Find the bracketing segment and lerp.
  for (int i = 0; i < ANGLE_OFFSET_TABLE_SIZE - 1; ++i) {
    float h0 = ANGLE_OFFSET_TABLE[i].height_mm;
    float h1 = ANGLE_OFFSET_TABLE[i + 1].height_mm;
    if (height_mm >= h0 && height_mm <= h1) {
      float t = (height_mm - h0) / (h1 - h0);
      float o0 = ANGLE_OFFSET_TABLE[i].offset_deg;
      float o1 = ANGLE_OFFSET_TABLE[i + 1].offset_deg;
      return o0 + t * (o1 - o0);
    }
  }
  // Should be unreachable.
  return ANGLE_OFFSET_TABLE[0].offset_deg;
}
