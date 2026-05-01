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


#define ROS_CMD_TIMEOUT_MS         500UL
#define ROS_BRIDGE_MAX_WHEEL_RAD_S 10.0f
#define ROS_BRIDGE_CMD_DEADBAND    0.02f

// ============================================================
// SECTION 3 – MAIN CONTROL LOOP
// ============================================================
#define LOOP_HZ          500    // Control loop frequency in Hz
#define SAFE_ANGLE_DEG   35.0f  // Tilt beyond this → safety stop
#define ALPHA            0.982f  // Complementary filter weight (gyro vs accel)

// ============================================================
// SECTION 4 – LQR GAIN SCHEDULING
// ============================================================
struct LqrGains {
  float k1;  // Wheel position gain
  float k2;  // Wheel velocity gain
  float k3;  // Body angle gain
  float k4;  // Body rate gain
};

static constexpr float HEIGHT_TALL_MM  = 490.0f;
static constexpr float HEIGHT_SHORT_MM = 330.0f;

// K3, k4 should be retuned.
static constexpr LqrGains GAINS_TALL = {
  -0.0f,    // k1 – wheel position
  -0.60f,    // k2 – wheel velocity 
  -22.2f,  // k3 – body angle    
  -3.4f    // k4 – body rate      
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
struct AngleOffsetPoint {
  float height_mm;
  float offset_deg;
};

// NOTE: Reset to 0.0f since the IMU tare function now handles the mechanical baseline.
// Tune these slightly to correct for leg-extension CoG shifts.
static constexpr AngleOffsetPoint ANGLE_OFFSET_TABLE[] = {
  { 330.0f, 0.0f },
  { 380.0f, 0.0f },
  { 450.0f, 0.0f },
  { 488.0f, 0.0f },
  { 490.0f, -0.917f },
};

static constexpr int ANGLE_OFFSET_TABLE_SIZE =
    sizeof(ANGLE_OFFSET_TABLE) / sizeof(ANGLE_OFFSET_TABLE[0]);

static inline float angle_offset_for_height(float height_mm) {
  if (height_mm <= ANGLE_OFFSET_TABLE[0].height_mm) {
    return ANGLE_OFFSET_TABLE[0].offset_deg;
  }
  if (height_mm >= ANGLE_OFFSET_TABLE[ANGLE_OFFSET_TABLE_SIZE - 1].height_mm) {
    return ANGLE_OFFSET_TABLE[ANGLE_OFFSET_TABLE_SIZE - 1].offset_deg;
  }
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
  return ANGLE_OFFSET_TABLE[0].offset_deg;
}

// ============================================================
// SECTION 8 – WIFI / TELEMETRY
// ============================================================
#define WIFI_SSID        "SPECIAL"
#define WIFI_PASSWORD    "AMIGO123"
#define TARGET_PC_IP     "192.168.154.91"
#define UDP_PORT         12345