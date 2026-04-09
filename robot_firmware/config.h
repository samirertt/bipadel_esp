#pragma once

#include <Arduino.h>
#include "driver/twai.h"

// ===============================
// CONTROL
// ===============================
#define LOOP_HZ 500
#define SAFE_ANGLE_DEG 20.0f
#define ALPHA 0.98f


// LQR gains
static constexpr float K1 = 2.40f;     // wheel position
static constexpr float K2 = 12.50f;     // wheel velocity    
static constexpr float K3 = -25.0f;    // body angle
static constexpr float K4 = -8.50f;    // body angular velocity   

// Output limits
//static constexpr float MAX_WHEEL_CMD_DPS = 1440.0f;

// ===============================
// ROBOT / MOTOR
// ===============================
#define GEAR_RATIO 8.0f
#define RIGHT_WHEEL_INVERT true

// If your LQR was designed in wheel radians, keep false.
// If it was designed in meters, set true and provide wheel radius.
#define USE_LINEAR_POSITION_UNITS false
static constexpr float WHEEL_RADIUS_M = 0.1f;  // adjust if needed

// ===============================
// PINS
// ===============================
#define SDA_PIN 21
#define SCL_PIN 22

#define CAN_RX_PIN 4
#define CAN_TX_PIN 5

// ===============================
// IMU SETTINGS
// ===============================
#define GYRO_ADDR 0x68
#define ANGLE_OFFSET_DEG -2.335f 