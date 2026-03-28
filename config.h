#pragma once

#include <Arduino.h>
#include "driver/twai.h"

// ===============================
// CONTROL
// ===============================
#define LOOP_HZ 200
#define SAFE_ANGLE_DEG 15.0f
#define ALPHA 0.95f

#define MAX_WHEEL_TORQUE_NM 0.8f

// LQR gains (use later, not first)
static constexpr float K1 = 0.0f;
static constexpr float K2 = 0.8f;
static constexpr float K3 = 14.0f;
static constexpr float K4 = 5.5f;

// ===============================
// ROBOT / MOTOR
// ===============================
#define GEAR_RATIO 8.0f
#define RIGHT_WHEEL_INVERT true

#define USE_LINEAR_POSITION_UNITS false
static constexpr float WHEEL_RADIUS_M = 0.1f;

// extra scaling for motor torque command
// if still aggressive, reduce to 0.2f
static constexpr float MOTOR_CMD_SCALE = 5.0f;

// ===============================
// PINS
// ===============================
#define SDA_PIN 21
#define SCL_PIN 22

#define CAN_RX_PIN 4
#define CAN_TX_PIN 5

#define MCP_CS_PIN 27
#define MCP_INT_PIN 33

// ===============================
// IMU
// ===============================
#define GYRO_ADDR 0x68
static constexpr float ANGLE_OFFSET_DEG = 0.0f;

// ===============================
// CAN
// ===============================
#define BAUD_RATE_NATIVE TWAI_TIMING_CONFIG_500KBITS()
#define BAUD_RATE_MCP CAN_500KBPS