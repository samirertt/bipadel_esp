#include "imu.h"
#include "config.h"
#include "utils.h"
#include <Wire.h>
#include <Adafruit_Sensor.h>
#include <Adafruit_ADXL345_U.h>

static Adafruit_ADXL345_Unified accel(12345);

static ImuData g_imu = {0};

// --- Gyro offsets ---
static float g_gyro_offset_x = 0.0f;
static float g_gyro_offset_y = 0.0f;
static float g_gyro_offset_z = 0.0f;

// --- State ---
static float g_robot_angle_deg = 0.0f;
static float g_robot_yaw_deg   = 0.0f;

// --- Height scheduling ---
static float g_current_height_mm = ANGLE_OFFSET_TABLE[0].height_mm;

void imu_set_current_height_mm(float height_mm) {
  g_current_height_mm = height_mm;
}

// =======================
// Gyro Read Functions
// =======================

static float read_gyro_axis(uint8_t reg_h, float offset) {
  Wire.beginTransmission(GYRO_ADDR);
  Wire.write(reg_h);
  Wire.endTransmission();
  Wire.requestFrom(GYRO_ADDR, 2);

  if (Wire.available() < 2) return 0.0f;

  int16_t raw = (Wire.read() << 8) | Wire.read();
  return (raw / 14.375f) - offset;
}

static float read_gyro_x_dps() { return read_gyro_axis(0x1D, g_gyro_offset_x); }
static float read_gyro_y_dps() { return read_gyro_axis(0x1F, g_gyro_offset_y); }
static float read_gyro_z_dps() { return read_gyro_axis(0x21, g_gyro_offset_z); }

// =======================
// Init
// =======================

bool imu_init() {
  Wire.begin(SDA_PIN, SCL_PIN);
  Wire.setClock(400000);

  if (!accel.begin()) return false;
  accel.setRange(ADXL345_RANGE_4_G);

  // ITG-3200 setup
  Wire.beginTransmission(GYRO_ADDR); Wire.write(0x3E); Wire.write(0x00); Wire.endTransmission(); // power
  Wire.beginTransmission(GYRO_ADDR); Wire.write(0x16); Wire.write(0x1E); Wire.endTransmission(); // config
  delay(100);

  // --- 3-axis calibration ---
  float sum_x = 0.0f, sum_y = 0.0f, sum_z = 0.0f;
  const int N = 500;

  for (int i = 0; i < N; i++) {

    // X
    Wire.beginTransmission(GYRO_ADDR); Wire.write(0x1D); Wire.endTransmission();
    Wire.requestFrom(GYRO_ADDR, 2);
    if (Wire.available() >= 2) {
      int16_t rx = (Wire.read() << 8) | Wire.read();
      sum_x += (rx / 14.375f);
    }

    // Y
    Wire.beginTransmission(GYRO_ADDR); Wire.write(0x1F); Wire.endTransmission();
    Wire.requestFrom(GYRO_ADDR, 2);
    if (Wire.available() >= 2) {
      int16_t ry = (Wire.read() << 8) | Wire.read();
      sum_y += (ry / 14.375f);
    }

    // Z
    Wire.beginTransmission(GYRO_ADDR); Wire.write(0x21); Wire.endTransmission();
    Wire.requestFrom(GYRO_ADDR, 2);
    if (Wire.available() >= 2) {
      int16_t rz = (Wire.read() << 8) | Wire.read();
      sum_z += (rz / 14.375f);
    }

    delay(2);
  }

  g_gyro_offset_x = sum_x / N;
  g_gyro_offset_y = sum_y / N;
  g_gyro_offset_z = sum_z / N;

  g_robot_angle_deg = 0.0f;
  g_robot_yaw_deg   = 0.0f;

  return true;
}

// =======================
// Update
// =======================

void imu_update(float dt) {

  // --- Accelerometer ---
  sensors_event_t event;
  accel.getEvent(&event);

  float accel_x = event.acceleration.x;
  float accel_y = event.acceleration.y;
  float accel_z = event.acceleration.z;

  // --- Gyro ---
  float gyro_x = read_gyro_x_dps();
  float gyro_y = read_gyro_y_dps();
  float gyro_z = read_gyro_z_dps();

  // --- Pitch (complementary filter) ---
  float accel_angle_deg = atan2(accel_y, accel_z) * 180.0f / PI;

  g_robot_angle_deg =
      ALPHA * (g_robot_angle_deg + gyro_x * dt) +
      (1.0f - ALPHA) * accel_angle_deg;

  // --- Yaw (integration) ---
  if (abs(gyro_z) < 0.5f) gyro_z = 0.0f;
  g_robot_yaw_deg += gyro_z * dt;

  // --- Offset ---
  float offset_deg = angle_offset_for_height(g_current_height_mm);

  // =======================
  // Store EVERYTHING
  // =======================

  // Raw accel
  g_imu.accel_x = accel_x;
  g_imu.accel_y = accel_y;
  g_imu.accel_z = accel_z;

  // Raw gyro
  g_imu.gyro_x = gyro_x;
  g_imu.gyro_y = g_robot_angle_deg;
  g_imu.gyro_z = gyro_z;

  // Processed
  g_imu.accel_angle_deg = accel_angle_deg;
  g_imu.gyro_rate_dps   = gyro_x;
  g_imu.fused_angle_deg = g_robot_angle_deg + offset_deg;

  g_imu.yaw_rate_dps  = gyro_z;
  g_imu.yaw_angle_deg = g_robot_yaw_deg;
}

// =======================
// Getter
// =======================

ImuData imu_get_data() {
  return g_imu;
}