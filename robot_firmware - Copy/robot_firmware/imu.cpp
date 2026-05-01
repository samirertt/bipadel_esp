#include "imu.h"
#include "config.h"
#include "utils.h"
#include <Wire.h>
#include <Adafruit_Sensor.h>
#include <Adafruit_ADXL345_U.h>
#include <Preferences.h>

static Preferences preferences;
static Adafruit_ADXL345_Unified accel(12345);

static ImuData g_imu = {0};
static float g_gyro_offset_x = 0.0f;
static float g_robot_angle_deg = 0.0f;

// --- Z-Axis Variables ---
static float g_gyro_offset_z = 0.0f;
static float g_robot_yaw_deg = 0.0f;

// --- Mechanical Offset Variable ---
static float g_mechanical_pitch_offset = 0.0f;

// --- current height for angle-offset scheduling ---
static float g_current_height_mm = ANGLE_OFFSET_TABLE[0].height_mm;

void imu_set_current_height_mm(float height_mm) {
  g_current_height_mm = height_mm;
}

static float read_gyro_x_dps() {
  Wire.beginTransmission(GYRO_ADDR);
  Wire.write(0x1D);
  Wire.endTransmission();
  Wire.requestFrom(GYRO_ADDR, 2);
  if (Wire.available() < 2) return 0.0f;
  int16_t rx = (Wire.read() << 8) | Wire.read();
  return (rx / 14.375f) - g_gyro_offset_x;
}

// --- Read Z-Axis from ITG-3200 ---
static float read_gyro_z_dps() {
  Wire.beginTransmission(GYRO_ADDR);
  Wire.write(0x21); // 0x21 is GYRO_ZOUT_H
  Wire.endTransmission();
  Wire.requestFrom(GYRO_ADDR, 2);
  if (Wire.available() < 2) return 0.0f;
  int16_t rz = (Wire.read() << 8) | Wire.read();
  return (rz / 14.375f) - g_gyro_offset_z;
}

bool imu_init() {
  Wire.begin(SDA_PIN, SCL_PIN);
  Wire.setClock(400000);

  if (!accel.begin()) return false;
  accel.setRange(ADXL345_RANGE_4_G);

  Wire.beginTransmission(GYRO_ADDR); Wire.write(0x3E); Wire.write(0x00); Wire.endTransmission();
  Wire.beginTransmission(GYRO_ADDR); Wire.write(0x16); Wire.write(0x1E); Wire.endTransmission();
  delay(100);

  // Dual-Axis Gyro Calibration
  float sum_x = 0.0f;
  float sum_z = 0.0f;
  const int N = 500;
  for (int i = 0; i < N; i++) {
    Wire.beginTransmission(GYRO_ADDR); Wire.write(0x1D); Wire.endTransmission();
    Wire.requestFrom(GYRO_ADDR, 2);
    if (Wire.available() >= 2) {
      int16_t rx = (Wire.read() << 8) | Wire.read();
      sum_x += (rx / 14.375f);
    }

    Wire.beginTransmission(GYRO_ADDR); Wire.write(0x21); Wire.endTransmission();
    Wire.requestFrom(GYRO_ADDR, 2);
    if (Wire.available() >= 2) {
      int16_t rz = (Wire.read() << 8) | Wire.read();
      sum_z += (rz / 14.375f);
    }
    delay(2);
  }
  g_gyro_offset_x = sum_x / N;
  g_gyro_offset_z = sum_z / N;

  // Load the saved mechanical offset from flash memory
  preferences.begin("robot_imu", false);
  g_mechanical_pitch_offset = preferences.getFloat("pitch_offset", 0.0f);
  preferences.end();

  // Prime the filter so it knows its real angle immediately
  sensors_event_t event;
  accel.getEvent(&event);
  g_robot_angle_deg = atan2(event.acceleration.y, event.acceleration.z) * 180.0f / PI;
  
  g_robot_yaw_deg = 0.0f;

  return true;
}

// Tare function to establish the true physical zero and save it
void imu_tare_pitch() {
  float sum = 0.0f;
  const int TARE_SAMPLES = 50;
  
  #if !ROS_BRIDGE_ENABLED
  Serial.println("\n--- IMU TARE SEQUENCE INITIATED ---");
  Serial.println("Keep the robot perfectly still at the physical balance point...");
  #endif

  for (int i = 0; i < TARE_SAMPLES; i++) {
    sensors_event_t event;
    accel.getEvent(&event);
    
    // Calculate the raw angle for this specific sample
    float current_angle = atan2(event.acceleration.y, event.acceleration.z) * 180.0f / PI;
    sum += current_angle;
    
    #if !ROS_BRIDGE_ENABLED
    // Print a quick progress update every 10 samples so we don't flood the Serial Monitor
    if (i % 10 == 0) {
      Serial.printf("Sample %d/%d: Raw Angle = %.2f deg\n", i, TARE_SAMPLES, current_angle);
    }
    #endif

    delay(2);
  }
  
  // Calculate new offset
  g_mechanical_pitch_offset = sum / TARE_SAMPLES;
  
  // Save the offset to flash memory so it persists through reboots
  preferences.begin("robot_imu", false);
  preferences.putFloat("pitch_offset", g_mechanical_pitch_offset);
  preferences.end();

  // Snap the filter to the new baseline immediately
  g_robot_angle_deg = g_mechanical_pitch_offset;
  //g_robot_angle_deg = -5.99f;

  #if !ROS_BRIDGE_ENABLED
  Serial.println("--- TARE COMPLETE ---");
  Serial.printf("New Mechanical Offset Baseline: %.2f deg\n", g_mechanical_pitch_offset);
  Serial.println("Successfully saved to ESP32 Flash Memory.\n");
  #endif
}

void imu_update(float dt) {
  sensors_event_t event;
  accel.getEvent(&event);

  float accel_angle_deg = atan2(event.acceleration.y, event.acceleration.z) * 180.0f / PI;
  float gyro_x_dps = read_gyro_x_dps();

  // Update Pitch Filter
  g_robot_angle_deg = ALPHA * (g_robot_angle_deg + gyro_x_dps * dt) + (1.0f - ALPHA) * accel_angle_deg;

  // Update Yaw Integration
  float gyro_z_dps = read_gyro_z_dps();
  if (abs(gyro_z_dps) < 0.5f) gyro_z_dps = 0.0f; // Deadband to kill micro-drift
  g_robot_yaw_deg += gyro_z_dps * dt;

  // Height-scheduled angle offset
  float offset_deg = angle_offset_for_height(g_current_height_mm);

  g_imu.accel_angle_deg = accel_angle_deg;
  g_imu.gyro_rate_dps   = gyro_x_dps;
  
  // Apply mechanical tare offset, then dynamic leg-height offset
  g_imu.fused_angle_deg = (g_robot_angle_deg - g_mechanical_pitch_offset) + offset_deg;

  g_imu.yaw_rate_dps  = gyro_z_dps;
  g_imu.yaw_angle_deg = g_robot_yaw_deg;
}

ImuData imu_get_data() {
  return g_imu;
}