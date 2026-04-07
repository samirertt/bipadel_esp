#include "imu.h"
#include "config.h"
#include "utils.h"
#include <Wire.h>
#include <Adafruit_Sensor.h>
#include <Adafruit_ADXL345_U.h>

static Adafruit_ADXL345_Unified accel(12345);

static ImuData g_imu = {0};
static float g_gyro_offset_x = 0.0f;
static float g_robot_angle_deg = 0.0f;

static float read_gyro_x_dps() {
  Wire.beginTransmission(GYRO_ADDR);
  Wire.write(0x1D);
  Wire.endTransmission();

  Wire.requestFrom(GYRO_ADDR, 2);
  if (Wire.available() < 2) return 0.0f;

  int16_t rx = (Wire.read() << 8) | Wire.read();
  return (rx / 14.375f) - g_gyro_offset_x;
}

bool imu_init() {
  Wire.begin(SDA_PIN, SCL_PIN);
  Wire.setClock(400000);

  if (!accel.begin()) {
    return false;
  }

  accel.setRange(ADXL345_RANGE_4_G);

  // gyro init
  Wire.beginTransmission(GYRO_ADDR);
  Wire.write(0x3E);
  Wire.write(0x00);
  Wire.endTransmission();

  Wire.beginTransmission(GYRO_ADDR);
  Wire.write(0x16);
  Wire.write(0x1E);
  Wire.endTransmission();

  delay(100);

  // gyro calibration
  float sum = 0.0f;
  const int N = 500;
  for (int i = 0; i < N; i++) {
    Wire.beginTransmission(GYRO_ADDR);
    Wire.write(0x1D);
    Wire.endTransmission();

    Wire.requestFrom(GYRO_ADDR, 2);
    if (Wire.available() >= 2) {
      int16_t rx = (Wire.read() << 8) | Wire.read();
      sum += (rx / 14.375f);
    }
    delay(2);
  }
  g_gyro_offset_x = sum / N;
  g_robot_angle_deg = 0.0f;

  return true;
}

void imu_update(float dt) {
  sensors_event_t event;
  accel.getEvent(&event);

  float accel_angle_deg = atan2(event.acceleration.y, event.acceleration.z) * 180.0f / PI;
  //accel_angle_deg += ANGLE_OFFSET_DEG;

  float gyro_rate_dps = read_gyro_x_dps();

  g_robot_angle_deg = ALPHA * (g_robot_angle_deg + gyro_rate_dps * dt)
                    + (1.0f - ALPHA) * accel_angle_deg;

  g_imu.accel_angle_deg = roundf_int(accel_angle_deg);
  g_imu.gyro_rate_dps   = roundf_int(gyro_rate_dps);
  g_imu.fused_angle_deg = g_robot_angle_deg + ANGLE_OFFSET_DEG;
}

ImuData imu_get_data() {
  return g_imu;
}