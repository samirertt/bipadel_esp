#include "logger.h"
#include "config.h"

#include <Arduino.h>
#include <WiFi.h>
#include <WiFiUdp.h>

const char* ssid = "Dronos";
const char* password = "Dr2025Kh";
const char* target_ip_str = "10.1.36.169"; 
const uint16_t target_port = 12345;          

static IPAddress pc_ip;
static WiFiUDP udp;
static bool udp_ready = false;

void logger_init() {
  Serial.println("# Initializing Logger...");

  pc_ip.fromString(target_ip_str);

  Serial.print("# Connecting to WiFi: ");
  Serial.println(ssid);

  WiFi.mode(WIFI_STA);
  WiFi.begin(ssid, password);

  unsigned long start_time = millis();
  const unsigned long timeout_ms = 5000; 

  while (WiFi.status() != WL_CONNECTED) {
    if (millis() - start_time > timeout_ms) {
      Serial.println("\n# WiFi Timeout! Starting robot in OFFLINE mode.");
      udp_ready = false;
      return; 
    }
    delay(500);
    Serial.print(".");
  }

  Serial.println("\n# WiFi connected!");
  Serial.print("# Robot IP address: ");
  Serial.println(WiFi.localIP());

  udp.begin(target_port);
  udp_ready = true;
  Serial.println("# UDP logger_ready");
}

void logger_log(
    unsigned long t_ms,
    float dt,
    const ImuData& imu,
    const MotorFeedback& fb,
    const RobotState& state,
    const ControlOutput& out,
    bool safety_stop,
    int cmd_currently
) {
  if (!udp_ready) {
    return; 
  }

  udp.beginPacket(pc_ip, target_port);

  udp.print(t_ms); udp.print(',');
  udp.print(dt, 6); udp.print(',');

  udp.print(imu.accel_angle_deg, 6); udp.print(',');
  udp.print(imu.gyro_rate_dps, 6); udp.print(',');
  udp.print(imu.fused_angle_deg, 6); udp.print(',');

  udp.print(fb.left_pos_deg, 6); udp.print(',');
  udp.print(fb.left_vel_dps, 6); udp.print(',');
  udp.print(fb.right_pos_deg, 6); udp.print(',');
  udp.print(fb.right_vel_dps, 6); udp.print(',');

  udp.print(state.angle_rad, 6); udp.print(',');
  udp.print(state.rate_rad, 6); udp.print(',');
  udp.print(state.x_pos, 6); udp.print(',');
  udp.print(state.x_vel, 6); udp.print(',');

  udp.print(out.balance_torque, 6); udp.print(',');
  udp.print(out.wheels.left_torque, 6); udp.print(',');
  udp.print(out.wheels.right_torque, 6); udp.print(',');

  udp.print(safety_stop ? 1 : 0); udp.print(',');
  udp.print(cmd_currently);

  udp.endPacket();
}