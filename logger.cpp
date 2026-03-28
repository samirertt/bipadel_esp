#include "logger.h"
#include "config.h"
#include "balance_controller.h"

#include <Arduino.h>
#include <WiFi.h>
#include <WiFiUdp.h>

static WiFiUDP udp;
static IPAddress pc_ip;
static uint16_t pc_port = 0;
static bool g_udp_enabled = false;

static void print_header_to_serial() {
  Serial.println(
    "t_ms,dt,"
    "mode,controller_type,safety_stop,"
    "accel_angle_deg,gyro_rate_dps,fused_angle_deg,"
    "left_pos_deg,left_vel_dps,right_pos_deg,right_vel_dps,"
    "angle_rad,rate_rad,x_pos,x_vel,"
    "balance_torque,forward_torque,left_cmd_torque,right_cmd_torque"
  );
}

void logger_init() {
  g_udp_enabled = false;
  print_header_to_serial();
  Serial.println("# logger_ready_serial");
}

void logger_init(const char* ssid, const char* password, IPAddress target_ip, uint16_t target_port) {
  pc_ip = target_ip;
  pc_port = target_port;
  g_udp_enabled = false;

  Serial.print("# Connecting to WiFi: ");
  Serial.println(ssid);

  WiFi.mode(WIFI_STA);
  WiFi.begin(ssid, password);

  while (WiFi.status() != WL_CONNECTED) {
    delay(500);
    Serial.print(".");
  }

  Serial.println();
  Serial.println("# WiFi connected!");
  Serial.print("# Robot IP address: ");
  Serial.println(WiFi.localIP());

  udp.begin(pc_port);
  g_udp_enabled = true;

  Serial.println("# UDP logger_ready");
  print_header_to_serial();
}

void logger_log(
    unsigned long t_ms,
    float dt,
    const ImuData& imu,
    const MotorFeedback& fb,
    const RobotState& state,
    const ControlOutput& out,
    bool safety_stop,
    int mode_value
) {
  int controller_type = (int)balance_controller_get_type();

  // -------- Serial output --------
  Serial.print(t_ms);                        Serial.print(',');
  Serial.print(dt, 6);                       Serial.print(',');

  Serial.print(mode_value);                  Serial.print(',');
  Serial.print(controller_type);             Serial.print(',');
  Serial.print(safety_stop ? 1 : 0);         Serial.print(',');

  Serial.print(imu.accel_angle_deg, 6);      Serial.print(',');
  Serial.print(imu.gyro_rate_dps, 6);        Serial.print(',');
  Serial.print(imu.fused_angle_deg, 6);      Serial.print(',');

  Serial.print(fb.left_pos_deg, 6);          Serial.print(',');
  Serial.print(fb.left_vel_dps, 6);          Serial.print(',');
  Serial.print(fb.right_pos_deg, 6);         Serial.print(',');
  Serial.print(fb.right_vel_dps, 6);         Serial.print(',');

  Serial.print(state.angle_rad, 6);          Serial.print(',');
  Serial.print(state.rate_rad, 6);           Serial.print(',');
  Serial.print(state.x_pos, 6);              Serial.print(',');
  Serial.print(state.x_vel, 6);              Serial.print(',');

  Serial.print(out.balance_torque, 6);       Serial.print(',');
  Serial.print(out.forward_torque, 6);       Serial.print(',');
  Serial.print(out.wheels.left_torque, 6);   Serial.print(',');
  Serial.println(out.wheels.right_torque, 6);

  // -------- UDP output --------
  if (g_udp_enabled) {
    udp.beginPacket(pc_ip, pc_port);

    udp.print(t_ms);                         udp.print(',');
    udp.print(dt, 6);                        udp.print(',');

    udp.print(mode_value);                   udp.print(',');
    udp.print(controller_type);              udp.print(',');
    udp.print(safety_stop ? 1 : 0);          udp.print(',');

    udp.print(imu.accel_angle_deg, 6);       udp.print(',');
    udp.print(imu.gyro_rate_dps, 6);         udp.print(',');
    udp.print(imu.fused_angle_deg, 6);       udp.print(',');

    udp.print(fb.left_pos_deg, 6);           udp.print(',');
    udp.print(fb.left_vel_dps, 6);           udp.print(',');
    udp.print(fb.right_pos_deg, 6);          udp.print(',');
    udp.print(fb.right_vel_dps, 6);          udp.print(',');

    udp.print(state.angle_rad, 6);           udp.print(',');
    udp.print(state.rate_rad, 6);            udp.print(',');
    udp.print(state.x_pos, 6);               udp.print(',');
    udp.print(state.x_vel, 6);               udp.print(',');

    udp.print(out.balance_torque, 6);        udp.print(',');
    udp.print(out.forward_torque, 6);        udp.print(',');
    udp.print(out.wheels.left_torque, 6);    udp.print(',');
    udp.print(out.wheels.right_torque, 6);

    udp.endPacket();
  }
}