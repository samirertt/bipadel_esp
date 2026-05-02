#include "wifi_logger.h"
#include <WiFi.h>
#include <WiFiUdp.h>

static WiFiUDP udp;
static const char* target_ip_addr;
static uint16_t target_udp_port;
static bool is_connected = false;

// We log at 200Hz to avoid choking the ESP32's main 500Hz control loop
static unsigned long last_log_us = 0;
static const unsigned long LOG_INTERVAL_US = 5000; // 5000 us = 200 Hz

bool wifi_logger_init(const char* ssid, const char* password, const char* target_ip, uint16_t target_port) {
    target_ip_addr = target_ip;
    target_udp_port = target_port;

    WiFi.mode(WIFI_STA);
    WiFi.begin(ssid, password);

    // Wait up to 5 seconds for connection to avoid hanging forever
    int attempts = 0;
    while (WiFi.status() != WL_CONNECTED && attempts < 20) {
        delay(250);
        attempts++;
    }

    if (WiFi.status() == WL_CONNECTED) {
        is_connected = true;
        return true;
    }
    
    return false; // Boot without WiFi if not available
}

void wifi_logger_update(float &fwd, float &turn, bool &valid) {
    
    if (!is_connected) return;

    // Non-blocking rate limiter (200Hz)
    unsigned long now_us = micros();
    if (now_us - last_log_us < LOG_INTERVAL_US) return;
    last_log_us = now_us;

    char buffer[256];
    
    // Formats strictly to your Python CSV header:
    // t_ms, dt, accel_angle_deg, gyro_rate_dps, fused_angle_deg, left_pos_deg, left_vel_dps, 
    // right_pos_deg, right_vel_dps, angle_rad, rate_rad, x_pos, x_vel, balance_torque_nm, 
    // forward_torque_nm, left_cmd_torque_nm, right_cmd_torque_nm, safety_stop, cmd_currently
    
    int len = snprintf(buffer, sizeof(buffer),
        "%lu,%.4f,%.2f,%.2f,%d",
        millis(), 
        fwd,
        turn,
        valid
    );

    udp.beginPacket(target_ip_addr, target_udp_port);
    udp.write((const uint8_t*)buffer, len);
    udp.endPacket();
}