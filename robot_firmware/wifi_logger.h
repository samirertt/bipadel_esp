#pragma once

#include <Arduino.h>
#include "types.h"

// Initialize WiFi and prepare UDP socket
bool wifi_logger_init(const char* ssid, const char* password, const char* target_ip, uint16_t target_port);

// Pack and send the telemetry frame (rate-limited internally to ~200Hz)
void wifi_logger_update(float dt, const ImuData& imu, const MotorFeedback& fb, 
                        const RobotState& state, const ControlOutput& out, 
                        float forward_cmd, bool safety_stop);