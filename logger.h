#pragma once

#include <Arduino.h>
#include <WiFi.h>
#include "types.h"

void logger_init();
void logger_init(const char* ssid, const char* password, IPAddress target_ip, uint16_t target_port);

void logger_log(
    unsigned long t_ms,
    float dt,
    const ImuData& imu,
    const MotorFeedback& fb,
    const RobotState& state,
    const ControlOutput& out,
    bool safety_stop,
    int mode_value
);