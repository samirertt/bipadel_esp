#include "motors.h"
#include "config.h"
#include "can_bus.h"

#include <Arduino.h>

bool motors_init() {
  return true;
}

void motors_enable() {
  for (int id : {1, 2}) {
    // control_mode = 1 (torque), input_mode = 1 (direct)
    uint8_t mode_data[8]   = {1, 0, 0, 0, 1, 0, 0, 0};
    uint8_t enable_data[8] = {8, 0, 0, 0, 0, 0, 0, 0}; // closed loop

    can_bus_transmit_frame(id, 0x0B, mode_data, 8);
    delay(10);
    can_bus_transmit_frame(id, 0x07, enable_data, 8);
    delay(10);
  }
}

void motors_stop() {
  can_bus_send_torque(1, 0.0f);
  can_bus_send_torque(2, 0.0f);
}

void motors_set_wheel_torque(float left_wheel_nm, float right_wheel_nm) {
  float left_rotor_nm  = left_wheel_nm / GEAR_RATIO;
  float right_rotor_nm = right_wheel_nm / GEAR_RATIO;

  float motor1_cmd = -left_rotor_nm * MOTOR_CMD_SCALE;
  float motor2_cmd = (RIGHT_WHEEL_INVERT ? right_rotor_nm : -right_rotor_nm) * MOTOR_CMD_SCALE;

  can_bus_send_torque(1, motor1_cmd);
  can_bus_send_torque(2, motor2_cmd);
}