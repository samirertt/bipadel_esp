#include "motors.h"
#include "config.h"
#include "can_bus.h"

#include <Arduino.h>

bool motors_init() {
  return true;
}

void motors_enable() {
  Serial.println("Sending CAN Commands to Enable Motors...");
  for (int id : {1, 2}) {
    // Mode 0x0B: Control Mode 1 (Torque), Input Mode 1 (Direct)
    uint8_t mode_data[8]   = {1, 0, 0, 0, 1, 0, 0, 0};
    // State 0x07: Axis State 8 (Closed Loop)
    uint8_t enable_data[8] = {8, 0, 0, 0, 0, 0, 0, 0}; 

    can_bus_transmit_frame(id, 0x0B, mode_data, 8);
    delay(50); // Give the motor driver time to process
    can_bus_transmit_frame(id, 0x07, enable_data, 8);
    delay(50); 
  }
}

void motors_stop() {
  // Send 0.0 Nm of torque to safely stop
  can_bus_send_torque(2, 0.0f);
  can_bus_send_torque(1, 0.0f);
}

void motors_set_wheel_torque(float left_wheel_nm, float right_wheel_nm) {
  // Convert wheel torque to rotor torque using the gear ratio
  float left_rotor_nm  = left_wheel_nm / GEAR_RATIO;
  float right_rotor_nm = right_wheel_nm / GEAR_RATIO;

  // Invert the right wheel direction so moving "forward" spins the wheels correctly
  if (RIGHT_WHEEL_INVERT) {
    right_rotor_nm = -right_rotor_nm;
  }

  // Send over CAN
  can_bus_send_torque(1, left_rotor_nm);
  can_bus_send_torque(2, right_rotor_nm);
}