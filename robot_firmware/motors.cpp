#include "motors.h"
#include "config.h"
#include "can_bus.h"
#include <Arduino.h>

bool motors_init() { return true; }

void motors_enable() {
  for (int id = 1; id <= 6; id++) {
    // Default for Wheels (1 = Torque Mode, 1 = Passthrough)
    uint8_t mode_data[8] = {1, 0, 0, 0, 1, 0, 0, 0}; 
    
    if (id >= 3) {
      // --- NEW: Legs use Position Mode (3) and Trapezoidal Trajectory (5) ---
      mode_data[0] = 3; 
      mode_data[4] = 1; 
    }
    
    uint8_t enable_data[8] = {8, 0, 0, 0, 0, 0, 0, 0}; 

    can_bus_transmit_frame(id, 0x0B, mode_data, 8);
    delay(20); 
    can_bus_transmit_frame(id, 0x07, enable_data, 8);
    delay(20); 
  }
}

void motors_stop() {
  can_bus_send_torque(1, 0.0f);
  can_bus_send_torque(2, 0.0f);
  uint8_t disable_data[8] = {1, 0, 0, 0, 0, 0, 0, 0};
  for (int id = 3; id <= 6; id++) can_bus_transmit_frame(id, 0x07, disable_data, 8);
}

void motors_set_wheel_torque(float left_wheel_nm, float right_wheel_nm) {
  // --- FIX: Invert the global drive direction ---
  left_wheel_nm = -left_wheel_nm;
  right_wheel_nm = -right_wheel_nm;
  
  float left_rotor_nm  = left_wheel_nm / GEAR_RATIO;
  float right_rotor_nm = right_wheel_nm / GEAR_RATIO;
  if (RIGHT_WHEEL_INVERT) right_rotor_nm = -right_rotor_nm;
  
  can_bus_send_torque(1, left_rotor_nm);
  can_bus_send_torque(2, right_rotor_nm);
}

void motors_set_leg_positions_deg(float l_knee_deg, float r_knee_deg, float l_torso_deg, float r_torso_deg) {
  // --- NEW: Convert degrees back to motor shaft revolutions before sending ---
  float l_knee_revs  = (l_knee_deg / 360.0f) * GEAR_RATIO;
  float r_knee_revs  = (r_knee_deg / 360.0f) * GEAR_RATIO;
  float l_torso_revs = (l_torso_deg / 360.0f) * GEAR_RATIO;
  float r_torso_revs = (r_torso_deg / 360.0f) * GEAR_RATIO;

  can_bus_send_position(3, l_knee_revs);
  can_bus_send_position(5, l_torso_revs);
  can_bus_send_position(4, r_knee_revs);
  can_bus_send_position(6, r_torso_revs);
}