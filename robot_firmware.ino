#include "config.h"
#include "utils.h"
#include "imu.h"
#include "can_bus.h"
#include "motors.h"
#include "estimator.h"
#include "lqr_controller.h"
#include "control_manager.h"
#include "safety.h"
#include "logger.h"
#include <PS4Controller.h> // NEW: PS4 Library

static unsigned long last_loop_ms = 0;
static const unsigned long loop_interval_ms = 1000 / LOOP_HZ;

void setup() {
  Serial.begin(115200);
  delay(2000);
  
  Serial.println("\n==================================");
  Serial.println("   ROBOT BOOT SEQUENCE STARTING   ");
  Serial.println("==================================");

  // --- NEW: Initialize PS4 Controller ---
  Serial.print("[0/3] Initializing PS4 Bluetooth... ");
  if (PS4.begin("C4:5B:BE:91:C3:06")) { // Using your real MAC!
    Serial.println("OK");
  } else {
    Serial.println("FAILED!");
  }

  Serial.print("[1/3] Initializing IMU... ");
  bool imu_ok = imu_init();
  Serial.println(imu_ok ? "OK" : "FAILED! Check I2C wiring (SDA 21, SCL 22)");

  Serial.print("[2/3] Initializing CAN Bus... ");
  bool can_ok = can_bus_init();
  Serial.println(can_ok ? "OK" : "FAILED! Check TWAI pins (RX 4, TX 5)");

  Serial.print("[3/3] Initializing Motors... ");
  bool motors_ok = motors_init();
  Serial.println(motors_ok ? "OK" : "FAILED!");

  estimator_init();
  lqr_controller_init();
  control_manager_init();
  
  Serial.println("\n--- Starting Logger (WiFi) ---");
  logger_init();

  if (!imu_ok || !can_ok || !motors_ok) {
    Serial.println("\n[CRITICAL ERROR] Hardware check failed. Halting system.");
    while (true) {
      delay(1000); 
    }
  }

  Serial.println("\nHardware Check Passed. Enabling Motors (Closed Loop)...");
  motors_enable();

  last_loop_ms = millis();
  Serial.println("==================================");
  Serial.println("   ENTERING MAIN CONTROL LOOP     ");
  Serial.println("==================================");
}

void loop() {
  can_bus_poll();
  
  unsigned long now = millis();
  if ((now - last_loop_ms) < loop_interval_ms) {
    return;
  }

  float dt = (now - last_loop_ms) / 1000.0f;
  last_loop_ms = now;

  imu_update(dt);
  ImuData imu = imu_get_data();
  MotorFeedback fb = can_bus_get_feedback();

  estimator_update(dt, imu, fb);
  RobotState state = estimator_get_state();

  bool safety_stop = !safety_angle_ok_deg(rad2deg(state.angle_rad));

  // --- NEW: Read PS4 Controller Inputs ---
  float forward_cmd = 0.0f;
  float turn_cmd = 0.0f;

  if (PS4.isConnected()) {
    int ly = PS4.LStickY(); // Forward/Back
    int rx = PS4.RStickX(); // Turning

    // Deadbands (ignores minor stick drift near center)
    if (abs(ly) > 15) forward_cmd = ly / 127.0f; 
    if (abs(rx) > 15) turn_cmd = rx / 127.0f;    
  }
  
  ControlOutput out = {};

  if (safety_stop) {
    out.wheels.left_torque = 0.0f;
    out.wheels.right_torque = 0.0f;
    motors_stop();
  } else {
    // Pass the commands and dt into the control manager
    out = control_manager_update(dt, state, forward_cmd, turn_cmd);
    motors_set_wheel_torque(out.wheels.left_torque, out.wheels.right_torque);
  }

  logger_log(now, dt, imu, fb, state, out, safety_stop, 0);

  static unsigned long last_print = 0;
  if (now - last_print > 1000) {
    if (safety_stop) {
      Serial.printf("SAFETY STOP: Ang %.1f deg (Limit is +/- 20)\n", rad2deg(state.angle_rad));
    } else {
      Serial.printf("BALANCING | Ang: %5.1f deg | Cmd Torque L: %5.2f Nm, R: %5.2f Nm | CAN Pos L: %5.1f, R: %5.1f\n",
                    rad2deg(state.angle_rad),
                    out.wheels.left_torque,
                    out.wheels.right_torque,
                    fb.left_pos_deg,
                    fb.right_pos_deg);
    }
    last_print = now;
  }
}