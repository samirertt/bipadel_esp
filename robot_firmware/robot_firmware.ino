// ============================================================
// robot_firmware_integrated.ino
// ============================================================
// Main entry point for the self-balancing robot firmware.
//
// ARCHITECTURE OVERVIEW
// ─────────────────────
// Each feature lives in its own .cpp/.h pair.  This file only
// orchestrates the boot sequence and the per-loop data flow:
//
//   config.h            – all compile-time constants and flags
//   types.h             – shared data structures
//   utils.h             – inline math helpers
//   imu.cpp             – ADXL345 + ITG-3200 sensor fusion
//   can_bus.cpp         – ESP32 TWAI / ODrive CAN Simple driver
//   motors.cpp          – motor mode setup & typed command wrappers
//   estimator.cpp       – state estimator (position, velocity, angle)
//   lqr_controller.cpp  – LQR balance torque computation
//   control_manager.cpp – velocity outer-loop + yaw + standby logic
//   kinematics.cpp      – 2-DOF leg inverse kinematics
//   safety.cpp          – tilt and motor-fault safety checks
//   ros_bridge.cpp      – ROS 2 serial hardware-interface bridge  ← NEW
//
// INPUT SOURCE SELECTION  (edit config.h, not this file)
// ──────────────────────
//   ROS_BRIDGE_TEST_MODE = 1  →  commands come from ROS 2
//   ROS_BRIDGE_TEST_MODE = 0  →  commands come from PS4 controller
//
// The control stack (estimator → LQR → control_manager) is identical
// for both sources; only the command acquisition block changes.
// ============================================================

#include "config.h"
#include "types.h"
#include "utils.h"

// -- Sensing & actuation modules --
#include "imu.h"
#include "can_bus.h"
#include "motors.h"

// -- State estimation & control --
#include "estimator.h"
#include "lqr_controller.h"
#include "control_manager.h"

// -- Leg kinematics --
#include "kinematics.h"

// -- Safety, logging --
#include "safety.h"

// -- ROS 2 bridge (conditionally compiled) --
#include "ros_bridge.h"

// -- WiFi Logger --
#include "wifi_logger.h"

// -- PS4 controller --
// PS4 is compiled in so it can coexist with the ROS bridge.
#include <PS4Controller.h>

// ============================================================
// Timing
// ============================================================
// Timing in us
static unsigned long last_loop_us = 0;
static const unsigned long LOOP_INTERVAL_US = 1000000UL / LOOP_HZ;  // 2000 us at 500 Hz

// ============================================================
// Boot posture & height trajectory
// ============================================================
// The robot boots at a known physical stand height.  Target height
// is modified at runtime by the PS4 D-pad (or future ROS service).
static const float BOOT_HEIGHT_MM  = 435.75f;
static float target_height_mm      = BOOT_HEIGHT_MM;
static float current_height_mm     = BOOT_HEIGHT_MM;  // Low-pass filtered

// ============================================================
// IK reference offsets (calibrated at boot)
// ============================================================
// All leg position commands are *relative* to the boot encoder
// readings, so absolute encoder position is not required.
static float knee_offset_deg  = 0.0f;
static float torso_offset_deg = 0.0f;

// Boot-time encoder snapshots (absolute motor positions in degrees)
static float boot_pos_l_knee_deg  = 0.0f;
static float boot_pos_r_knee_deg  = 0.0f;
static float boot_pos_l_torso_deg = 0.0f;
static float boot_pos_r_torso_deg = 0.0f;

// ============================================================
// PS4 state
// ============================================================
static bool standby_mode       = true;   // Robot wakes up in standby
static bool has_taken_off      = false;  // Tracks first liftoff
static bool last_share_pressed = false;

// ============================================================
// setup()
// ============================================================
void setup() {
  Serial.begin(921600);
  delay(2000);

  // Suppress the ASCII banner when ROS is on the other end of the
  // serial port, so the ROS driver does not have to filter it.
#if !ROS_BRIDGE_ENABLED
  Serial.println("\n==================================");
  Serial.println("   ROBOT BOOT SEQUENCE STARTING   ");
  Serial.println("==================================");
#endif

  // ----------------------------------------------------------
  // [0/3] Bluetooth PS4 Controller
  // ----------------------------------------------------------
#if !ROS_BRIDGE_ENABLED
  Serial.print("[0/3] Initializing PS4 Bluetooth... ");
#endif
  bool ps4_ok = PS4.begin("C4:5B:BE:91:C3:06");
#if !ROS_BRIDGE_ENABLED
  Serial.println(ps4_ok ? "OK" : "FAILED!");
#endif

  // ----------------------------------------------------------
  // [1/3] IMU
  // ----------------------------------------------------------
#if !ROS_BRIDGE_ENABLED
  Serial.print("[1/3] Initializing IMU... ");
#endif
  bool imu_ok = imu_init();
#if !ROS_BRIDGE_ENABLED
  Serial.println(imu_ok ? "OK" : "FAILED! Check I2C wiring");
#endif

  // ----------------------------------------------------------
  // [2/3] CAN Bus
  // ----------------------------------------------------------
#if !ROS_BRIDGE_ENABLED
  Serial.print("[2/3] Initializing CAN Bus... ");
#endif
  bool can_ok = can_bus_init();
#if !ROS_BRIDGE_ENABLED
  Serial.println(can_ok ? "OK" : "FAILED! Check TWAI pins");
#endif

  if (!can_ok) {
#if !ROS_BRIDGE_ENABLED
    Serial.println("\n[CRITICAL ERROR] CAN Hardware failed. Halting system.");
#endif
    while (true) delay(1000);
  }

  // ----------------------------------------------------------
  // WiFi Telemetry
  // ----------------------------------------------------------
#if !ROS_BRIDGE_ENABLED
  Serial.print("Connecting to WiFi for Telemetry... ");
  bool wifi_ok = wifi_logger_init(WIFI_SSID, WIFI_PASSWORD, TARGET_PC_IP, UDP_PORT);
  Serial.println(wifi_ok ? "OK" : "FAILED! (Running without WiFi)");
#endif

  // ----------------------------------------------------------
  // ROS 2 Bridge initialisation
  // ----------------------------------------------------------
#if ROS_BRIDGE_ENABLED
  ros_bridge_init(ROS_BRIDGE_BAUD);
  ros_bridge_set_timeout_ms(ROS_CMD_TIMEOUT_MS);
#endif

  // ----------------------------------------------------------
  // Wait for all 6 motor heartbeats
  // ----------------------------------------------------------
#if !ROS_BRIDGE_DEMO_FEEDBACK
  #if !ROS_BRIDGE_ENABLED
    Serial.print("Waiting for all 6 motors to heartbeat... ");
  #endif

  while (true) {
    can_bus_poll();
    MotorFeedback init_fb = can_bus_get_feedback();
    uint32_t now = millis();
    int online_count = 0;

    for (int i = 1; i <= 6; i++) {
      if (init_fb.last_msg_time_ms[i] > 0 &&
          (now - init_fb.last_msg_time_ms[i]) < 500) {
        online_count++;
      }
    }

    if (online_count == 6) break;
    delay(10);
  }

  #if !ROS_BRIDGE_ENABLED
    Serial.println("OK! All 6 Motors Verified.");
  #endif
#endif

  // ----------------------------------------------------------
  // Wake motors in 0 Nm torque mode to read boot encoders
  // ----------------------------------------------------------
#if !ROS_BRIDGE_ENABLED
  Serial.println("Waking motors in 0 Nm Torque Mode to read encoders...");
#endif
  for (int id = 1; id <= 6; id++) {
    uint8_t torque_mode_data[8] = {1, 0, 0, 0, 1, 0, 0, 0};
    can_bus_transmit_frame(id, 0x0B, torque_mode_data, 8);
    delay(10);
    uint8_t enable_data[8] = {8, 0, 0, 0, 0, 0, 0, 0};
    can_bus_transmit_frame(id, 0x07, enable_data, 8);
    delay(10);
    uint8_t zero_torque[4] = {0, 0, 0, 0};
    can_bus_transmit_frame(id, 0x0E, zero_torque, 4);
  }

  // Flush CAN RX buffers so encoder positions are fresh readings
#if !ROS_BRIDGE_ENABLED
  Serial.print("Waiting for encoder buffers to populate... ");
#endif
  unsigned long flush_start = millis();
  while (millis() - flush_start < 500) {
    can_bus_poll();
    delay(5);
  }
#if !ROS_BRIDGE_ENABLED
  Serial.println("Done.");
#endif

  // Capture the physical boot encoder positions.
  MotorFeedback final_init_fb = can_bus_get_feedback();
  boot_pos_l_knee_deg  = final_init_fb.left_knee_pos_deg;
  boot_pos_r_knee_deg  = final_init_fb.right_knee_pos_deg;
  boot_pos_l_torso_deg = final_init_fb.left_torso_pos_deg;
  boot_pos_r_torso_deg = final_init_fb.right_torso_pos_deg;

#if !ROS_BRIDGE_ENABLED
  Serial.println("\n--- TRUE BOOT ENCODER VALUES ---");
  Serial.printf("L_Knee:  %.2f deg\n", boot_pos_l_knee_deg);
  Serial.printf("R_Knee:  %.2f deg\n", boot_pos_r_knee_deg);
  Serial.printf("L_Torso: %.2f deg\n", boot_pos_l_torso_deg);
  Serial.printf("R_Torso: %.2f deg\n", boot_pos_r_torso_deg);
  Serial.println("-------------------------------");
#endif

  // ----------------------------------------------------------
  // [3/3] Motors, estimator, controller, kinematics
  // ----------------------------------------------------------
#if !ROS_BRIDGE_ENABLED
  Serial.print("[3/3] Initializing Motors... ");
#endif
  bool motors_ok = motors_init();
#if !ROS_BRIDGE_ENABLED
  Serial.println(motors_ok ? "OK" : "FAILED!");
#endif

  estimator_init();
  lqr_controller_init();
  control_manager_init();
  kinematics_init();

  // Compute the IK offsets from the boot stand height
#if !ROS_BRIDGE_ENABLED
  Serial.print("Calibrating IK Offsets from Boot Stand (435.75mm)... ");
#endif
  JointAngles boot_angles = kinematics_compute(BOOT_HEIGHT_MM);
  if (boot_angles.valid) {
    knee_offset_deg  = boot_angles.knee_angle_deg;
    torso_offset_deg = boot_angles.torso_angle_deg;
#if !ROS_BRIDGE_ENABLED
    Serial.println("OK");
#endif
  } else {
#if !ROS_BRIDGE_ENABLED
    Serial.println("FAILED! Boot height is physically impossible.");
#endif
    while (true) delay(100);
  }

  // ----------------------------------------------------------
  // Final hardware check before enabling
  // ----------------------------------------------------------
  if (!imu_ok || !motors_ok) {
#if !ROS_BRIDGE_ENABLED
    Serial.println("\n[CRITICAL ERROR] Hardware check failed. Halting system.");
#endif
    while (true) delay(1000);
  }

#if !ROS_BRIDGE_ENABLED
  Serial.println("\nHardware Check Passed. Enabling Motors (Closed Loop)...");
  Serial.println("Enabling motors...");
#endif
  motors_enable();
  delay(500);

  estimator_init();
  control_manager_init();

  last_loop_us = micros();

#if !ROS_BRIDGE_ENABLED
  Serial.println("==================================");
  Serial.println("   ENTERING MAIN CONTROL LOOP     ");
  Serial.println("==================================");
#endif
}

// ============================================================
// loop()
// ============================================================
void loop() {
  // ----------------------------------------------------------
  // [A] CAN polling  (runs every iteration, not rate-limited)
  // ----------------------------------------------------------
  // THIS MUST BE OUTSIDE OF THE ROS #IFDEF SO IT ALWAYS RUNS!
  can_bus_poll();
  can_bus_check_alerts();

  // ----------------------------------------------------------
  // [B] ROS bridge update  (runs every iteration)
  // ----------------------------------------------------------
#if ROS_BRIDGE_ENABLED
  ros_bridge_update();
#endif

  // ----------------------------------------------------------
  // [C] Rate limiter – skip the rest until next control tick
  // ----------------------------------------------------------
  unsigned long now = millis();
  unsigned long now_us = micros();
  
  static uint32_t loop_count = 0;
  static unsigned long last_hz_print = 0;

  loop_count++;
  if ((now - last_hz_print) > 1000) {
    #if !ROS_BRIDGE_ENABLED
    Serial.printf("Actual loop Hz: %u\n", loop_count);
    #endif
    loop_count = 0;
    last_hz_print = now;
  }

  if ((now_us - last_loop_us) < LOOP_INTERVAL_US) return;
  float dt = (now_us - last_loop_us) / 1e6f;
  last_loop_us = now_us;

  // ----------------------------------------------------------
  // [D] Sensing – IMU and motor feedback
  // ----------------------------------------------------------
  imu_set_current_height_mm(current_height_mm);
  imu_update(dt);
  ImuData      imu = imu_get_data();
  MotorFeedback fb = can_bus_get_feedback();

  // ----------------------------------------------------------
  // [E] Publish encoder feedback to ROS (test mode only)
  // ----------------------------------------------------------
#if ROS_BRIDGE_ENABLED
  ros_bridge_publish_feedback(fb);
#endif

  // ----------------------------------------------------------
  // [F] State estimation
  // ----------------------------------------------------------
  estimator_update(dt, imu, fb);
  RobotState state = estimator_get_state();

  // ----------------------------------------------------------
  // [G] Safety checks
  // ----------------------------------------------------------
  bool is_upright     = safety_angle_ok_deg(rad2deg(state.angle_rad));
  bool motors_healthy = safety_motors_ok(fb, now);
  bool safety_stop    = !is_upright || !motors_healthy;

  // ----------------------------------------------------------
  // [H] Command acquisition
  // ----------------------------------------------------------
  float forward_cmd = 0.0f;
  float turn_cmd    = 0.0f;
  bool  r3_pressed  = false;

  bool has_ros_cmd = false;
#if ROS_BRIDGE_ENABLED
  ros_bridge_get_commands(forward_cmd, turn_cmd, has_ros_cmd);
#endif
  if (!has_ros_cmd) {
    forward_cmd = 0.0f;
    turn_cmd    = 0.0f;
  }

  bool ps4_override = false;
  if (PS4.isConnected()) {
    int ly = PS4.LStickY();
    int rx = PS4.RStickX();
    bool up_pressed    = PS4.Up();
    bool down_pressed  = PS4.Down();
    bool share_pressed = PS4.Share();
    r3_pressed         = PS4.R3();

    if (share_pressed && !last_share_pressed) {
      standby_mode = !standby_mode;
      ps4_override = true;
    }
    last_share_pressed = share_pressed;

    float ps4_forward_cmd = 0.0f;
    float ps4_turn_cmd    = 0.0f;

    if (abs(ly) > 15) {
      ps4_forward_cmd = ly / 127.0f;
      ps4_override = true;
    }
    if (abs(rx) > 15) {
      ps4_turn_cmd = rx / 127.0f;
      ps4_override = true;
    }

    if (up_pressed) {
      if (!has_taken_off) {
        standby_mode  = false;
        has_taken_off = true;
      }
      target_height_mm += 50.0f * dt;
      ps4_override = true;
    }

    if (down_pressed) {
      target_height_mm -= 50.0f * dt;
      ps4_override = true;
    }

    target_height_mm = clampf(target_height_mm, 330.0f, 490.0f);

    if (ps4_override) {
      forward_cmd = ps4_forward_cmd;
      turn_cmd    = ps4_turn_cmd;
    }
  } else {
    last_share_pressed = false;
  }

  // ----------------------------------------------------------
  // [I] Height trajectory – smooth low-pass filter
  // ----------------------------------------------------------
  current_height_mm += (target_height_mm - current_height_mm) * 8.0f * dt;

  // ----------------------------------------------------------
  // [J] Control output
  // ----------------------------------------------------------
  ControlOutput out = {};

  if (safety_stop) {
    out.wheels.left_torque  = 0.0f;
    out.wheels.right_torque = 0.0f;
    motors_stop();

  } else {
    out = control_manager_update(dt, state, forward_cmd, turn_cmd,
                                  r3_pressed, current_height_mm, standby_mode);

    JointAngles joint_targets = kinematics_compute(current_height_mm);
    if (joint_targets.valid) {
      float relative_knee_deg  = joint_targets.knee_angle_deg  - knee_offset_deg;
      float relative_torso_deg = joint_targets.torso_angle_deg - torso_offset_deg;

      float l_knee_ik_deg  = -relative_knee_deg;
      float r_knee_ik_deg  =  relative_knee_deg;
      float l_torso_ik_deg = -relative_torso_deg;
      float r_torso_ik_deg =  relative_torso_deg;

      float final_l_knee_deg  = boot_pos_l_knee_deg  + l_knee_ik_deg;
      float final_r_knee_deg  = boot_pos_r_knee_deg  + r_knee_ik_deg;
      float final_l_torso_deg = boot_pos_l_torso_deg + l_torso_ik_deg;
      float final_r_torso_deg = boot_pos_r_torso_deg + r_torso_ik_deg;

      motors_set_leg_positions_deg(final_l_knee_deg, final_r_knee_deg,
                                    final_l_torso_deg, final_r_torso_deg);
    }

    motors_set_wheel_torque(out.wheels.left_torque, out.wheels.right_torque);
  }

  // ----------------------------------------------------------
  // [L] Serial status print (1 Hz, human-readable)
  // ----------------------------------------------------------
  static unsigned long last_print = 0;
  if ((now - last_print) > 1000) {
    last_print = now;

#if !ROS_BRIDGE_ENABLED
    if (safety_stop) {
      if (!is_upright)
        Serial.printf("SAFETY STOP: Tipped over! Pitch: %.1f deg\n",
                      rad2deg(state.angle_rad));
      if (!motors_healthy)
        Serial.println("SAFETY STOP: A Motor Faulted OR Disconnected (Timeout)!");
    } else if (standby_mode) {
      Serial.printf("STANDBY MODE | Wheels Free, Legs Locked | "
                    "Pitch: %.1f deg | Ht: %d mm\n",
                    rad2deg(state.angle_rad), (int)current_height_mm);
    } else {
      Serial.printf("BALANCING | Pitch: %.1f deg | Yaw: %5.1f deg | "
                    "Target Ht: %d mm | Current Ht: %d mm\n",
                    rad2deg(state.angle_rad), state.yaw_angle_deg,
                    (int)target_height_mm, (int)current_height_mm);
    }
#endif
  }

  // ----------------------------------------------------------
  // [K] UDP WiFi Logging
  // ----------------------------------------------------------
  wifi_logger_update(dt, imu, fb, state, out, forward_cmd, safety_stop);
}