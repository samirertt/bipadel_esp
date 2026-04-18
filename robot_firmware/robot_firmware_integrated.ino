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
//   logger.cpp          – WiFi UDP telemetry streamer
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
#include "logger.h"

// -- ROS 2 bridge (conditionally compiled) --
#if ROS_BRIDGE_ENABLED
  #include "ros_bridge.h"
#endif

// -- PS4 controller (used when NOT in ROS test mode) --
#if !ROS_BRIDGE_TEST_MODE
  #include <PS4Controller.h>
#endif

// ============================================================
// Timing
// ============================================================
static unsigned long last_loop_ms    = 0;
static const unsigned long LOOP_INTERVAL_MS = 1000 / LOOP_HZ;

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
// PS4 state (only used in non-ROS mode)
// ============================================================
#if !ROS_BRIDGE_TEST_MODE
static bool standby_mode      = true;   // Robot wakes up in standby
static bool has_taken_off     = false;  // Tracks first liftoff
static bool last_share_pressed = false;
#endif

// ============================================================
// setup()
// ============================================================
void setup() {
  Serial.begin(115200);
  delay(2000);

  // Suppress the ASCII banner when ROS is on the other end of the
  // serial port, so the ROS driver does not have to filter it.
#if !ROS_BRIDGE_TEST_MODE
  Serial.println("\n==================================");
  Serial.println("   ROBOT BOOT SEQUENCE STARTING   ");
  Serial.println("==================================");
#endif

  // ----------------------------------------------------------
  // [0/3] Bluetooth PS4 Controller
  // ----------------------------------------------------------
  // Skipped entirely in ROS test mode to avoid BT / WiFi
  // radio contention and to keep boot time short.
#if !ROS_BRIDGE_TEST_MODE
  Serial.print("[0/3] Initializing PS4 Bluetooth... ");
  if (PS4.begin("C4:5B:BE:91:C3:06")) Serial.println("OK");
  else                                 Serial.println("FAILED!");
#endif

  // ----------------------------------------------------------
  // [1/3] IMU
  // ----------------------------------------------------------
  Serial.print("[1/3] Initializing IMU... ");
  bool imu_ok = imu_init();
  Serial.println(imu_ok ? "OK" : "FAILED! Check I2C wiring");

  // ----------------------------------------------------------
  // [2/3] CAN Bus
  // ----------------------------------------------------------
  Serial.print("[2/3] Initializing CAN Bus... ");
  bool can_ok = can_bus_init();
  Serial.println(can_ok ? "OK" : "FAILED! Check TWAI pins");

  if (!can_ok) {
    Serial.println("\n[CRITICAL ERROR] CAN Hardware failed. Halting system.");
    while (true) delay(1000);
  }

  // ----------------------------------------------------------
  // ROS 2 Bridge initialisation
  // ----------------------------------------------------------
  // Must come after Serial.begin() and before any Serial traffic
  // that ROS might misparse.  The bridge takes over the same
  // Serial port that the boot messages use above.
#if ROS_BRIDGE_ENABLED
  ros_bridge_init(ROS_BRIDGE_BAUD);
  ros_bridge_set_timeout_ms(ROS_CMD_TIMEOUT_MS);
  // At this point the ROS hardware interface can connect and will
  // receive valid responses to 'b', 'e', 'r', 'm' commands.
#endif

  // ----------------------------------------------------------
  // Wait for all 6 motor heartbeats
  // ----------------------------------------------------------
  // Block until every motor node has sent at least one CAN frame.
  // Prevents sending position commands before the ODrives are ready.
  Serial.print("Waiting for all 6 motors to heartbeat... ");
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
  Serial.println("OK! All 6 Motors Verified.");

  // ----------------------------------------------------------
  // Wake motors in 0 Nm torque mode to read boot encoders
  // ----------------------------------------------------------
  // Bring all axes to closed-loop torque mode with zero torque so
  // their encoder feedback streams begin without moving the joints.
  Serial.println("Waking motors in 0 Nm Torque Mode to read encoders...");
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
  Serial.print("Waiting for encoder buffers to populate... ");
  unsigned long flush_start = millis();
  while (millis() - flush_start < 500) {
    can_bus_poll();
    delay(5);
  }
  Serial.println("Done.");

  // Capture the physical boot encoder positions.  These become the
  // IK reference frame so all subsequent leg commands are relative.
  MotorFeedback final_init_fb = can_bus_get_feedback();
  boot_pos_l_knee_deg  = final_init_fb.left_knee_pos_deg;
  boot_pos_r_knee_deg  = final_init_fb.right_knee_pos_deg;
  boot_pos_l_torso_deg = final_init_fb.left_torso_pos_deg;
  boot_pos_r_torso_deg = final_init_fb.right_torso_pos_deg;

  Serial.println("\n--- TRUE BOOT ENCODER VALUES ---");
  Serial.printf("L_Knee:  %.2f deg\n", boot_pos_l_knee_deg);
  Serial.printf("R_Knee:  %.2f deg\n", boot_pos_r_knee_deg);
  Serial.printf("L_Torso: %.2f deg\n", boot_pos_l_torso_deg);
  Serial.printf("R_Torso: %.2f deg\n", boot_pos_r_torso_deg);
  Serial.println("-------------------------------");

  // ----------------------------------------------------------
  // [3/3] Motors, estimator, controller, kinematics
  // ----------------------------------------------------------
  Serial.print("[3/3] Initializing Motors... ");
  bool motors_ok = motors_init();
  Serial.println(motors_ok ? "OK" : "FAILED!");

  estimator_init();
  lqr_controller_init();
  control_manager_init();
  kinematics_init();

  // Compute the IK offsets from the boot stand height so that
  // relative leg position commands are correctly zeroed.
  Serial.print("Calibrating IK Offsets from Boot Stand (435.75mm)... ");
  JointAngles boot_angles = kinematics_compute(BOOT_HEIGHT_MM);
  if (boot_angles.valid) {
    knee_offset_deg  = boot_angles.knee_angle_deg;
    torso_offset_deg = boot_angles.torso_angle_deg;
    Serial.println("OK");
  } else {
    Serial.println("FAILED! Boot height is physically impossible.");
    while (true) delay(100);
  }

  // ----------------------------------------------------------
  // Logger (WiFi UDP telemetry)
  // ----------------------------------------------------------
  logger_init();  // Non-fatal if WiFi is unavailable

  // ----------------------------------------------------------
  // Final hardware check before enabling
  // ----------------------------------------------------------
  if (!imu_ok || !motors_ok) {
    Serial.println("\n[CRITICAL ERROR] Hardware check failed. Halting system.");
    while (true) delay(1000);
  }

  Serial.println("\nHardware Check Passed. Enabling Motors (Closed Loop)...");
  motors_enable();

  last_loop_ms = millis();

#if !ROS_BRIDGE_TEST_MODE
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
  // Must be called as fast as possible to drain the RX queue
  // before the ODrive's 500 Hz heartbeat frames overflow it.
  can_bus_poll();
  can_bus_check_alerts();

  // ----------------------------------------------------------
  // [B] ROS bridge update  (runs every iteration)
  // ----------------------------------------------------------
  // Drains the Serial RX buffer and enforces the command watchdog.
  // Must run every loop, not only on control-cycle ticks, so that
  // ROS serial ACKs ('OK') are sent without lag.
#if ROS_BRIDGE_ENABLED
  ros_bridge_update();
#endif

  // ----------------------------------------------------------
  // [C] Rate limiter – skip the rest until next control tick
  // ----------------------------------------------------------
  unsigned long now = millis();
  if ((now - last_loop_ms) < LOOP_INTERVAL_MS) return;

  float dt = (now - last_loop_ms) / 1000.0f;
  last_loop_ms = now;

  // ----------------------------------------------------------
  // [D] Sensing – IMU and motor feedback
  // ----------------------------------------------------------
  imu_update(dt);
  ImuData      imu = imu_get_data();
  MotorFeedback fb = can_bus_get_feedback();

  // ----------------------------------------------------------
  // [E] Publish encoder feedback to ROS (test mode only)
  // ----------------------------------------------------------
  // Sends "F,<left_pos>,<left_vel>,<right_pos>,<right_vel>" so
  // the ros2_control joint_state_broadcaster can read odometry.
#if ROS_BRIDGE_TEST_MODE
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
  // Two mutually exclusive input paths produce the same
  // normalised forward_cmd / turn_cmd in [-1, 1].
  float forward_cmd = 0.0f;
  float turn_cmd    = 0.0f;
  bool  r3_pressed  = false;

#if ROS_BRIDGE_TEST_MODE
  // ---- ROS 2 path ----
  // ros_bridge_update() (called above in section B) has already
  // parsed any new serial commands.  We just read the result here.
  bool has_ros_cmd = false;
  ros_bridge_get_commands(forward_cmd, turn_cmd, has_ros_cmd);

  // If the watchdog fired (no command within timeout), zero commands.
  if (!has_ros_cmd) {
    forward_cmd = 0.0f;
    turn_cmd    = 0.0f;
  }

  // Standby mode is always false in ROS test mode; ROS controls
  // the robot directly without a standby concept.
  bool standby_mode = false;

#else
  // ---- PS4 controller path ----
  if (PS4.isConnected()) {
    int ly = PS4.LStickY();
    int rx = PS4.RStickX();
    r3_pressed = PS4.R3();

    // Share button toggles standby mode
    bool share_pressed = PS4.Share();
    if (share_pressed && !last_share_pressed) {
      standby_mode = !standby_mode;
    }
    last_share_pressed = share_pressed;

    // Dead-band ±15 on stick to prevent drift
    if (abs(ly) > 15) forward_cmd = ly / 127.0f;
    if (abs(rx) > 15) turn_cmd    = rx / 127.0f;

    // D-pad Up = take off and raise height
    if (PS4.Up()) {
      if (!has_taken_off) {
        standby_mode  = false;
        has_taken_off = true;
      }
      target_height_mm += 50.0f * dt;
    }

    // D-pad Down = lower height
    if (PS4.Down()) target_height_mm -= 50.0f * dt;

    target_height_mm = clampf(target_height_mm, 280.0f, 502.75f);
  }
#endif  // ROS_BRIDGE_TEST_MODE

  // ----------------------------------------------------------
  // [I] Height trajectory – smooth low-pass filter
  // ----------------------------------------------------------
  // Prevents step changes in leg IK targets; time constant ≈ 125 ms.
  current_height_mm += (target_height_mm - current_height_mm) * 8.0f * dt;

  // ----------------------------------------------------------
  // [J] Control output
  // ----------------------------------------------------------
  ControlOutput out = {};

  if (safety_stop) {
    // Safety stop: zero wheel torques and disable leg position hold
    out.wheels.left_torque  = 0.0f;
    out.wheels.right_torque = 0.0f;
    motors_stop();

  } else {
    // Normal operation: compute balance + yaw torques
    out = control_manager_update(dt, state, forward_cmd, turn_cmd,
                                  r3_pressed, current_height_mm, standby_mode);

    // Leg IK: convert target height to joint angles and apply
    JointAngles joint_targets = kinematics_compute(current_height_mm);
    if (joint_targets.valid) {
      // Relative offset from the boot stand IK solution
      float relative_knee_deg  = joint_targets.knee_angle_deg  - knee_offset_deg;
      float relative_torso_deg = joint_targets.torso_angle_deg - torso_offset_deg;

      // Sign convention: left side inverted to match robot geometry
      float l_knee_ik_deg  = -relative_knee_deg;
      float r_knee_ik_deg  =  relative_knee_deg;
      float l_torso_ik_deg = -relative_torso_deg;
      float r_torso_ik_deg =  relative_torso_deg;

      // Add the absolute boot encoder offset → absolute motor target
      float final_l_knee_deg  = boot_pos_l_knee_deg  + l_knee_ik_deg;
      float final_r_knee_deg  = boot_pos_r_knee_deg  + r_knee_ik_deg;
      float final_l_torso_deg = boot_pos_l_torso_deg + l_torso_ik_deg;
      float final_r_torso_deg = boot_pos_r_torso_deg + r_torso_ik_deg;

      motors_set_leg_positions_deg(final_l_knee_deg, final_r_knee_deg,
                                    final_l_torso_deg, final_r_torso_deg);
    }

    // Apply wheel torques
    motors_set_wheel_torque(out.wheels.left_torque, out.wheels.right_torque);
  }

  // ----------------------------------------------------------
  // [K] Telemetry logging (WiFi UDP, non-blocking)
  // ----------------------------------------------------------
  logger_log(now, dt, imu, fb, state, out, safety_stop, 0);

  // ----------------------------------------------------------
  // [L] Serial status print (1 Hz, human-readable)
  // ----------------------------------------------------------
  // In ROS_BRIDGE_TEST_MODE these prints are suppressed to avoid
  // polluting the serial line that ROS reads.
  static unsigned long last_print = 0;
  if ((now - last_print) > 1000) {
    last_print = now;

#if !ROS_BRIDGE_TEST_MODE
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
}
