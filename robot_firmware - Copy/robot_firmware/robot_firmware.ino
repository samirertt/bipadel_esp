// ============================================================
// robot_firmware_integrated.ino
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
#include <PS4Controller.h>

// ============================================================
// Timing
// ============================================================
static unsigned long last_loop_us = 0;
static const unsigned long LOOP_INTERVAL_US = 1000000UL / LOOP_HZ; 

// ============================================================
// Boot posture & height trajectory
// ============================================================
static const float BOOT_HEIGHT_MM  = 435.75f;
static float target_height_mm      = BOOT_HEIGHT_MM;
static float current_height_mm     = BOOT_HEIGHT_MM; 

// ============================================================
// IK reference offsets (calibrated at boot)
// ============================================================
static float knee_offset_deg  = 0.0f;
static float torso_offset_deg = 0.0f;

static float boot_pos_l_knee_deg  = 0.0f;
static float boot_pos_r_knee_deg  = 0.0f;
static float boot_pos_l_torso_deg = 0.0f;
static float boot_pos_r_torso_deg = 0.0f;

// ============================================================
// PS4 state
// ============================================================
static bool standby_mode       = true;   
static bool has_taken_off      = false;  
static bool last_share_pressed = false;

// ============================================================
// setup()
// ============================================================
void setup() {
  Serial.begin(921600);
  delay(2000);

#if !ROS_BRIDGE_ENABLED
  Serial.println("\n==================================");
  Serial.println("   ROBOT BOOT SEQUENCE STARTING   ");
  Serial.println("==================================");
#endif

#if !ROS_BRIDGE_ENABLED
  Serial.print("[0/3] Initializing PS4 Bluetooth... ");
#endif
  bool ps4_ok = PS4.begin("C4:5B:BE:91:C3:06");
#if !ROS_BRIDGE_ENABLED
  Serial.println(ps4_ok ? "OK" : "FAILED!");
#endif

#if !ROS_BRIDGE_ENABLED
  Serial.print("[1/3] Initializing IMU... ");
#endif
  bool imu_ok = imu_init();
#if !ROS_BRIDGE_ENABLED
  Serial.println(imu_ok ? "OK" : "FAILED! Check I2C wiring");
#endif

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

#if !ROS_BRIDGE_ENABLED
  Serial.print("Connecting to WiFi for Telemetry... ");
  bool wifi_ok = wifi_logger_init(WIFI_SSID, WIFI_PASSWORD, TARGET_PC_IP, UDP_PORT);
  Serial.println(wifi_ok ? "OK" : "FAILED! (Running without WiFi)");
#endif

#if ROS_BRIDGE_ENABLED
  ros_bridge_init(ROS_BRIDGE_BAUD);
  ros_bridge_set_timeout_ms(ROS_CMD_TIMEOUT_MS);
#endif

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
  can_bus_poll();
  can_bus_check_alerts();

#if ROS_BRIDGE_ENABLED
  ros_bridge_update();
#endif

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

  imu_set_current_height_mm(current_height_mm);
  imu_update(dt);
  ImuData      imu = imu_get_data();
  MotorFeedback fb = can_bus_get_feedback();

#if ROS_BRIDGE_ENABLED
  ros_bridge_publish_feedback(fb);
#endif

  estimator_update(dt, imu, fb);
  RobotState state = estimator_get_state();

  bool is_upright     = safety_angle_ok_deg(rad2deg(state.angle_rad));
  bool motors_healthy = safety_motors_ok(fb, now);
  bool safety_stop    = !is_upright || !motors_healthy;

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
  static bool last_options_pressed = false;

  if (PS4.isConnected()) {
    int ly = PS4.LStickY();
    int rx = PS4.RStickX();
    bool up_pressed      = PS4.Up();
    bool down_pressed    = PS4.Down();
    bool share_pressed   = PS4.Share();
    bool options_pressed = PS4.Options();
    r3_pressed           = PS4.R3();

    // --- IMU Tare Trigger ---
    if (options_pressed && !last_options_pressed) {
      if (standby_mode) { 
        #if !ROS_BRIDGE_ENABLED
        Serial.println("Taring IMU mechanical pitch to flash memory...");
        #endif
        imu_tare_pitch();
      }
    }
    last_options_pressed = options_pressed;

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
    last_options_pressed = false;
  }

  current_height_mm += (target_height_mm - current_height_mm) * 8.0f * dt;

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

  wifi_logger_update(dt, imu, fb, state, out, forward_cmd, safety_stop);
}