#include "config.h"
#include "utils.h"
#include "imu.h"
#include "can_bus.h"
#include "motors.h"
#include "estimator.h"
#include "lqr_controller.h"
#include "control_manager.h"
#include "safety.h"
#include <PS4Controller.h>

static unsigned long last_loop_ms = 0;
static const unsigned long loop_interval_ms = 1000 / LOOP_HZ;

void setup() {
  // INCREASED BAUD RATE: Crucial for 500Hz serial streaming without blocking
  Serial.begin(921600); 
  delay(2000);
  
  // You can keep the boot text; your Raspberry Pi parser should just ignore 
  // any lines that don't look like CSV data.
  Serial.println("BOOT_START");

  if (PS4.begin("C4:5B:BE:91:C3:06")) {
    Serial.println("PS4_OK");
  }

  bool imu_ok = imu_init();
  bool can_ok = can_bus_init();
  bool motors_ok = motors_init();

  estimator_init();
  lqr_controller_init();
  control_manager_init();
  
  if (!imu_ok || !can_ok || !motors_ok) {
    Serial.println("CRITICAL_ERROR");
    while (true) { delay(1000); }
  }

  motors_enable();
  last_loop_ms = millis();
  Serial.println("BOOT_COMPLETE");
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

  float forward_cmd = 0.0f;
  float turn_cmd = 0.0f;
  if (PS4.isConnected()) {
    int ly = PS4.LStickY(); 
    int rx = PS4.RStickX();

    if (abs(ly) > 15) forward_cmd = ly / 127.0f;
    if (abs(rx) > 15) turn_cmd = rx / 127.0f;    
  }
  
  ControlOutput out = {};
  if (safety_stop) {
    out.wheels.left_torque = 0.0f;
    out.wheels.right_torque = 0.0f;
    motors_stop();
  } else {
    out = control_manager_update(dt, state, forward_cmd, turn_cmd);
    motors_set_wheel_torque(out.wheels.left_torque, out.wheels.right_torque);
  }

  // ==========================================
  // RASPBERRY PI SERIAL TELEMETRY (CSV FORMAT)
  // Format: timestamp_ms, fused_angle, gyro_rate, left_pos, left_vel, right_pos, right_vel
  // ==========================================
  Serial.print(now);
  Serial.print(",");
  Serial.print(imu.fused_angle_deg, 2);
  Serial.print(",");
  Serial.print(imu.gyro_rate_dps, 2);
  Serial.print(",");
  Serial.print(fb.left_pos_deg, 2);
  Serial.print(",");
  Serial.print(fb.left_vel_dps, 2);
  Serial.print(",");
  Serial.print(fb.right_pos_deg, 2);
  Serial.print(",");
  Serial.println(fb.right_vel_dps, 2);
}