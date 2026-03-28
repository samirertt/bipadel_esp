#include "config.h"
#include "utils.h"
#include "imu.h"
#include "can_bus.h"
#include "motors.h"
#include "estimator.h"
#include "control_manager.h"
#include "safety.h"
#include "logger.h"
#include "mode_manager.h"
#include "balance_controller.h"
#include "command_filter.h"

static unsigned long last_loop_ms = 0;
static const unsigned long loop_interval_ms = 1000 / LOOP_HZ;

void setup() {
  Serial.begin(115200);
  delay(1000);

  bool ok = true;

  ok &= imu_init();
  ok &= can_bus_init();
  ok &= motors_init();

  estimator_init();
  control_manager_init();
  mode_manager_init();
  balance_controller_init();
  command_filter_init();
  logger_init("Dronos", "Dr2025Kh", IPAddress(10,1,36,33), 4210);

  if (!ok) {
    while (true) {
      delay(1000);
    }
  }

  motors_enable();

  // START WITH PD FIRST
  balance_controller_set_type(BalanceControllerType::CASCADE_PID);

  last_loop_ms = millis();
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

  bool angle_safe = safety_angle_ok_deg(rad2deg(state.angle_rad));
  bool imu_ok = true;
  bool can_ok = true;

  // keep true for now, later replace with button/serial/RC
  bool arm_request = true;

  mode_manager_update(imu_ok, can_ok, angle_safe, arm_request);

  ControlOutput out = {};
  bool safety_stop = !angle_safe;

  if (safety_stop) {
    motors_stop();
  } else {
    out = control_manager_update(state, 0.0f);
    motors_set_wheel_torque(out.wheels.left_torque, out.wheels.right_torque);
  }
  logger_log(now, dt, imu, fb, state, out, safety_stop, (int)mode_manager_get());
}