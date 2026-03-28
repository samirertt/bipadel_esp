#pragma once
#include "types.h"

enum class BalanceControllerType : uint8_t {
  CASCADE_PID = 0,
  LQR = 1
};

void balance_controller_init();
void balance_controller_set_type(BalanceControllerType type);
BalanceControllerType balance_controller_get_type();
float balance_controller_compute(const RobotState& state);