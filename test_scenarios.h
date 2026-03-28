#pragma once

#include "types.h"

enum class TestScenarioMode {
  OFF = 0,
  STEP_HOLD_REVERSAL = 1
};

struct TestScenarioOutput {
  float forward_deg_s;
  bool active;
  int cmd_currently;
};

void test_scenarios_init();
void test_scenarios_set_mode(TestScenarioMode mode);
TestScenarioOutput test_scenarios_update(unsigned long now_ms, const RobotState& state);