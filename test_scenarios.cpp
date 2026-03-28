#include "test_scenarios.h"

#include <Arduino.h>

namespace {

TestScenarioMode g_mode = TestScenarioMode::OFF;

enum class ScenarioPhase {
  WAIT_START = 0,
  CMD_POSITIVE,
  HOLD_AFTER_POSITIVE,
  CMD_NEGATIVE,
  HOLD_AFTER_NEGATIVE,
  DONE
};

ScenarioPhase g_phase = ScenarioPhase::DONE;
unsigned long g_phase_start_ms = 0;

static constexpr unsigned long START_DELAY_MS = 5000;

static constexpr float CMD_POSITIVE_DPS = 0.0f;
static constexpr float CMD_NEGATIVE_DPS = 0.0f;

static constexpr unsigned long CMD_POSITIVE_TIME_MS = 500;
static constexpr unsigned long CMD_NEGATIVE_TIME_MS = 500;
static constexpr unsigned long HOLD_TIME_MS = 10000;

void enter_phase(ScenarioPhase phase, unsigned long now_ms) {
  g_phase = phase;
  g_phase_start_ms = now_ms;
}

}  // namespace

void test_scenarios_init() {
  g_mode = TestScenarioMode::OFF;
  g_phase = ScenarioPhase::DONE;
  g_phase_start_ms = 0;
}

void test_scenarios_set_mode(TestScenarioMode mode) {
  g_mode = mode;

  if (mode == TestScenarioMode::OFF) {
    g_phase = ScenarioPhase::DONE;
  } else {
    g_phase = ScenarioPhase::WAIT_START;
  }

  g_phase_start_ms = millis();
}

TestScenarioOutput test_scenarios_update(unsigned long now_ms, const RobotState& state) {
  (void)state;

  TestScenarioOutput out = {};
  out.forward_deg_s = 0.0f;
  out.active = (g_mode != TestScenarioMode::OFF);
  out.cmd_currently = (int)g_phase;

  if (g_mode == TestScenarioMode::OFF) {
    return out;
  }

  unsigned long elapsed = now_ms - g_phase_start_ms;

  switch (g_phase) {
    case ScenarioPhase::WAIT_START:
      out.forward_deg_s = 0.0f;
      if (elapsed >= START_DELAY_MS) {
        enter_phase(ScenarioPhase::CMD_POSITIVE, now_ms);
      }
      break;

    case ScenarioPhase::CMD_POSITIVE:
      out.forward_deg_s = CMD_POSITIVE_DPS;
      if (elapsed >= CMD_POSITIVE_TIME_MS) {
        enter_phase(ScenarioPhase::HOLD_AFTER_POSITIVE, now_ms);
      }
      break;

    case ScenarioPhase::HOLD_AFTER_POSITIVE:
      out.forward_deg_s = 0.0f;
      if (elapsed >= HOLD_TIME_MS) {
        enter_phase(ScenarioPhase::CMD_NEGATIVE, now_ms);
      }
      break;

    case ScenarioPhase::CMD_NEGATIVE:
      out.forward_deg_s = CMD_NEGATIVE_DPS;
      if (elapsed >= CMD_NEGATIVE_TIME_MS) {
        enter_phase(ScenarioPhase::HOLD_AFTER_NEGATIVE, now_ms);
      }
      break;

    case ScenarioPhase::HOLD_AFTER_NEGATIVE:
      out.forward_deg_s = 0.0f;
      if (elapsed >= HOLD_TIME_MS) {
        enter_phase(ScenarioPhase::DONE, now_ms);
        out.active = false;
      }
      break;

    case ScenarioPhase::DONE:
      out.forward_deg_s = 0.0f;
      out.active = false;
      break;
  }

  out.cmd_currently = (int)g_phase;
  return out;
}