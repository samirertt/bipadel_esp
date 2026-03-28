#include "command_filter.h"

void command_filter_init() {
}

float command_filter_limit(float target, float previous, float max_step) {
  float d = target - previous;

  if (d > max_step) d = max_step;
  if (d < -max_step) d = -max_step;

  return previous + d;
}