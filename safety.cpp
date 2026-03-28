#include "safety.h"
#include "config.h"

bool safety_angle_ok_deg(float angle_deg) {
  return fabs(angle_deg) <= SAFE_ANGLE_DEG;
}
