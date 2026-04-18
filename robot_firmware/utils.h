#pragma once

// ============================================================
// utils.h
// ============================================================
// Inline math helper functions shared across all modules.
// Kept as a header-only file so there is no corresponding .cpp.
// ============================================================

#include <Arduino.h>

// Convert degrees to radians
inline float deg2rad(float x) { return x * PI / 180.0f; }

// Convert radians to degrees
inline float rad2deg(float x) { return x * 180.0f / PI; }

// Clamp v to the range [min_v, max_v]
inline float clampf(float v, float min_v, float max_v) {
  if (v < min_v) return min_v;
  if (v > max_v) return max_v;
  return v;
}

// Round to 2 decimal places (avoids printf floating-point overhead)
inline float roundf_2(float x) {
  float scaled = x * 100.0f;
  if (scaled >= 0.0f) scaled = (int)(scaled + 0.5f);
  else                scaled = (int)(scaled - 0.5f);
  return scaled / 100.0f;
}

// Round to nearest integer (as float)
inline float roundf_int(float x) {
  if (x >= 0.0f) return (int)(x + 0.5f);
  else           return (int)(x - 0.5f);
}
