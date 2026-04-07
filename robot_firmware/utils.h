#pragma once

#include <Arduino.h>

inline float deg2rad(float x) { return x * PI / 180.0f; }
inline float rad2deg(float x) { return x * 180.0f / PI; }

inline float clampf(float v, float min_v, float max_v) {
  if (v < min_v) return min_v;
  if (v > max_v) return max_v;
  return v;
}

inline float roundf_2(float x) {
  float scaled = x * 100.0f;

  if (scaled >= 0.0f)
    scaled = (int)(scaled + 0.5f);
  else
    scaled = (int)(scaled - 0.5f);

  return scaled / 100.0f;
}
inline float roundf_int(float x) {

  if (x >= 0.0f)
  return (int)(x + 0.5f);
  else
  return (int)(x - 0.5f);

}