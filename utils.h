#pragma once
#include <Arduino.h>
#include <math.h>

inline float deg2rad(float deg) {
  return deg * 0.01745329252f;
}

inline float rad2deg(float rad) {
  return rad * 57.2957795131f;
}

inline float clampf(float x, float mn, float mx) {
  if (x < mn) return mn;
  if (x > mx) return mx;
  return x;
}