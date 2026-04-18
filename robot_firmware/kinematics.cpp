#include "kinematics.h"
#include "utils.h"         
#include <Arduino.h>
#include <math.h>

static constexpr float L1 = 200.0f;
static constexpr float L2 = 200.0f;
static constexpr float D1 = 0.548208245f;
static constexpr float D2 = 0.606484005f;
static constexpr float ML1 = 314.0f;
static constexpr float ML2 = 296.0f;
static constexpr float M0 = 1126.0f;
static constexpr float M1 = 786.0f;
static constexpr float M2 = 4465.0f;
static constexpr float WHEEL_RADIUS = 102.75f;

static constexpr float M_TOTAL = M0 + ML1 + M1 + ML2 + M2;
static constexpr float W1 = (ML1 * D1 + M1 + ML2 + M2) * L1;
static constexpr float W2 = (ML2 * D2 + M2) * L2;
static constexpr float K_FACTOR = W1 / W2;

void kinematics_init() {}

JointAngles kinematics_compute(float target_height_mm) {
    JointAngles result = {0};
    result.valid = false;

    // Bulletproof IK Hardware Constraints
    target_height_mm = clampf(target_height_mm, 280.0f, 502.75f);

    float y_int = target_height_mm - WHEEL_RADIUS;

    if (y_int >= (L1 + L2 - 1e-4f)) {
        result.alpha_deg = 90.0f;
        result.knee_angle_deg = 0.0f;
        result.torso_angle_deg = 0.0f;
        result.com_y = WHEEL_RADIUS + ((W1 * 1.0f + W2 * 1.0f) / M_TOTAL);
        result.valid = true;
        return result;
    }

    float A = (L1 * L1) - (L2 * L2 * K_FACTOR * K_FACTOR);
    float B = -2.0f * y_int * L1;
    float C0 = (y_int * y_int) - (L2 * L2) + (L2 * L2 * K_FACTOR * K_FACTOR);
    float discriminant = B * B - 4.0f * A * C0;

    if (discriminant < 0.0f) return result; 

    float sqrtD = sqrtf(discriminant);
    float s1 = (-B + sqrtD) / (2.0f * A);
    float s2 = (-B - sqrtD) / (2.0f * A);
    float s = -1.0f;

    if (s2 >= 0.0f && s2 <= 1.0f) s = s2;
    else if (s1 >= 0.0f && s1 <= 1.0f) s = s1;

    if (s < 0.0f) return result;

    float alpha = asinf(s);
    float cosAlpha = sqrtf(1.0f - s * s);
    float C_val = -K_FACTOR * cosAlpha;

    if (fabsf(C_val) > 1.0f) return result; 

    float theta = acosf(C_val) - alpha;

    // Convert everything to degrees silently before returning
    result.alpha_deg       = alpha * 180.0f / PI;
    result.knee_angle_deg  = theta * 180.0f / PI;
    result.torso_angle_deg = (alpha + theta - (PI / 2.0f)) * 180.0f / PI; 
    result.com_y           = WHEEL_RADIUS + ((W1 * s + W2 * sinf(alpha + theta)) / M_TOTAL);
    result.valid           = true;

    return result;
}