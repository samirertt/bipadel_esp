#pragma once

#include "types.h"

bool imu_init();
void imu_update(float dt);
ImuData imu_get_data();

// Call this once per control tick BEFORE imu_update(dt) so the
// height-scheduled angle offset table (in config.h) is evaluated
// against the robot's current filtered height.  If never called,
// the offset defaults to the first entry of the table.
void imu_set_current_height_mm(float height_mm);
