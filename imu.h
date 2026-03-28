#pragma once

#include "types.h"

bool imu_init();
void imu_update(float dt);
ImuData imu_get_data();