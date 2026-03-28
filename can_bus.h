#pragma once

#include "types.h"
#include <stdint.h>

bool can_bus_init();
void can_bus_poll();

void can_bus_transmit_frame(int node_id, int cmd_id, uint8_t* data, int len);

void can_bus_send_velocity(int node_id, float vel_deg_s);
void can_bus_send_torque(int node_id, float torque_nm);

MotorFeedback can_bus_get_feedback();