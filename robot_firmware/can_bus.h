#pragma once

// ============================================================
// can_bus.h
// ============================================================
// Low-level CAN / TWAI interface for the ODrive motor controllers.
//
// This module owns all direct hardware access to the ESP32's TWAI
// peripheral (ESP-IDF name for CAN).  It:
//   • Initialises the TWAI driver with a 500 kbps timing.
//   • Polls the RX queue and demultiplexes heartbeat and encoder
//     frames from up to 6 motor nodes.
//   • Exposes typed helper functions for the three ODrive command
//     types used by this firmware: torque, velocity, and position.
//   • Monitors TWAI alert flags for bus errors.
//
// Node ID assignments (CAN Simple protocol):
//   1 = Left  wheel    (velocity / torque mode)
//   2 = Right wheel    (velocity / torque mode)
//   3 = Left  knee     (position mode)
//   4 = Right knee     (position mode)
//   5 = Left  torso    (position mode)
//   6 = Right torso    (position mode)
// ============================================================

#include "types.h"
#include <Arduino.h>

// Lifecycle
bool         can_bus_init();

// Per-loop processing
void         can_bus_poll();         // Drain RX queue, update g_feedback
void         can_bus_check_alerts(); // Print TWAI driver warnings

// Raw frame transmit
void         can_bus_transmit_frame(int node_id, int cmd_id, uint8_t *data, int len);

// Typed command helpers (convert to ODrive CAN Simple frames internally)
void         can_bus_send_velocity(int node_id, float vel_value);   // cmd 0x0D
void         can_bus_send_torque  (int node_id, float torque_nm);   // cmd 0x0E
void         can_bus_send_position(int node_id, float pos_revs);    // cmd 0x0C

// Feedback accessor – returns a snapshot of the last decoded values
MotorFeedback can_bus_get_feedback();
