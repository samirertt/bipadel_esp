#pragma once

// ============================================================
// ros_bridge.h
// ============================================================
// Public API for the ROS 2 serial bridge module.
//
// This module implements the simple ASCII serial protocol used
// by the ros2_control DiffDrive hardware interface.  It:
//   • Parses single-character commands from the ROS host.
//   • Converts the received wheel velocity targets into the
//     same normalised forward/turn commands used by the PS4
//     controller path, so the rest of the control stack is
//     command-source agnostic.
//   • Publishes encoder feedback back to ROS when requested.
//   • Implements a watchdog that zeroes commands if the host
//     stops sending within ROS_CMD_TIMEOUT_MS.
//
// Compile-time feature flags (defined in config.h):
//   ROS_BRIDGE_ENABLED       – include this module at all
//   ROS_BRIDGE_TEST_MODE     – use ROS commands in the main loop
//   ROS_BRIDGE_DEMO_FEEDBACK – simulate feedback without real CAN
// ============================================================

#include <Arduino.h>
#include "types.h"

// ---- Lifecycle ----

// Initialise the bridge.  Call once from setup() AFTER Serial is
// already started by the boot sequence.  The baudrate parameter
// must match ROS_BRIDGE_BAUD in config.h (passed in from main).
bool ros_bridge_init(unsigned long baudrate = 115200);

// ---- Per-loop update ----

// Drain the Serial RX buffer, parse any complete commands, and
// enforce the command watchdog.  Must be called every loop iteration
// BEFORE reading commands with ros_bridge_get_commands().
void ros_bridge_update();

// ---- Command accessors ----

// Returns the latest normalised drive commands in the same [-1,1]
// range as the PS4 joystick path.
//   forward_cmd : positive = forward
//   turn_cmd    : positive = turn right
//   has_ros_cmd : true if a command arrived within the timeout window
void ros_bridge_get_commands(float &forward_cmd, float &turn_cmd, bool &has_ros_cmd);

// ---- Configuration ----

// Override the watchdog timeout at runtime (default: ROS_CMD_TIMEOUT_MS).
void ros_bridge_set_timeout_ms(unsigned long timeout_ms);

// ---- Encoder reset flag ----

// Returns true if ROS sent an 'r' (reset encoders) command since the
// last call to ros_bridge_clear_reset_request().
bool ros_bridge_reset_requested();
void ros_bridge_clear_reset_request();

// ---- Telemetry ----

// Serialise and transmit a MotorFeedback packet to the ROS host.
// Format: "F,<left_pos_deg>,<left_vel_dps>,<right_pos_deg>,<right_vel_dps>\n"
// Call once per control cycle when ROS_BRIDGE_TEST_MODE is active.
void ros_bridge_publish_feedback(const MotorFeedback &fb);
